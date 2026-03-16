#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// TEASER++ Headers
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <teaser/fpfh.h>

// Livox Support
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class TeaserRelocNode : public rclcpp::Node
{
public:
    TeaserRelocNode()
        : Node("teaser_reloc_node")
    {
        // === 参数声明 ===
        
        // 1. 基础配置
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("pcl_type", "livox"); // "livox" or "standard"
        
        // 2. 降采样参数 (TEASER++ 对点数敏感，建议体素设大一点，例如 0.3 - 0.5)
        this->declare_parameter("map_voxel_leaf_size", 0.4);
        this->declare_parameter("cloud_voxel_leaf_size", 0.4);

        // 3. FPFH 特征参数 (关键几何约束: voxel < normal < feature)
        this->declare_parameter("fpfh_normal_radius", 0.6);   // 法向量搜索半径
        this->declare_parameter("fpfh_feature_radius", 0.9);  // 特征描述子搜索半径

        // 4. TEASER++ 求解器参数
        this->declare_parameter("noise_bound", 0.2);          // 预期噪声水平 (米)
        this->declare_parameter("solver_max_iter", 100);
        this->declare_parameter("rotation_gnc_factor", 1.4);
        this->declare_parameter("inlier_threshold", 30);      // 最小内点数阈值

        // === 获取参数 ===
        this->get_parameter("map_path", map_path_);
        this->get_parameter("map_frame_id", map_frame_id_);
        this->get_parameter("pcl_type", pcl_type_);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size_);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size_);
        this->get_parameter("fpfh_normal_radius", fpfh_normal_radius_);
        this->get_parameter("fpfh_feature_radius", fpfh_feature_radius_);
        this->get_parameter("noise_bound", noise_bound_);
        this->get_parameter("solver_max_iter", solver_max_iter_);
        this->get_parameter("rotation_gnc_factor", rotation_gnc_factor_);
        int inlier_thr;
        this->get_parameter("inlier_threshold", inlier_thr);
        inlier_threshold_ = static_cast<size_t>(inlier_thr);

        // === 初始化 ROS 接口 ===
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
        // 使用 Transient Local QoS 确保晚加入的 Rviz 也能收到地图
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "prior_map", rclcpp::QoS(1).transient_local().reliable());
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

#ifdef USE_LIVOX
        if (pcl_type_ == "livox") {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&TeaserRelocNode::lvx_cloud_callback, this, std::placeholders::_1));
        } else {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pointcloud2", 10, std::bind(&TeaserRelocNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&TeaserRelocNode::cloud_callback, this, std::placeholders::_1));
#endif
        // TEASER 不需要初值，但保留接口用于手动触发重定位（可选）
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&TeaserRelocNode::pose_callback, this, std::placeholders::_1));

        // === 加载并处理地图 ===
        load_and_process_map();
    }

private:
    // 加载地图并计算特征 (只运行一次)
    void load_and_process_map()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *temp_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not read file: %s", map_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map with %zu points", temp_cloud->size());

        // 1. 降采样地图
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(temp_cloud);
        sor.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
        sor.filter(*map_cloud_pcl_);
        RCLCPP_INFO(this->get_logger(), "Downsampled map to %zu points", map_cloud_pcl_->size());

        // 2. 转换为 TEASER 数据结构
        convert_pcl_to_teaser(map_cloud_pcl_, map_cloud_teaser_);

        // 3. 计算地图 FPFH 特征
        RCLCPP_INFO(this->get_logger(), "Start computing Map FPFH (this may take time)...");
        teaser::FPFHEstimation fpfh;
        // 自动计算法向量并计算特征
        map_descriptors_ = fpfh.computeFPFHFeatures(map_cloud_teaser_, fpfh_normal_radius_, fpfh_feature_radius_);
        RCLCPP_INFO(this->get_logger(), "Map FPFH features computed.");

        // 4. 发布地图
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud_pcl_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        map_pub_->publish(map_msg);
    }

    // 核心处理函数
    void process_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        if (map_cloud_teaser_.empty() || !map_descriptors_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map not ready yet.");
            return;
        }

        // 1. 降采样输入点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(cloud_voxel_leaf_size_, cloud_voxel_leaf_size_, cloud_voxel_leaf_size_);
        sor.filter(*src_cloud_pcl);

        if (src_cloud_pcl->size() < 50) {
            RCLCPP_WARN(this->get_logger(), "Input cloud too small: %zu", src_cloud_pcl->size());
            return;
        }

        // 2. 计算输入点云 FPFH
        teaser::PointCloud src_cloud_teaser;
        convert_pcl_to_teaser(src_cloud_pcl, src_cloud_teaser);

        teaser::FPFHEstimation fpfh;
        auto src_descriptors = fpfh.computeFPFHFeatures(src_cloud_teaser, fpfh_normal_radius_, fpfh_feature_radius_);

        // 3. 特征匹配
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_cloud_teaser, map_cloud_teaser_, *src_descriptors, *map_descriptors_, 
            false, true, false, 0.95);

        if (correspondences.size() < inlier_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Insufficient correspondences: %zu", correspondences.size());
            return;
        }

        // 4. TEASER++ 求解
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = noise_bound_;
        params.cbar2 = 1;
        params.estimate_scaling = false;
        params.rotation_max_iterations = solver_max_iter_;
        params.rotation_gnc_factor = rotation_gnc_factor_;
        params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold = 0.005;

        teaser::RobustRegistrationSolver solver(params);
        
        auto start = std::chrono::high_resolution_clock::now();
        solver.solve(src_cloud_teaser, map_cloud_teaser_, correspondences);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto solution = solver.getSolution();

        // 5. 结果验证与发布
        if (solution.valid)
        {
            // --- 修复点开始 ---
            // 从 solver 获取内点索引
            auto inliers = solver.getTranslationInliers();
            
            RCLCPP_INFO(this->get_logger(), "TEASER Solved in %.2f ms. Inliers: %zu", 
                std::chrono::duration<double, std::milli>(end - start).count(), inliers.size());
            // --- 修复点结束 ---

            // 发布 Pose
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = map_frame_id_;
            
            pose_msg.pose.pose.position.x = solution.translation.x();
            pose_msg.pose.pose.position.y = solution.translation.y();
            pose_msg.pose.pose.position.z = solution.translation.z();
            
            Eigen::Quaterniond q(solution.rotation);
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();

            for(int i=0; i<36; i++) pose_msg.pose.covariance[i] = 0.0;
            pose_msg.pose.covariance[0] = 0.01; 
            pose_msg.pose.covariance[7] = 0.01; 
            pose_msg.pose.covariance[35] = 0.01;

            publisher_->publish(pose_msg);

            // 发布变换后的点云用于可视化验证
            Eigen::Matrix4f final_transform = Eigen::Matrix4f::Identity();
            final_transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
            final_transform.block<3, 1>(0, 3) = solution.translation.cast<float>();

            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl::transformPointCloud(*src_cloud_pcl, transformed_cloud, final_transform);
            
            sensor_msgs::msg::PointCloud2 debug_msg;
            pcl::toROSMsg(transformed_cloud, debug_msg);
            debug_msg.header.stamp = this->now();
            debug_msg.header.frame_id = map_frame_id_;
            transformed_cloud_pub_->publish(debug_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "TEASER solution invalid.");
        }
    }

    // 辅助：PCL 转 TEASER
    void convert_pcl_to_teaser(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, teaser::PointCloud& teaser_cloud)
    {
        teaser_cloud.clear();
        teaser_cloud.reserve(pcl_cloud->size());
        for (const auto& p : *pcl_cloud) {
            teaser_cloud.push_back({p.x, p.y, p.z});
        }
    }

    // ROS 消息回调
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        process_cloud(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        input_cloud->reserve(msg->point_num);
        for (uint i = 0; i < msg->point_num; i++)
        {
            // 简单过滤原点附近的噪点
            if (std::abs(msg->points[i].x) < 0.01 && std::abs(msg->points[i].y) < 0.01) continue;
            
            input_cloud->push_back(pcl::PointXYZ(msg->points[i].x, msg->points[i].y, msg->points[i].z));
        }
        process_cloud(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received InitialPose, but TEASER++ is performing Global Registration. "
                                        "Triggering re-computation...");
        // 可以在这里加逻辑：只有收到 manual pose 时才进行一次 TEASER 配准，节省计算资源
    }

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // 数据存储
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
    teaser::PointCloud map_cloud_teaser_;
    std::shared_ptr<teaser::FPFHCloud> map_descriptors_;

    // 参数
    std::string map_path_, map_frame_id_, pcl_type_;
    double map_voxel_leaf_size_, cloud_voxel_leaf_size_;
    double fpfh_normal_radius_, fpfh_feature_radius_, noise_bound_, rotation_gnc_factor_;
    int solver_max_iter_;
    size_t inlier_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeaserRelocNode>());
    rclcpp::shutdown();
    return 0;
}
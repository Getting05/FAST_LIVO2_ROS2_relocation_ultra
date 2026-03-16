#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 引入 small_gicp 头文件
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp> // OpenMP 加速降采样

#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class SmallGICPNode : public rclcpp::Node
{
public:
    SmallGICPNode()
        : Node("small_gicp_node")
    {
        // 声明参数
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        // small_gicp 通常不需要那么多迭代次数就能收敛，默认给 30-50 足够
        this->declare_parameter("solver_max_iter", 50); 
        this->declare_parameter("num_threads", 4); // 新增：线程数
        this->declare_parameter("max_correspondence_distance", 0.5); // 稍微调大一点，VGICP 鲁棒性更好
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("fitness_score_thre", 0.1); // 这里的 score 定义可能与 PCL 略有不同，建议根据实际情况调整
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type", "livox");
        this->declare_parameter("registration_type", "VGICP"); // 可选: VGICP, GICP

        // 获取参数
        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("num_threads", num_threads);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("fitness_score_thre", fitness_score_thre);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        std::string reg_type;
        this->get_parameter("registration_type", reg_type);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

        // 初始化订阅者
#ifdef USE_LIVOX
        if (pcl_type == "livox")
        {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&SmallGICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        }
        else
        {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pointcloud2", 10, std::bind(&SmallGICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&SmallGICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&SmallGICPNode::pose_callback, this, std::placeholders::_1));

        // 初始化初值
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_a);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", initial_x, initial_y, initial_z, initial_a);

        // 加载并处理地图
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *target_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", map_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu data points from map", target_cloud_->size());

        // 1. 使用 small_gicp 的 OpenMP 加速降采样
        target_cloud_ = small_gicp::voxelgrid_sampling_omp(*target_cloud_, map_voxel_leaf_size, num_threads);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud to %zu points", target_cloud_->size());

        // 发布处理后的地图
        pcl::toROSMsg(*target_cloud_, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        map_pub_->publish(target_cloud_msg);

        // 2. 配置 small_gicp 配准器
        // 设置为成员变量，避免每次回调都重建 Target KdTree，这是大幅提升性能的关键
        reg_.setRegistrationType(reg_type); // 推荐 VGICP
        reg_.setNumThreads(num_threads);
        reg_.setMaxCorrespondenceDistance(max_correspondence_distance);
        reg_.setCorrespondenceRandomness(20); // VGICP/GICP 邻域搜索点数
        // 注意：GICP/VGICP 不需要像 PCL 那样设置 RANSAC，它们通过稳健核函数处理外点
        // reg_.setVoxelResolution(1.0); // 如果使用 VGICP，可以调整体素分辨率，默认 1.0 通常可以

        // 设置目标点云 (会自动构建 KdTree)
        reg_.setInputTarget(target_cloud_);
    }

private:
    // 通用处理逻辑，供不同回调调用
    void process_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        if (target_cloud_->empty()) return;

        // 1. 降采样 Input Cloud
        input_cloud = small_gicp::voxelgrid_sampling_omp(*input_cloud, cloud_voxel_leaf_size, num_threads);
        // RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %zu points", input_cloud->size());

        // 2. 设置 Source (small_gicp 会自动计算协方差)
        reg_.setInputSource(input_cloud);
        
        // small_gicp 的 RegistrationPCL 接口类似 PCL，但在 align 时可以直接传入 Matrix4f 作为 hint
        // 注意：Align 实际上会修改传入的 cloud (input_cloud)，将其变换到目标位置
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud; 
        
        // 3. 执行配准
        // small_gicp::RegistrationPCL::align 接受 output cloud 和 initial guess
        reg_.align(aligned_cloud, initGuess);

        // 4. 获取结果
        bool converged = reg_.hasConverged();
        double fitness_score = reg_.getFitnessScore();
        Eigen::Matrix4f final_transform = reg_.getFinalTransformation();

        RCLCPP_INFO(this->get_logger(), "GICP fitness: %.4f, Converged: %d", fitness_score, converged);

        if (converged && fitness_score < fitness_score_thre)
        {
            converged_count++;
            RCLCPP_INFO(this->get_logger(), "Converged count: %d / %d", converged_count, converged_count_thre);

            if (converged_count > converged_count_thre)
            {
                RCLCPP_INFO(this->get_logger(), "Localization SUCCESS! Publishing pose and shutting down.");

                // 发布 Pose
                geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = map_frame;
                
                pose_msg.pose.pose.position.x = final_transform(0, 3);
                pose_msg.pose.pose.position.y = final_transform(1, 3);
                pose_msg.pose.pose.position.z = final_transform(2, 3);

                Eigen::Matrix3f rotation = final_transform.block<3, 3>(0, 0);
                Eigen::Quaternionf q(rotation);
                pose_msg.pose.pose.orientation.x = q.x();
                pose_msg.pose.pose.orientation.y = q.y();
                pose_msg.pose.pose.orientation.z = q.z();
                pose_msg.pose.pose.orientation.w = q.w();
                
                // 设置一些协方差 (可选，表示置信度很高)
                for(int i=0; i<36; i++) pose_msg.pose.covariance[i] = 0.0;
                pose_msg.pose.covariance[0] = 0.01; // x
                pose_msg.pose.covariance[7] = 0.01; // y
                pose_msg.pose.covariance[35] = 0.01; // theta

                publisher_->publish(pose_msg);

                // 发布变换后的点云用于可视化
                // aligned_cloud 已经是变换后的了
                sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
                pcl::toROSMsg(aligned_cloud, transformed_cloud_msg);
                transformed_cloud_msg.header.stamp = this->now();
                transformed_cloud_msg.header.frame_id = map_frame;
                transformed_cloud_pub_->publish(transformed_cloud_msg);

                // 任务完成，退出节点 (保留原本逻辑)
                rclcpp::shutdown();
                return;
            }
            else
            {
                // 收敛但次数不够，更新初值用于下一帧追踪
                initGuess = final_transform;
            }
        }
        else
        {
            // 发散或分数太低
            converged_count = 0;
            // 策略：如果完全失败，保持原来的 initGuess 不动，或者依据里程计更新(这里没有odom，暂时不动)
            // 但为了可视化，我们把点云按当前 initGuess 变换一下发出去
            if(converged && fitness_score >= fitness_score_thre) {
                 // 如果算法认为收敛了但分数很高，可能只是找到了局部最优，还是更新一下试试，防止卡死
                 initGuess = final_transform;
                 RCLCPP_WARN(this->get_logger(), "Converged but High Score (%.3f > %.3f)", fitness_score, fitness_score_thre);
            } else {
                 RCLCPP_WARN(this->get_logger(), "Not Converged!");
            }
        }

        // 无论是否成功，都发布一下当前的匹配状态用于调试
        pcl::PointCloud<pcl::PointXYZ> debug_cloud;
        pcl::transformPointCloud(*input_cloud, debug_cloud, initGuess); // 使用当前的猜测位置变换
        sensor_msgs::msg::PointCloud2 debug_cloud_msg;
        pcl::toROSMsg(debug_cloud, debug_cloud_msg);
        debug_cloud_msg.header.stamp = this->now();
        debug_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(debug_cloud_msg);

        // 定时发布地图，防止 Rviz 中丢失
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

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
        for (int i = 0; i < (int)msg->point_num; i++)
        {
            // 过滤掉极其靠近原点的无效点
            if (std::abs(msg->points[i].x) < 0.01 && std::abs(msg->points[i].y) < 0.01 && std::abs(msg->points[i].z) < 0.01) continue;
            
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        process_cloud(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        
        // 手动重置状态
        converged_count = 0;
        
        double r, p, yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Manual Initial Pose Received: \n x: %f, y: %f, z: %f, yaw: %f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
    }

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    sensor_msgs::msg::PointCloud2 target_cloud_msg;

    // 核心修改：使用 small_gicp 的 PCL 接口
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg_;

    Eigen::Matrix4f initGuess;
    double initial_x, initial_y, initial_z, initial_a;
    int solver_max_iter, num_threads;
    double max_correspondence_distance;
    std::string map_path, map_frame;
    double fitness_score_thre;
    double map_voxel_leaf_size, cloud_voxel_leaf_size;
    int converged_count = 0;
    int converged_count_thre;
    std::string pcl_type;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmallGICPNode>());
    rclcpp::shutdown();
    return 0;
}
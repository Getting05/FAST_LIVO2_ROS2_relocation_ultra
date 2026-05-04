#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

#include <teaser/fpfh.h>
#include <teaser/matcher.h>
#include <teaser/registration.h>

using namespace std::chrono_literals;

namespace
{
enum class RelocState
{
  ACCUMULATING,
  TEASER_INIT,
  GICP_REFINE,
  LOCKED
};

const char *state_name(const RelocState state)
{
  switch (state)
  {
    case RelocState::ACCUMULATING:
      return "ACCUMULATING";
    case RelocState::TEASER_INIT:
      return "TEASER_INIT";
    case RelocState::GICP_REFINE:
      return "GICP_REFINE";
    case RelocState::LOCKED:
      return "LOCKED";
  }
  return "UNKNOWN";
}

double rotation_delta_deg(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  const Eigen::Matrix3f r_delta = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const float trace = std::max(-1.0f, std::min(3.0f, r_delta.trace()));
  const float cos_angle = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f));
  return std::acos(cos_angle) * kRadToDeg;
}

double translation_delta(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}
}  // namespace

class MapTeaserGicpNode : public rclcpp::Node
{
public:
  MapTeaserGicpNode()
  : Node("map_teaser_gicp_node")
  {
    declare_parameters();
    read_parameters();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
    prior_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "prior_map", rclcpp::QoS(1).transient_local().reliable());
    transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("relocalization_status", 10);

    source_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      source_cloud_topic_, 10, std::bind(&MapTeaserGicpNode::source_cloud_callback, this, std::placeholders::_1));

    if (!load_map())
    {
      RCLCPP_ERROR(this->get_logger(), "Map loading failed. The node will stay idle.");
      map_ready_ = false;
      return;
    }

    init_teaser();
    init_gicp();
    map_ready_ = true;

    const auto timer_ms = std::max(100, static_cast<int>(registration_period_sec_ * 1000.0));
    registration_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_ms), std::bind(&MapTeaserGicpNode::registration_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "MapTeaserGicpNode ready. Accumulating %.2f seconds from %s.",
      accumulate_duration_sec_,
      source_cloud_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    this->declare_parameter<std::string>("map_path", "");
    this->declare_parameter<std::string>("source_cloud_topic", "/cloud_registered");
    this->declare_parameter<std::string>("source_frame_id", "camera_init");
    this->declare_parameter<std::string>("map_frame_id", "map");

    this->declare_parameter<double>("accumulate_duration_sec", 10.0);
    this->declare_parameter<double>("registration_period_sec", 1.0);
    this->declare_parameter<int>("max_accumulated_points", 300000);

    this->declare_parameter<double>("map_voxel_leaf_size", 0.4);
    this->declare_parameter<double>("cloud_voxel_leaf_size", 0.4);
    this->declare_parameter<double>("gicp_map_voxel_leaf_size", 0.2);
    this->declare_parameter<double>("gicp_cloud_voxel_leaf_size", 0.1);

    this->declare_parameter<double>("fpfh_normal_radius", 0.8);
    this->declare_parameter<double>("fpfh_feature_radius", 1.2);
    this->declare_parameter<double>("noise_bound", 0.3);
    this->declare_parameter<int>("teaser_solver_max_iter", 100);
    this->declare_parameter<double>("rotation_gnc_factor", 1.4);
    this->declare_parameter<int>("teaser_inlier_threshold", 5);
    this->declare_parameter<int>("teaser_success_count", 3);

    this->declare_parameter<int>("gicp_solver_max_iter", 50);
    this->declare_parameter<int>("num_threads", 4);
    this->declare_parameter<double>("max_correspondence_distance", 5.0);
    this->declare_parameter<double>("publish_fitness_score_thre", 0.5);
    this->declare_parameter<double>("final_fitness_score_thre", 0.2);
    this->declare_parameter<double>("stable_translation_thre", 0.05);
    this->declare_parameter<double>("stable_rotation_thre_deg", 1.0);
    this->declare_parameter<int>("stable_count_thre", 3);
    this->declare_parameter<std::string>("registration_type", "VGICP");
  }

  void read_parameters()
  {
    this->get_parameter("map_path", map_path_);
    this->get_parameter("source_cloud_topic", source_cloud_topic_);
    this->get_parameter("source_frame_id", source_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);

    this->get_parameter("accumulate_duration_sec", accumulate_duration_sec_);
    this->get_parameter("registration_period_sec", registration_period_sec_);
    this->get_parameter("max_accumulated_points", max_accumulated_points_);

    this->get_parameter("map_voxel_leaf_size", teaser_map_voxel_);
    this->get_parameter("cloud_voxel_leaf_size", teaser_cloud_voxel_);
    this->get_parameter("gicp_map_voxel_leaf_size", gicp_map_voxel_);
    this->get_parameter("gicp_cloud_voxel_leaf_size", gicp_cloud_voxel_);

    this->get_parameter("fpfh_normal_radius", fpfh_normal_radius_);
    this->get_parameter("fpfh_feature_radius", fpfh_feature_radius_);
    this->get_parameter("noise_bound", noise_bound_);
    this->get_parameter("teaser_solver_max_iter", teaser_solver_max_iter_);
    this->get_parameter("rotation_gnc_factor", rotation_gnc_factor_);
    int teaser_inlier_threshold = 0;
    this->get_parameter("teaser_inlier_threshold", teaser_inlier_threshold);
    teaser_inlier_threshold_ = static_cast<size_t>(std::max(0, teaser_inlier_threshold));
    this->get_parameter("teaser_success_count", teaser_success_count_thre_);

    this->get_parameter("gicp_solver_max_iter", gicp_solver_max_iter_);
    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("max_correspondence_distance", max_correspondence_distance_);
    this->get_parameter("publish_fitness_score_thre", publish_fitness_score_thre_);
    this->get_parameter("final_fitness_score_thre", final_fitness_score_thre_);
    this->get_parameter("stable_translation_thre", stable_translation_thre_);
    this->get_parameter("stable_rotation_thre_deg", stable_rotation_thre_deg_);
    this->get_parameter("stable_count_thre", stable_count_thre_);
    this->get_parameter("registration_type", registration_type_);

    accumulate_duration_sec_ = std::max(0.1, accumulate_duration_sec_);
    registration_period_sec_ = std::max(0.1, registration_period_sec_);
    num_threads_ = std::max(1, num_threads_);
    max_accumulated_points_ = std::max(1000, max_accumulated_points_);
    teaser_success_count_thre_ = std::max(1, teaser_success_count_thre_);
    stable_count_thre_ = std::max(1, stable_count_thre_);

    if (map_path_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Parameter map_path is empty.");
    }

    if (gicp_solver_max_iter_ > 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "gicp_solver_max_iter=%d is accepted for configuration compatibility; the current small_gicp PCL wrapper uses its internal iteration default.",
        gicp_solver_max_iter_);
    }
  }

  bool load_map()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *raw_map) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not read prior map: %s", map_path_.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded prior map with %zu points", raw_map->size());

    pcl::VoxelGrid<pcl::PointXYZ> teaser_filter;
    teaser_filter.setInputCloud(raw_map);
    teaser_filter.setLeafSize(teaser_map_voxel_, teaser_map_voxel_, teaser_map_voxel_);
    teaser_filter.filter(*teaser_map_pcl_);

    gicp_map_pcl_ = small_gicp::voxelgrid_sampling_omp(*raw_map, gicp_map_voxel_, num_threads_);

    RCLCPP_INFO(
      this->get_logger(),
      "Prior map downsampled: TEASER=%zu, GICP=%zu",
      teaser_map_pcl_->size(),
      gicp_map_pcl_->size());

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*gicp_map_pcl_, map_msg);
    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = map_frame_id_;
    prior_map_pub_->publish(map_msg);
    return !teaser_map_pcl_->empty() && !gicp_map_pcl_->empty();
  }

  void init_teaser()
  {
    convert_pcl_to_teaser(teaser_map_pcl_, teaser_map_cloud_);
    teaser::FPFHEstimation fpfh;
    RCLCPP_INFO(this->get_logger(), "Computing prior-map FPFH features...");
    teaser_map_descriptors_ = fpfh.computeFPFHFeatures(teaser_map_cloud_, fpfh_normal_radius_, fpfh_feature_radius_);
    RCLCPP_INFO(this->get_logger(), "Prior-map FPFH features ready.");
  }

  void init_gicp()
  {
    gicp_reg_.setRegistrationType(registration_type_);
    gicp_reg_.setNumThreads(num_threads_);
    gicp_reg_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    gicp_reg_.setCorrespondenceRandomness(20);
    gicp_reg_.setInputTarget(gicp_map_pcl_);
    current_guess_ = Eigen::Matrix4f::Identity();
  }

  void source_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!map_ready_ || state_ == RelocState::LOCKED)
    {
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != source_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Source cloud frame is %s, expected %s. Using points as %s anyway.",
        msg->header.frame_id.c_str(),
        source_frame_id_.c_str(),
        source_frame_id_.c_str());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!has_accumulation_start_)
    {
      accumulation_start_time_ = this->now();
      has_accumulation_start_ = true;
    }

    *accumulated_cloud_ += *cloud;
    if (static_cast<int>(accumulated_cloud_->size()) > max_accumulated_points_)
    {
      accumulated_cloud_ = small_gicp::voxelgrid_sampling_omp(*accumulated_cloud_, gicp_cloud_voxel_, num_threads_);
    }
  }

  void registration_timer_callback()
  {
    if (!map_ready_)
    {
      return;
    }

    if (!has_accumulation_start_)
    {
      publish_status("waiting_for_cloud", 0.0);
      return;
    }

    if (state_ == RelocState::LOCKED)
    {
      publish_status("locked", last_fitness_score_);
      return;
    }

    const double elapsed = (this->now() - accumulation_start_time_).seconds();
    if (state_ == RelocState::ACCUMULATING && elapsed < accumulate_duration_sec_)
    {
      publish_status("accumulating", last_fitness_score_);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Accumulating FAST-LIVO2 local map: %.1f / %.1f sec",
        elapsed,
        accumulate_duration_sec_);
      return;
    }

    if (state_ == RelocState::ACCUMULATING)
    {
      state_ = RelocState::TEASER_INIT;
      RCLCPP_INFO(this->get_logger(), "Accumulation complete. Starting TEASER initialization.");
    }

    if (state_ == RelocState::TEASER_INIT)
    {
      run_teaser_initialization();
      return;
    }

    if (state_ == RelocState::GICP_REFINE)
    {
      run_gicp_refinement();
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot_cloud(const double leaf_size)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot(new pcl::PointCloud<pcl::PointXYZ>);
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      *snapshot = *accumulated_cloud_;
    }

    if (snapshot->empty())
    {
      return snapshot;
    }

    return small_gicp::voxelgrid_sampling_omp(*snapshot, leaf_size, num_threads_);
  }

  void run_teaser_initialization()
  {
    auto source = snapshot_cloud(teaser_cloud_voxel_);
    if (source->size() < 50)
    {
      RCLCPP_WARN(this->get_logger(), "[TEASER] Accumulated local map is too small: %zu", source->size());
      publish_status("teaser_source_too_small", last_fitness_score_);
      return;
    }

    teaser::PointCloud source_teaser;
    convert_pcl_to_teaser(source, source_teaser);

    teaser::FPFHEstimation fpfh;
    auto source_descriptors = fpfh.computeFPFHFeatures(source_teaser, fpfh_normal_radius_, fpfh_feature_radius_);

    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
      source_teaser,
      teaser_map_cloud_,
      *source_descriptors,
      *teaser_map_descriptors_,
      false,
      true,
      false,
      0.95);

    if (correspondences.size() < 10)
    {
      teaser_success_count_ = 0;
      RCLCPP_WARN(this->get_logger(), "[TEASER] Too few correspondences: %zu", correspondences.size());
      publish_debug_cloud(source, Eigen::Matrix4f::Identity(), source_frame_id_);
      publish_status("teaser_too_few_correspondences", last_fitness_score_);
      return;
    }

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = noise_bound_;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = teaser_solver_max_iter_;
    params.rotation_gnc_factor = rotation_gnc_factor_;
    params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    teaser::RobustRegistrationSolver solver(params);
    solver.solve(source_teaser, teaser_map_cloud_, correspondences);
    const auto solution = solver.getSolution();

    if (!solution.valid)
    {
      teaser_success_count_ = 0;
      RCLCPP_WARN(this->get_logger(), "[TEASER] Solution invalid.");
      publish_debug_cloud(source, Eigen::Matrix4f::Identity(), source_frame_id_);
      publish_status("teaser_invalid", last_fitness_score_);
      return;
    }

    const auto inliers = solver.getTranslationInliers();
    RCLCPP_INFO(
      this->get_logger(),
      "[TEASER] Inliers: %zu/%zu (threshold: %zu)",
      inliers.size(),
      correspondences.size(),
      teaser_inlier_threshold_);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    transform.block<3, 1>(0, 3) = solution.translation.cast<float>();
    publish_debug_cloud(source, transform, map_frame_id_);

    if (inliers.size() < teaser_inlier_threshold_)
    {
      teaser_success_count_ = 0;
      publish_status("teaser_inliers_below_threshold", last_fitness_score_);
      return;
    }

    teaser_success_count_++;
    current_guess_ = transform;
    publish_status("teaser_success", last_fitness_score_);

    if (teaser_success_count_ >= teaser_success_count_thre_)
    {
      state_ = RelocState::GICP_REFINE;
      gicp_fail_count_ = 0;
      stable_count_ = 0;
      RCLCPP_INFO(
        this->get_logger(),
        "TEASER initialized map->%s: x=%.3f y=%.3f z=%.3f. Switching to GICP.",
        source_frame_id_.c_str(),
        current_guess_(0, 3),
        current_guess_(1, 3),
        current_guess_(2, 3));
    }
  }

  void run_gicp_refinement()
  {
    auto source = snapshot_cloud(gicp_cloud_voxel_);
    if (source->size() < 50)
    {
      RCLCPP_WARN(this->get_logger(), "[GICP] Accumulated local map is too small: %zu", source->size());
      publish_status("gicp_source_too_small", last_fitness_score_);
      return;
    }

    gicp_reg_.setInputSource(source);
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    gicp_reg_.align(aligned_cloud, current_guess_);

    const bool converged = gicp_reg_.hasConverged();
    const double fitness_score = gicp_reg_.getFitnessScore();
    const Eigen::Matrix4f final_transform = gicp_reg_.getFinalTransformation();
    last_fitness_score_ = fitness_score;

    RCLCPP_INFO(
      this->get_logger(),
      "[GICP] fitness=%.4f converged=%d stable=%d/%d",
      fitness_score,
      converged,
      stable_count_,
      stable_count_thre_);

    if (!converged)
    {
      handle_gicp_failure("gicp_not_converged");
      publish_debug_cloud(source, current_guess_, map_frame_id_);
      return;
    }

    current_guess_ = final_transform;

    if (fitness_score >= publish_fitness_score_thre_)
    {
      gicp_fail_count_++;
      stable_count_ = 0;
      publish_debug_cloud(source, final_transform, map_frame_id_);
      publish_status("gicp_high_fitness", fitness_score);
      if (gicp_fail_count_ > 10)
      {
        RCLCPP_WARN(this->get_logger(), "[GICP] Fitness stayed high. Returning to TEASER initialization.");
        state_ = RelocState::TEASER_INIT;
        teaser_success_count_ = 0;
        gicp_fail_count_ = 0;
      }
      return;
    }

    gicp_fail_count_ = 0;
    publish_pose(final_transform, fitness_score);
    publish_debug_cloud(aligned_cloud, map_frame_id_);

    update_stability(final_transform, fitness_score);
    publish_status("gicp_refine", fitness_score);
  }

  void handle_gicp_failure(const std::string &reason)
  {
    gicp_fail_count_++;
    stable_count_ = 0;
    publish_status(reason, last_fitness_score_);
    if (gicp_fail_count_ > 5)
    {
      RCLCPP_WARN(this->get_logger(), "[GICP] Failed repeatedly. Returning to TEASER initialization.");
      state_ = RelocState::TEASER_INIT;
      teaser_success_count_ = 0;
      gicp_fail_count_ = 0;
    }
  }

  void update_stability(const Eigen::Matrix4f &transform, const double fitness_score)
  {
    if (fitness_score >= final_fitness_score_thre_)
    {
      stable_count_ = 0;
      last_stable_candidate_ = transform;
      have_last_stable_candidate_ = true;
      return;
    }

    if (!have_last_stable_candidate_)
    {
      stable_count_ = 1;
      last_stable_candidate_ = transform;
      have_last_stable_candidate_ = true;
      return;
    }

    const double trans_delta = translation_delta(last_stable_candidate_, transform);
    const double rot_delta = rotation_delta_deg(last_stable_candidate_, transform);
    if (trans_delta <= stable_translation_thre_ && rot_delta <= stable_rotation_thre_deg_)
    {
      stable_count_++;
    }
    else
    {
      stable_count_ = 1;
    }

    last_stable_candidate_ = transform;

    if (stable_count_ >= stable_count_thre_)
    {
      publish_pose(transform, fitness_score);
      state_ = RelocState::LOCKED;
      RCLCPP_INFO(
        this->get_logger(),
        "Relocalization locked. fitness=%.4f stable_count=%d",
        fitness_score,
        stable_count_);
      publish_status("locked", fitness_score);
    }
  }

  void convert_pcl_to_teaser(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, teaser::PointCloud &teaser_cloud)
  {
    teaser_cloud.clear();
    teaser_cloud.reserve(cloud->size());
    for (const auto &point : cloud->points)
    {
      teaser_cloud.push_back({point.x, point.y, point.z});
    }
  }

  void publish_pose(const Eigen::Matrix4f &transform, const double fitness_score)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = map_frame_id_;

    pose_msg.pose.pose.position.x = transform(0, 3);
    pose_msg.pose.pose.position.y = transform(1, 3);
    pose_msg.pose.pose.position.z = transform(2, 3);

    Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    q.normalize();
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    for (double &cov : pose_msg.pose.covariance)
    {
      cov = 0.0;
    }
    const double cov = std::max(0.001, fitness_score);
    pose_msg.pose.covariance[0] = cov;
    pose_msg.pose.covariance[7] = cov;
    pose_msg.pose.covariance[14] = cov;
    pose_msg.pose.covariance[21] = cov;
    pose_msg.pose.covariance[28] = cov;
    pose_msg.pose.covariance[35] = cov;

    pose_pub_->publish(pose_msg);
  }

  void publish_debug_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Eigen::Matrix4f &transform,
    const std::string &frame_id)
  {
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);
    publish_debug_cloud(transformed_cloud, frame_id);
  }

  void publish_debug_cloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &frame_id)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = frame_id;
    transformed_cloud_pub_->publish(cloud_msg);
  }

  void publish_status(const std::string &detail, const double fitness_score)
  {
    std_msgs::msg::String status;
    std::ostringstream oss;
    size_t cloud_size = 0;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_size = accumulated_cloud_->size();
    }

    oss << "state=" << state_name(state_)
        << " detail=" << detail
        << " fitness=" << fitness_score
        << " stable=" << stable_count_ << "/" << stable_count_thre_
        << " points=" << cloud_size;
    status.data = oss.str();
    status_pub_->publish(status);
  }

  bool map_ready_ = false;
  RelocState state_ = RelocState::ACCUMULATING;

  std::string map_path_;
  std::string source_cloud_topic_;
  std::string source_frame_id_;
  std::string map_frame_id_;
  std::string registration_type_;

  double accumulate_duration_sec_ = 10.0;
  double registration_period_sec_ = 1.0;
  int max_accumulated_points_ = 300000;

  double teaser_map_voxel_ = 0.4;
  double teaser_cloud_voxel_ = 0.4;
  double gicp_map_voxel_ = 0.2;
  double gicp_cloud_voxel_ = 0.1;

  double fpfh_normal_radius_ = 0.8;
  double fpfh_feature_radius_ = 1.2;
  double noise_bound_ = 0.3;
  double rotation_gnc_factor_ = 1.4;
  int teaser_solver_max_iter_ = 100;
  size_t teaser_inlier_threshold_ = 5;
  int teaser_success_count_thre_ = 3;
  int teaser_success_count_ = 0;

  int gicp_solver_max_iter_ = 50;
  int num_threads_ = 4;
  double max_correspondence_distance_ = 5.0;
  double publish_fitness_score_thre_ = 0.5;
  double final_fitness_score_thre_ = 0.2;
  double stable_translation_thre_ = 0.05;
  double stable_rotation_thre_deg_ = 1.0;
  int stable_count_thre_ = 3;
  int stable_count_ = 0;
  int gicp_fail_count_ = 0;
  double last_fitness_score_ = 0.0;

  bool has_accumulation_start_ = false;
  rclcpp::Time accumulation_start_time_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr teaser_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
  teaser::PointCloud teaser_map_cloud_;
  std::shared_ptr<teaser::FPFHCloud> teaser_map_descriptors_;

  small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp_reg_;
  Eigen::Matrix4f current_guess_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f last_stable_candidate_ = Eigen::Matrix4f::Identity();
  bool have_last_stable_candidate_ = false;

  std::mutex cloud_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapTeaserGicpNode>());
  rclcpp::shutdown();
  return 0;
}

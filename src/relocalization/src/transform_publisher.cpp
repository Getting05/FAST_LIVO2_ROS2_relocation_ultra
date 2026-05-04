#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
  TransformPublisherNode()
  : Node("transform_publisher_node")
  {
    this->declare_parameter<std::string>("odom_frame_id", "camera_init");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<bool>("static_transform", false);
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("static_transform", static_transform_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));

    if (static_transform_)
    {
      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      RCLCPP_INFO(this->get_logger(), "Publishing map->odom as a static transform for compatibility.");
    }
    else
    {
      dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      const auto period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
      republish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(std::max(1, period_ms)),
        std::bind(&TransformPublisherNode::republish_latest_transform, this));
      RCLCPP_INFO(this->get_logger(), "Publishing map->odom as a dynamic transform.");
    }
  }

private:
  void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = map_frame_id_;
    transform.child_frame_id = odom_frame_id_;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      latest_transform_ = transform;
      has_transform_ = true;
    }

    send_transform(transform);
  }

  void republish_latest_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      if (!has_transform_)
      {
        return;
      }
      transform = latest_transform_;
    }

    transform.header.stamp = this->now();
    send_transform(transform);
  }

  void send_transform(const geometry_msgs::msg::TransformStamped &transform)
  {
    if (static_transform_)
    {
      static_broadcaster_->sendTransform(transform);
      return;
    }

    dynamic_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  rclcpp::TimerBase::SharedPtr republish_timer_;

  geometry_msgs::msg::TransformStamped latest_transform_;
  bool has_transform_ = false;
  std::mutex transform_mutex_;

  std::string odom_frame_id_;
  std::string map_frame_id_;
  bool static_transform_ = false;
  double publish_rate_hz_ = 20.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

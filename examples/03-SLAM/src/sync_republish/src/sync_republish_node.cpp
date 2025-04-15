#include <memory>
#include <string>
#include <functional>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TF2 for broadcasting transforms
#include "tf2_ros/transform_broadcaster.h"

// Message filters for time synchronization
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

class SyncRepublishNode : public rclcpp::Node
{
public:
  SyncRepublishNode() : Node("sync_republish_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting SyncRepublishNode");

    // Create publishers for republishing the synchronized messages
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_synced", 10);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_synced", 10);

    // Create the TF broadcaster for publishing the transform
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize message_filters subscribers for "odom" and "scan"
    odom_sub_.subscribe(this, "odom");
    scan_sub_.subscribe(this, "scan");

    // Create an approximate time synchronizer with a queue size of 10 and a slop of 0.1 seconds
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), odom_sub_, scan_sub_);
    sync_->registerCallback(std::bind(&SyncRepublishNode::callback, this, _1, _2));
  }

private:
  // Define the synchronizer policy: approximate time synchronization for Odometry and LaserScan messages.
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::LaserScan
  > SyncPolicy;

  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    // Use current time as the common timestamp
    rclcpp::Time common_time = this->now();

    // Create copies of the messages so we can modify their header stamps
    auto odom_synced = std::make_shared<nav_msgs::msg::Odometry>(*odom_msg);
    auto scan_synced = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);

    odom_synced->header.stamp = common_time;
    scan_synced->header.stamp = common_time;
    scan_synced->header.frame_id = "lidar_link";

    // Republish the synchronized messages
    odom_pub_->publish(*odom_synced);
    scan_pub_->publish(*scan_synced);

    // Build and broadcast a transform using the pose from the odom message.
    // The transform will have the parent frame from the odom message header (e.g., "odom")
    // and "scan" as the child frame.
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = common_time;
    t.header.frame_id = odom_synced->header.frame_id; // e.g., "odom"
    t.child_frame_id = "base_link";

    t.transform.translation.x = odom_synced->pose.pose.position.x;
    t.transform.translation.y = odom_synced->pose.pose.position.y;
    t.transform.translation.z = odom_synced->pose.pose.position.z;
    t.transform.rotation = odom_synced->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
   
  }

  // Declare message_filters subscribers without initializer list
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publishers for republishing messages
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SyncRepublishNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

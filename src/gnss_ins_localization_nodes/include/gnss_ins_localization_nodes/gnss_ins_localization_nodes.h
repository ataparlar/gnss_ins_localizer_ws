// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_
#define GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();

  GeographicLib::LocalCartesian local_cartesian_map;
  bool is_init = true;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_to_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr map_to_pose_twist;

  rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr msg_49_sub_;
  rclcpp::Subscription<applanix_msgs::msg::NavigationPerformanceGsof50>::SharedPtr msg_50_sub_;

  std::shared_ptr<tf2_ros::Buffer> buffer_ptr_transform;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_ptr_transform;

  geometry_msgs::msg::TransformStamped transformStampedInsCorrected;
  double deg2rad = M_PI / 180;
 private:

  void msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg_49);
  void msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg_50);

  geometry_msgs::msg::TwistWithCovarianceStamped gnss_baselink_twist;
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_baselink_pose;

};

#endif  // GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_

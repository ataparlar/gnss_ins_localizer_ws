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

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();

  GeographicLib::LocalCartesian local_cartesian_map;
  bool is_init = true;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_to_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr map_to_pose_twist;

  std::shared_ptr<tf2_ros::Buffer> buffer_ptr_transform;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_ptr_transform;

  geometry_msgs::msg::TransformStamped transformStampedInsCorrected;

 private:
  using SyncPolicyT = message_filters::sync_policies::ExactTime<
      applanix_msgs::msg::NavigationSolutionGsof49, applanix_msgs::msg::NavigationPerformanceGsof50>;
  std::unique_ptr<message_filters::Subscriber<
      applanix_msgs::msg::NavigationSolutionGsof49 >> sub_49;

  std::unique_ptr<message_filters::Subscriber<
      applanix_msgs::msg::NavigationPerformanceGsof50 >> sub_50;

  std::unique_ptr<message_filters::Synchronizer<SyncPolicyT >> my_synchronizer;

  void GnssCallback(
      const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &Msg_49,
      const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &Msg_50);
};

#endif  // GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_

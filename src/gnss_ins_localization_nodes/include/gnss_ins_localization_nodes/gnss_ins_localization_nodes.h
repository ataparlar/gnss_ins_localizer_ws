// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_
#define GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();

//  GeographicLib::LocalCartesian local_cartesian_map;
  bool is_init = true;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_to_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vehicle_twist_publisher;

  rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr msg_49_sub_;
  rclcpp::Subscription<applanix_msgs::msg::NavigationPerformanceGsof50>::SharedPtr msg_50_sub_;

//  geometry_msgs::msg::TransformStamped transformStampedInsCorrected;
  double deg2rad = M_PI / 180;
 private:

  void msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg_49);
  void msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg_50);

  geometry_msgs::msg::TwistWithCovarianceStamped gnss_baselink_twist;
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_baselink_pose;

  geometry_msgs::msg::TwistWithCovarianceStamped vehicle_twist_;

  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr pub_point_cloud_right_scan;
  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr sub_point_cloud_right_scan;

  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr sub_velodyne_scan_test;

  velodyne_msgs::msg::VelodyneScan pc_will_publish_;

  void cb_right_scan(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr msg);

  // imu
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  sensor_msgs::msg::Imu imu_msg;

  // velocity report
  void velocity_report_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_report;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_report;


};

#endif  // GNSS_INS_LOCALIZATION_NODE__GNSS_INS_LOCALIZATION_NODE_H_
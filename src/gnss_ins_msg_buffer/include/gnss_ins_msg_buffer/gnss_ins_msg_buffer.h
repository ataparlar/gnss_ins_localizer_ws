// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_
#define GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vehicle_twist_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr pvt_msg_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_msg_publisher;


  double deg2rad = M_PI / 180;
  double rad2deg = 180 / M_PI;
  sensor_msgs::msg::NavSatFix  nav_sat_fix_msg;

  sensor_msgs::msg::Imu imu_msg;

  rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr msg_49_sub_;
  rclcpp::Subscription<applanix_msgs::msg::NavigationPerformanceGsof50>::SharedPtr msg_50_sub_;



    //added for point cloud bag file test
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr msg_point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr msg_point_cloud_pub_;
  void msg_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg);
  void msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg);

//  geometry_msgs::msg::TwistWithCovarianceStamped vehicle_twist;
  geometry_msgs::msg::TwistWithCovarianceStamped vehicle_twist_;
};

#endif  // GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_

// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_
#define GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <memory>

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();

  rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr pvt_msg_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_msg_publisher;

  double deg2rad = M_PI / 180;
  double rad2deg = 180 / M_PI;
  sensor_msgs::msg::NavSatFix  nav_sat_fix_msg;

  rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr msg_49_sub_;
  rclcpp::Subscription<applanix_msgs::msg::NavigationPerformanceGsof50>::SharedPtr msg_50_sub_;

  void msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg);
  void msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg);
};

#endif  // GNSS_INS_MSG_BUFFER_NODE__GNSS_INS_MSG_BUFFER_NODE_H_

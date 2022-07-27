// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "gnss_ins_localization_nodes/gnss_ins_localization_nodes.h"

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <GeographicLib/UTMUPS.hpp>
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

using namespace GeographicLib;
CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/twist_estimator/twist_with_covariance/test",
      10);
  map_to_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss_pose", 10); // /sensing/gnss/pose_with_covariance

  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
      "/lvx_client/gsof/ins_solution_49", 10,
      std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
      "/lvx_client/gsof/ins_solution_rms_50", 10,
      std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));


  std::function<void(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr msg)> cb_right_scan = std::bind(
            &CartesianConv::cb_right_scan, this,
            std::placeholders::_1);

  // scan pub-sub
  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  sub_velodyne_scan_test = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
            "/right", rclcpp::SensorDataQoS(),
            std::bind(&CartesianConv::cb_right_scan, this, std::placeholders::_1), main_sub_opt);

  pub_point_cloud_right_scan = this->create_publisher<velodyne_msgs::msg::VelodyneScan>("/sensing/lidar/right/velodyne_packets", 10);

  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
            "/sensing/imu/imu_data",
            10);

  sub_velocity_report = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "velocity_status", 10,
            std::bind(&CartesianConv::velocity_report_callback, this, std::placeholders::_1));

  pub_velocity_report = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", rclcpp::QoS{1});

}

void CartesianConv::velocity_report_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {

    autoware_auto_vehicle_msgs::msg::VelocityReport vel_report_will_publish;
    vel_report_will_publish.lateral_velocity = msg->lateral_velocity;
    vel_report_will_publish.longitudinal_velocity = msg->longitudinal_velocity;
    vel_report_will_publish.heading_rate = msg->heading_rate;
    vel_report_will_publish.header = msg->header;
    vel_report_will_publish.header.stamp = this->now();

    pub_velocity_report->publish(vel_report_will_publish);

    std::cout<<" publish velocity status"<<std::endl;

}
void CartesianConv::cb_right_scan(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr msg){

    pc_will_publish_.header.frame_id = "velodyne_right";
    pc_will_publish_.packets = msg->packets;
    pc_will_publish_.header.stamp = this->now();
    for (int i = 0; i <= pc_will_publish_.packets.size(); i++){
        pc_will_publish_.packets[i].stamp = this->now();
    }
    pub_point_cloud_right_scan->publish(pc_will_publish_);
    std::cout<<" publish scan"<<std::endl;

}

void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg_49) {
  double Latitude = msg_49->lla.latitude;
  double Longitude = msg_49->lla.longitude;
  double Altitude = msg_49->lla.altitude;

  // pose with covariance stamped msg
  gnss_baselink_pose.header.frame_id = "base_link";
  gnss_baselink_pose.header.stamp = this->now();

  // map to base_link rotations
  double deg2rad = M_PI / 180;

  tf2::Quaternion quaternion, ins_corrected_quat;
  double roll  = msg_49->roll * deg2rad;
  double pitch = msg_49->pitch * deg2rad;
  double yaw   = msg_49->heading * deg2rad;
  quaternion.setRPY(roll, pitch, yaw);

  tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_, ins_corrected_rot_, ins_to_ins_corrected;
  ins_rot_matrix.setRotation(quaternion);

  ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
  ins_to_ins_corrected = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

  ins_rot_ = ENU2NED * ins_rot_matrix;

  ins_corrected_rot_ = ins_rot_ * ins_to_ins_corrected;
  ins_corrected_rot_.getRotation(ins_corrected_quat);

  geometry_msgs::msg::PoseStamped ins_corrected_rot_pose;
  ins_corrected_rot_pose.pose.orientation.x = ins_corrected_quat.getX();
  ins_corrected_rot_pose.pose.orientation.y = ins_corrected_quat.getY();
  ins_corrected_rot_pose.pose.orientation.z = ins_corrected_quat.getZ();
  ins_corrected_rot_pose.pose.orientation.w = ins_corrected_quat.getW();

  //changed as ins
  gnss_baselink_pose.pose.pose.orientation = ins_corrected_rot_pose.pose.orientation;

  // map to base_link position
  geometry_msgs::msg::PoseStamped ins_corrected_pose_map;

  double lat = Latitude, lon = Longitude; // Baghdad
  int zone ;
  bool northp;

  UTMUPS::Forward(lat, lon, zone, northp, ins_corrected_pose_map.pose.position.x, ins_corrected_pose_map.pose.position.y);

  ins_corrected_pose_map.pose.position.x = ins_corrected_pose_map.pose.position.x - 698891.000 ;
  ins_corrected_pose_map.pose.position.y = ins_corrected_pose_map.pose.position.y - 4520550.000;
  ins_corrected_pose_map.pose.position.z = Altitude - 47.47;

  gnss_baselink_pose.pose.pose.position = ins_corrected_pose_map.pose.position;

  map_to_pose_->publish(gnss_baselink_pose);

  vehicle_twist_.header.frame_id = "base_link";
  vehicle_twist_.header.stamp = this->now();
  vehicle_twist_.twist.twist.linear.x = msg_49->total_speed;
  vehicle_twist_.twist.twist.angular.z = - msg_49->ang_rate_down * deg2rad;
  vehicle_twist_publisher->publish(vehicle_twist_);

  ///// imu

  imu_msg.header.frame_id = "base_link";
  imu_msg.header.stamp = rclcpp::Clock().now();
  imu_msg.orientation.x = ins_corrected_quat.getX();
  imu_msg.orientation.y = ins_corrected_quat.getY();
  imu_msg.orientation.z = ins_corrected_quat.getZ();
  imu_msg.orientation.w = ins_corrected_quat.getW();

  imu_msg.angular_velocity.x = msg_49->ang_rate_long * deg2rad;
  imu_msg.angular_velocity.y = - msg_49->ang_rate_trans * deg2rad;
  imu_msg.angular_velocity.z = - msg_49->ang_rate_down * deg2rad;

  imu_msg.linear_acceleration.x = msg_49->acc_long;
  imu_msg.linear_acceleration.y = - msg_49->acc_trans;
  imu_msg.linear_acceleration.z = - msg_49->acc_down;

  imu_publisher->publish(imu_msg);


}

void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg_50) {

  gnss_baselink_pose.pose.covariance[0]  = std::pow(msg_50->pos_rms_error.east, 2);
  gnss_baselink_pose.pose.covariance[7]  = std::pow(msg_50->pos_rms_error.north, 2);
  gnss_baselink_pose.pose.covariance[14] = std::pow(msg_50->pos_rms_error.down, 2);
  gnss_baselink_pose.pose.covariance[21] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[28] = std::pow(msg_50->attitude_rms_error_pitch * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[35] = std::pow( msg_50->attitude_rms_error_heading * deg2rad, 2);

  imu_msg.orientation_covariance[0] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);
  imu_msg.orientation_covariance[4] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);
  imu_msg.orientation_covariance[8] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);

  imu_msg.angular_velocity_covariance[0] = 0.1;
  imu_msg.angular_velocity_covariance[4] = 0.1;
  imu_msg.angular_velocity_covariance[8] = 10;

  imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}


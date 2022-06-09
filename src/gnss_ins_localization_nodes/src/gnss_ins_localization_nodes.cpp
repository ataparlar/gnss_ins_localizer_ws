// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "gnss_ins_localization_nodes/gnss_ins_localization_nodes.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <memory>
#include <string>
#include <iostream>
// include geographic lib WGS84_GRS80
#include <GeographicLib/Geocentric.hpp>
#include "velodyne_msgs/msg/velodyne_scan.hpp"
using namespace GeographicLib;
CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/twist_estimator/twist_with_covariance",
      10);
  map_to_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 10);

  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
      "/lvx_client/gsof/ins_solution_49", 10,
      std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
      "/lvx_client/gsof/ins_solution_rms_50", 10,
      std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));

  msg_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/test/point_cloud", 10);
  msg_point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_right/velodyne_points", 10,
        std::bind(&CartesianConv::msg_point_cloud_callback, this, std::placeholders::_1));

  buffer_ptr_transform = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ptr_transform = std::make_shared<tf2_ros::TransformListener>(*buffer_ptr_transform);


    std::function<void(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr msg)> cb_right_scan = std::bind(
            &CartesianConv::cb_right_scan, this,
            std::placeholders::_1);
    sub_point_cloud_right_scan = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
            "/lidar_right/velodyne_packets", 10, cb_right_scan);
    pub_point_cloud_right_scan = this->create_publisher<velodyne_msgs::msg::VelodyneScan>("/sensing/lidar/right/velodyne_packets", 10);


}

void CartesianConv::msg_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 msg_out;
    msg_out = *msg;
    msg_out.header.stamp = this->get_clock()->now();
    msg_point_cloud_pub_->publish(msg_out);
}

void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg_49) {
  double Latitude = msg_49->lla.latitude;
  double Longitude = msg_49->lla.longitude;
  double Altitude = msg_49->lla.altitude;

  // pose with covariance stamped msg
  gnss_baselink_pose.header.frame_id = "map";
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
  ins_to_ins_corrected = tf2::Matrix3x3(0, 1, 0, 0, 0, -1, -1, 0, 0);

  ins_rot_ = ENU2NED * ins_rot_matrix;

  ins_corrected_rot_ = ins_rot_ * ins_to_ins_corrected;
//  ins_corrected_rot_ = ins_rot_ ;
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
  vehicle_twist_.header.stamp = rclcpp::Clock().now();
  vehicle_twist_.twist.twist.linear.x = msg_49->total_speed;
  vehicle_twist_.twist.twist.angular.z = - msg_49->ang_rate_down * deg2rad;
  vehicle_twist_publisher->publish(vehicle_twist_);



}

void CartesianConv::cb_right_scan(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr &msg){

    pc_will_publish_.packets = msg->packets;
    pc_will_publish_.header.frame_id = "velodyne_right";
    pc_will_publish_.header.stamp = gnss_baselink_pose.header.stamp;
    pc_will_publish_.packets.data()->stamp = this->now();
    pub_point_cloud_right_scan->publish(pc_will_publish_);
    std::cout<<" publishing scan"<<std::endl;

}
void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg_50) {

  gnss_baselink_pose.pose.covariance[0]  = std::pow(msg_50->pos_rms_error.east, 2);
  gnss_baselink_pose.pose.covariance[7]  = std::pow(msg_50->pos_rms_error.north, 2);
  gnss_baselink_pose.pose.covariance[14] = std::pow(msg_50->pos_rms_error.down, 2);
  gnss_baselink_pose.pose.covariance[21] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[28] = std::pow(msg_50->attitude_rms_error_pitch * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[35] = std::pow( msg_50->attitude_rms_error_heading * deg2rad, 2);

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}


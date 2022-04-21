// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "gnss_ins_msg_buffer/gnss_ins_msg_buffer.h"

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

  pvt_msg_publisher = this->create_publisher<ublox_msgs::msg::NavPVT>(
      "/navpvt",
      10);
  navsat_fix_msg_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/fix",
      10);
  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
          "/lvx_client/gsof/ins_solution_49", 10,
          std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
            "/lvx_client/gsof/ins_solution_rms_50", 10,
            std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));

}

void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr Msg_49) {

    tf2::Quaternion quaternion, ins_corrected_quat;
    double roll = Msg_49->roll * deg2rad;
    double pitch = Msg_49->pitch * deg2rad;
    double yaw = Msg_49->heading * deg2rad;
    quaternion.setRPY(roll, pitch, yaw);

    tf2::Matrix3x3 ins_rot_matrix, ins_corrected_rot_, ins_to_ins_corrected;
    ins_rot_matrix.setRotation(quaternion);
    ins_to_ins_corrected = tf2::Matrix3x3(0, 1, 0, 0, 0, -1, -1, 0, 0);

    ins_corrected_rot_ = ins_rot_matrix * ins_to_ins_corrected;
    ins_corrected_rot_.getRotation(ins_corrected_quat);

    geometry_msgs::msg::PoseStamped ins_corrected_rot_pose;
    ins_corrected_rot_pose.pose.orientation.x = ins_corrected_quat.getX();
    ins_corrected_rot_pose.pose.orientation.y = ins_corrected_quat.getY();
    ins_corrected_rot_pose.pose.orientation.z = ins_corrected_quat.getZ();
    ins_corrected_rot_pose.pose.orientation.w = ins_corrected_quat.getW();


    nav_sat_fix_msg.header.frame_id = "gnss";
    nav_sat_fix_msg.header.stamp = rclcpp::Time();

    nav_sat_fix_msg.latitude = Msg_49->lla.latitude;
    nav_sat_fix_msg.longitude = Msg_49->lla.longitude;
    nav_sat_fix_msg.altitude = Msg_49->lla.altitude;


    tf2::Matrix3x3 m(ins_corrected_quat);
    double roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected;
    m.getRPY(roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected);

    ublox_msgs::msg::NavPVT nav_pvt_msg;
    nav_pvt_msg.heading = yaw_ins_corrected * rad2deg / 1e-5;

    navsat_fix_msg_publisher->publish(nav_sat_fix_msg);
    pvt_msg_publisher->publish(nav_pvt_msg);

}
void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr Msg_50) {

    nav_sat_fix_msg.position_covariance[0] = std::pow(Msg_50->pos_rms_error.east, 2);
    nav_sat_fix_msg.position_covariance[7] = std::pow(Msg_50->pos_rms_error.north, 2);
    nav_sat_fix_msg.position_covariance[14] = std::pow(Msg_50->pos_rms_error.down, 2);
    nav_sat_fix_msg.position_covariance[21] = std::pow(Msg_50->attitude_rms_error_roll * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[28] = std::pow(Msg_50->attitude_rms_error_pitch * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[35] = std::pow(Msg_50->attitude_rms_error_heading * deg2rad, 2);
    nav_sat_fix_msg.position_covariance_type = 1;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}

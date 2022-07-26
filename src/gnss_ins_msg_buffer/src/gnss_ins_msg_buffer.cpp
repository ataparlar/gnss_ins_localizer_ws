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

  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/vehicle/status/twist_with_covariance",
          10);

  vehicle_twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "gyro_twist_with_covariance",
          10);
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
          "/sensing/imu/imu_data",
          10);
  pvt_msg_publisher = this->create_publisher<ublox_msgs::msg::NavPVT>(
      "/navpvt",
      10);
  navsat_fix_msg_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/fix",
      10);
//  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
//          "/lvx_client/gsof/ins_solution_49", 10,
//          std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
//  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
//            "/lvx_client/gsof/ins_solution_rms_50", 10,
//            std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));
  msg_point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/lidar_right/velodyne_points", 10,
          std::bind(&CartesianConv::msg_point_cloud_callback, this, std::placeholders::_1));
  msg_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/test/lidar_right/velodyne_points_converted", 10);

  // for message_filters
  lla_subs.subscribe(
          this,
          "/lvx_client/gsof/ins_solution_49");
  rms_subs.subscribe(
          this,
          "/lvx_client/gsof/ins_solution_rms_50");

  // synchronizer defined here with reset() function.
  // initialized with a new pointer object inside
  // with approximate policy
  syncobj_.reset(
          new SyncObj(
                  approximate_policy(10),
                  CartesianConv::lla_subs,
                  CartesianConv::rms_subs));

  // callback is called here.
  syncobj_->registerCallback(
          std::bind(
                  &CartesianConv::applanix_msgs_callback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

  // publisher initialized
  autoware_msg_publisher = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
          "/autoware_orientation",
          10);
}

//void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr Msg_49) {
//
//    tf2::Quaternion quaternion, ins_corrected_quat;
//    double roll = Msg_49->roll * deg2rad;
//    double pitch = Msg_49->pitch * deg2rad;
//    double yaw = Msg_49->heading * deg2rad;
//    quaternion.setRPY(roll, pitch, yaw);
//
//    tf2::Matrix3x3 ins_rot_matrix, ins_corrected_rot_, ins_to_ins_corrected;
//    ins_rot_matrix.setRotation(quaternion);
//    ins_to_ins_corrected = tf2::Matrix3x3(0, 1, 0, 0, 0, -1, -1, 0, 0);
//
//    ins_corrected_rot_ = ins_rot_matrix * ins_to_ins_corrected;
//    ins_corrected_rot_.getRotation(ins_corrected_quat);
//
//
//    imu_msg.header.frame_id = "base_link";
//    imu_msg.header.stamp = rclcpp::Clock().now();
//    imu_msg.orientation.x = ins_corrected_quat.getX();
//    imu_msg.orientation.y = ins_corrected_quat.getY();
//    imu_msg.orientation.z = ins_corrected_quat.getZ();
//    imu_msg.orientation.w = ins_corrected_quat.getW();
//
//    imu_msg.angular_velocity.z = - Msg_49->ang_rate_down * deg2rad;
//    imu_msg.linear_acceleration.x = Msg_49->acc_trans;
//    imu_msg.linear_acceleration.y = Msg_49->acc_long;
//    imu_msg.linear_acceleration.z = - Msg_49->acc_down;
//
//    imu_publisher->publish(imu_msg);
//
//
//    nav_sat_fix_msg.header.frame_id = "gnss";
//    nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();
//
//    nav_sat_fix_msg.latitude = Msg_49->lla.latitude;
//    nav_sat_fix_msg.longitude = Msg_49->lla.longitude;
//    nav_sat_fix_msg.altitude = Msg_49->lla.altitude;
//
//
//    tf2::Matrix3x3 m(ins_corrected_quat);
//    double roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected;
//    m.getRPY(roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected);
//
//    ublox_msgs::msg::NavPVT nav_pvt_msg;
//    nav_pvt_msg.heading = yaw_ins_corrected * rad2deg / 1e-5;
//
//    navsat_fix_msg_publisher->publish(nav_sat_fix_msg);
//    pvt_msg_publisher->publish(nav_pvt_msg);
//
//
//    vehicle_twist_.header.frame_id = "base_link";
//    vehicle_twist_.header.stamp = rclcpp::Clock().now();
//    vehicle_twist_.twist.twist.linear.x = Msg_49->total_speed;
//    vehicle_twist_.twist.twist.angular.z = - Msg_49->ang_rate_down * deg2rad;
//    vehicle_twist_publisher->publish(vehicle_twist_);
//
//}
//void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr Msg_50) {
//
//    vehicle_twist_.twist.covariance[0]  = std::pow(Msg_50->vel_rms_error.east, 2);
//    vehicle_twist_.twist.covariance[7]  = std::pow(Msg_50->vel_rms_error.north, 2);
//    vehicle_twist_.twist.covariance[14] = std::pow(Msg_50->vel_rms_error.down, 2);
//
//    vehicle_twist_.twist.covariance[21] = 1000;
//    vehicle_twist_.twist.covariance[28] = 1000;
//    vehicle_twist_.twist.covariance[35] = 1000;
//
//    nav_sat_fix_msg.position_covariance[0] = std::pow(Msg_50->pos_rms_error.east, 2);
//    nav_sat_fix_msg.position_covariance[1] = std::pow(Msg_50->pos_rms_error.north, 2);
//    nav_sat_fix_msg.position_covariance[2] = std::pow(Msg_50->pos_rms_error.down, 2);
//    nav_sat_fix_msg.position_covariance[3] = std::pow(Msg_50->attitude_rms_error_roll * deg2rad, 2);
//    nav_sat_fix_msg.position_covariance[4] = std::pow(Msg_50->attitude_rms_error_pitch * deg2rad, 2);
//    nav_sat_fix_msg.position_covariance[5] = std::pow(Msg_50->attitude_rms_error_heading * deg2rad, 2);
//    nav_sat_fix_msg.position_covariance_type = 1;
//
//    imu_msg.angular_velocity_covariance[0]  = std::pow(Msg_50->vel_rms_error.east, 2);
//    imu_msg.angular_velocity_covariance[7]  = std::pow(Msg_50->vel_rms_error.north, 2);
//    imu_msg.angular_velocity_covariance[14] = std::pow(Msg_50->vel_rms_error.down, 2);
//
//    imu_msg.angular_velocity_covariance[21] = 1000;
//    imu_msg.angular_velocity_covariance[28] = 1000;
//    imu_msg.angular_velocity_covariance[35] = 1000;
//}
void CartesianConv::msg_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg(new sensor_msgs::msg::PointCloud2);
    *point_cloud_msg = *msg;
    point_cloud_msg->header.frame_id = "velodyne_right";
    point_cloud_msg->header.stamp = rclcpp::Clock().now();
    msg_point_cloud_pub_->publish(*point_cloud_msg);
}

void CartesianConv::applanix_msgs_callback(
        const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &lla,
        const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &rms) {

    autoware_sensing_msgs::msg::GnssInsOrientationStamped pose_msg;
    tf2::Quaternion quaternion, ins_corrected_quat;
    double roll = lla->roll * deg2rad;
    double pitch = lla->pitch * deg2rad;
    double yaw = lla->heading * deg2rad;
    quaternion.setRPY(roll, pitch, yaw);

    tf2::Matrix3x3 ins_rot_matrix, ins_corrected_rot_, ins_to_ins_corrected;
    ins_rot_matrix.setRotation(quaternion);
    ins_to_ins_corrected = tf2::Matrix3x3(0, 1, 0, 0, 0, -1, -1, 0, 0);

    ins_corrected_rot_ = ins_rot_matrix * ins_to_ins_corrected;
    ins_corrected_rot_.getRotation(ins_corrected_quat);


    autoware_orientation_msg.header.frame_id = "base_link";
    autoware_orientation_msg.header.stamp = rclcpp::Clock().now();
    autoware_orientation_msg.orientation.orientation.x= ins_corrected_quat.getX();
    autoware_orientation_msg.orientation.orientation.y = ins_corrected_quat.getY();
    autoware_orientation_msg.orientation.orientation.z = ins_corrected_quat.getZ();
    autoware_orientation_msg.orientation.orientation.w = ins_corrected_quat.getW();
    autoware_orientation_msg.orientation.rmse_rotation_x = pow(rms->attitude_rms_error_roll, 2);
    autoware_orientation_msg.orientation.rmse_rotation_y = pow(rms->attitude_rms_error_pitch, 2);
    autoware_orientation_msg.orientation.rmse_rotation_z = pow(rms->attitude_rms_error_heading, 2);
    autoware_msg_publisher->publish(autoware_orientation_msg);

    imu_msg.angular_velocity.z = - lla->ang_rate_down * deg2rad;
    imu_msg.linear_acceleration.x = lla->acc_trans;
    imu_msg.linear_acceleration.y = lla->acc_long;
    imu_msg.linear_acceleration.z = - lla->acc_down;

    imu_publisher->publish(imu_msg);


    nav_sat_fix_msg.header.frame_id = "gnss";
    nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();

    nav_sat_fix_msg.latitude = lla->lla.latitude;
    nav_sat_fix_msg.longitude = lla->lla.longitude;
    nav_sat_fix_msg.altitude = lla->lla.altitude;


    tf2::Matrix3x3 m(ins_corrected_quat);
    double roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected;
    m.getRPY(roll_ins_corrected, pitch_ins_corrected, yaw_ins_corrected);

    ublox_msgs::msg::NavPVT nav_pvt_msg;
    nav_pvt_msg.heading = yaw_ins_corrected * rad2deg / 1e-5;

    vehicle_twist_.header.frame_id = "base_link";
    vehicle_twist_.header.stamp = rclcpp::Clock().now();
    vehicle_twist_.twist.twist.linear.x = lla->total_speed;
    vehicle_twist_.twist.twist.angular.z = - lla->ang_rate_down * deg2rad;
    vehicle_twist_publisher->publish(vehicle_twist_);


    vehicle_twist_.twist.covariance[0]  = std::pow(rms->vel_rms_error.east, 2);
    vehicle_twist_.twist.covariance[7]  = std::pow(rms->vel_rms_error.north, 2);
    vehicle_twist_.twist.covariance[14] = std::pow(rms->vel_rms_error.down, 2);

    vehicle_twist_.twist.covariance[21] = 1000;
    vehicle_twist_.twist.covariance[28] = 1000;
    vehicle_twist_.twist.covariance[35] = 1000;

    nav_sat_fix_msg.position_covariance[0] = std::pow(rms->pos_rms_error.east, 2);
    nav_sat_fix_msg.position_covariance[1] = std::pow(rms->pos_rms_error.north, 2);
    nav_sat_fix_msg.position_covariance[2] = std::pow(rms->pos_rms_error.down, 2);
    nav_sat_fix_msg.position_covariance[3] = std::pow(rms->attitude_rms_error_roll * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[4] = std::pow(rms->attitude_rms_error_pitch * deg2rad, 2);
    nav_sat_fix_msg.position_covariance[5] = std::pow(rms->attitude_rms_error_heading * deg2rad, 2);
    nav_sat_fix_msg.position_covariance_type = 1;

    imu_msg.angular_velocity_covariance[0]  = std::pow(rms->vel_rms_error.east, 2);
    imu_msg.angular_velocity_covariance[7]  = std::pow(rms->vel_rms_error.north, 2);
    imu_msg.angular_velocity_covariance[14] = std::pow(rms->vel_rms_error.down, 2);

    imu_msg.angular_velocity_covariance[21] = 1000;
    imu_msg.angular_velocity_covariance[28] = 1000;
    imu_msg.angular_velocity_covariance[35] = 1000;

    navsat_fix_msg_publisher->publish(nav_sat_fix_msg);
    pvt_msg_publisher->publish(nav_pvt_msg);

}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}

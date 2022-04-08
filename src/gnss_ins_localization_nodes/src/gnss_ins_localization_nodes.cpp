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
#include <memory>



CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

  map_to_pose_twist = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/test/twist_with_covariance_stamped",
      10);
  map_to_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/test/pose_with_covariance_stamped", 10);

  msg_49_sub_ = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
      "/lvx_client/gsof/ins_solution_49", 10,
      std::bind(&CartesianConv::msg_49_callback, this, std::placeholders::_1));
  msg_50_sub_ = this->create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
      "/lvx_client/gsof/ins_solution_rms_50", 10,
      std::bind(&CartesianConv::msg_50_callback, this, std::placeholders::_1));


  buffer_ptr_transform = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ptr_transform = std::make_shared<tf2_ros::TransformListener>(*buffer_ptr_transform);

}
void CartesianConv::msg_49_callback(const applanix_msgs::msg::NavigationSolutionGsof49::SharedPtr msg_49) {
  double Latitude = msg_49->lla.latitude;
  double Longitude = msg_49->lla.longitude;
  double Altitude = msg_49->lla.altitude;

  if (is_init) {

    // lookup base_link - ins_corrected transform
    std::string targetFrame = "ins_corrected";
    std::string sourceFrame = "base_link";
    try {
      transformStampedInsCorrected = buffer_ptr_transform->lookupTransform(
          targetFrame, sourceFrame,
          tf2::TimePointZero, tf2::Duration(1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          targetFrame.c_str(), sourceFrame.c_str(), ex.what());
      return;
    }

    double initial_map_lat = this->declare_parameter("latitude_map").template get<double>();
    double initial_map_lon = this->declare_parameter("longitude_map").template get<double>();
    double initial_map_alt = this->declare_parameter("altitude_map").template get<double>();

    std::cout << "map parameters: " << "  latitude " << initial_map_lat << " longitude " << initial_map_lon
              << " altitude " << initial_map_alt << std::endl;


    //create local cartesian reference map position according to earth
    local_cartesian_map = GeographicLib::LocalCartesian(initial_map_lat, initial_map_lon, initial_map_alt);

    is_init = false;
    return;
  }

  // pose with covariance stamped msg
  gnss_baselink_pose.header.frame_id = "map";
  gnss_baselink_pose.header.stamp = rclcpp::Clock().now();

  // map to base_link rotations
  double deg2rad = M_PI / 180;

  tf2::Quaternion quaternion, ins_corrected_quat;
  double roll = msg_49->roll * deg2rad;
  double pitch = msg_49->pitch * deg2rad;
  double yaw = msg_49->heading * deg2rad;
  quaternion.setRPY(roll, pitch, yaw);

  tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_, ins_corrected_rot_, ins_to_ins_corrected;
  ins_rot_matrix.setRotation(quaternion);

  ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
  ins_to_ins_corrected = tf2::Matrix3x3(0, 1, 0, 0, 0, -1, -1, 0, 0);

  ins_rot_ = ENU2NED * ins_rot_matrix;

  ins_corrected_rot_ = ins_rot_ * ins_to_ins_corrected;
  ins_corrected_rot_.getRotation(ins_corrected_quat);

  geometry_msgs::msg::PoseStamped ins_corrected_rot_pose;
  ins_corrected_rot_pose.pose.orientation.x = ins_corrected_quat.getX();
  ins_corrected_rot_pose.pose.orientation.y = ins_corrected_quat.getY();
  ins_corrected_rot_pose.pose.orientation.z = ins_corrected_quat.getZ();
  ins_corrected_rot_pose.pose.orientation.w = ins_corrected_quat.getW();

  geometry_msgs::msg::PoseStamped baselink_rot_pose;
  tf2::doTransform(ins_corrected_rot_pose, baselink_rot_pose, transformStampedInsCorrected);

  gnss_baselink_pose.pose.pose.orientation = baselink_rot_pose.pose.orientation;

  // map to base_link position
  geometry_msgs::msg::PoseStamped ins_corrected_pose_map;
  local_cartesian_map.Forward(Latitude,
                              Longitude,
                              Altitude,
                              ins_corrected_pose_map.pose.position.x,
                              ins_corrected_pose_map.pose.position.y,
                              ins_corrected_pose_map.pose.position.z);

  // transform ins_corrected_pose_map to base_link_map_pose
  geometry_msgs::msg::PoseStamped base_link_map_pose;
  tf2::doTransform(ins_corrected_pose_map, base_link_map_pose, transformStampedInsCorrected);

  gnss_baselink_pose.pose.pose.position = base_link_map_pose.pose.position;

  // gnss_base_link_twist
  gnss_baselink_twist.header.frame_id = "base_link";
  gnss_baselink_twist.header.stamp = rclcpp::Clock().now();

  // map to base_link twist
  gnss_baselink_twist.twist.twist.linear.x = msg_49->velocity.east;
  gnss_baselink_twist.twist.twist.linear.y = msg_49->velocity.north;
  gnss_baselink_twist.twist.twist.linear.z = msg_49->velocity.down;

  gnss_baselink_twist.twist.twist.angular.x = msg_49->ang_rate_trans * deg2rad;
  gnss_baselink_twist.twist.twist.angular.y = msg_49->ang_rate_long * deg2rad;
  gnss_baselink_twist.twist.twist.angular.z = msg_49->ang_rate_down * deg2rad;

  map_to_pose_twist->publish(gnss_baselink_twist);
  map_to_pose_->publish(gnss_baselink_pose);

  // map-base_link transform
//  geometry_msgs::msg::TransformStamped transformStamped_map_to_base_link;
//  transformStamped_map_to_base_link.header.stamp = rclcpp::Clock().now();
//  transformStamped_map_to_base_link.header.frame_id = "map";
//  transformStamped_map_to_base_link.child_frame_id = "base_link";
//  transformStamped_map_to_base_link.transform.translation.x = gnss_baselink_pose.pose.pose.position.x;
//  transformStamped_map_to_base_link.transform.translation.y = gnss_baselink_pose.pose.pose.position.y;
//  transformStamped_map_to_base_link.transform.translation.z = gnss_baselink_pose.pose.pose.position.z;
//
//  transformStamped_map_to_base_link.transform.rotation.x = gnss_baselink_pose.pose.pose.orientation.x;
//  transformStamped_map_to_base_link.transform.rotation.y = gnss_baselink_pose.pose.pose.orientation.y;
//  transformStamped_map_to_base_link.transform.rotation.z = gnss_baselink_pose.pose.pose.orientation.z;
//  transformStamped_map_to_base_link.transform.rotation.w = gnss_baselink_pose.pose.pose.orientation.w;
//
//
//  // send transforms
//  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_map_to_base_link =
//      std::make_shared<tf2_ros::TransformBroadcaster>(this);
//  tf_broadcaster_map_to_base_link->sendTransform(transformStamped_map_to_base_link);


}
void CartesianConv::msg_50_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg_50) {

  gnss_baselink_twist.twist.covariance[0]  = std::pow(msg_50->vel_rms_error.east, 2);
  gnss_baselink_twist.twist.covariance[7]  = std::pow(msg_50->vel_rms_error.north, 2);
  gnss_baselink_twist.twist.covariance[14] = std::pow(msg_50->vel_rms_error.down, 2);
  gnss_baselink_twist.twist.covariance[21] = 1000;
  gnss_baselink_twist.twist.covariance[28] = 1000;
  gnss_baselink_twist.twist.covariance[35] = 1000;

  gnss_baselink_pose.pose.covariance[0]  = std::pow(msg_50->pos_rms_error.east, 2);
  gnss_baselink_pose.pose.covariance[7]  = std::pow(msg_50->pos_rms_error.north, 2);
  gnss_baselink_pose.pose.covariance[14] = std::pow(msg_50->pos_rms_error.down, 2);
  gnss_baselink_pose.pose.covariance[21] = std::pow(msg_50->attitude_rms_error_roll * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[28] = std::pow(msg_50->attitude_rms_error_pitch * deg2rad, 2);
  gnss_baselink_pose.pose.covariance[35] = std::pow( msg_50->attitude_rms_error_heading * deg2rad, 2);

  std::cout<<"std::pow(msg_50->vel_rms_error.north, 2):  "<<std::pow(msg_50->vel_rms_error.north, 2)<<std::endl;
  std::cout<<"std::pow(msg_50->vel_rms_error.down, 2): "<<std::pow(msg_50->vel_rms_error.down, 2)<<std::endl;
  std::cout<<"std::pow( msg_50->attitude_rms_error_heading * deg2rad, 2) : "<<std::pow( msg_50->attitude_rms_error_heading * deg2rad, 2) <<std::endl;

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}

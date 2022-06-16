// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.


#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

using namespace std::chrono_literals;


class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher() : Node("OdomPublisher") {


        std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)> gnss_pose_cb = std::bind(
                &OdomPublisher::gnss_pose_cb, this,
                std::placeholders::_1);

        gnss_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/sensing/gnss/pose_with_covariance", 10, gnss_pose_cb);

        gnss_pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/test/gnss_pose", 10);

        std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)> ndt_pose_cb = std::bind(
                &OdomPublisher::ndt_pose_cb, this,
                std::placeholders::_1);

        ndt_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/pose_estimator/pose_with_covariance", 10, ndt_pose_cb);

        ndt_pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/test/ndt_pose", 10);


        std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)> ekf_pose_cb = std::bind(
                &OdomPublisher::ekf_pose_cb, this,
                std::placeholders::_1);
        ekf_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias", 10, ekf_pose_cb);

        ekf_pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/test/ekf_pose", 10);


    }

private:

    void ekf_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
        geometry_msgs::msg::Pose EKF_Pose;
        RCLCPP_INFO(this->get_logger(), "EKF");
        pose_array_ekf.header.frame_id = "map";
        pose_array_ekf.header.stamp = this->now();
        EKF_Pose= msg->pose.pose;
        pose_array_ekf.poses.push_back(EKF_Pose);
        ekf_pose_pub->publish(pose_array_ekf);
        std::cout<<"i am here"<<std::endl;

    }

    void gnss_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
        geometry_msgs::msg::Pose GNSS_Pose;
        RCLCPP_INFO(this->get_logger(), "GNSS");
        pose_array_gnss.header.frame_id = "map";
        pose_array_gnss.header.stamp = this->now();
        GNSS_Pose= msg->pose.pose;
        pose_array_gnss.poses.push_back(GNSS_Pose);
        gnss_pose_pub->publish(pose_array_gnss);
        std::cout<<"i am here"<<std::endl;

    }

    void ndt_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
        geometry_msgs::msg::Pose NDT_Pose;
        RCLCPP_INFO(this->get_logger(), "NDT");
        pose_array_ndt.header.frame_id = "map";
        pose_array_ndt.header.stamp = this->now();
        NDT_Pose= msg->pose.pose;
        pose_array_ndt.poses.push_back(NDT_Pose);
        ndt_pose_pub->publish(pose_array_ndt);
        std::cout<<"i am here"<<std::endl;

    }

    geometry_msgs::msg::PoseArray pose_array_ekf;
    geometry_msgs::msg::PoseArray pose_array_gnss;
    geometry_msgs::msg::PoseArray pose_array_ndt;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gnss_pose_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ndt_pose_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ekf_pose_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_sub;


};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}

from launch import LaunchDescription
from launch_ros.actions import Node

import ament_index_python.packages
import launch
import launch_ros.actions

def generate_launch_description():

    ld = LaunchDescription()

    node_left = Node(package = "tf2_ros", 
    		namespace="lidar_left",
                       executable = "static_transform_publisher",
                       arguments = ["-0.209462", "1.27408", "0.42718", "0.032233", "-0.0909166", "-0.274596", "0.956709","velodyne_middle","velodyne_left"])
                       
                       
    node_right = Node(package = "tf2_ros", 
    		namespace="lidar_right",
                       executable = "static_transform_publisher",
                       arguments = ["-0.121292", "-1.20668", "0.435086",  "-0.0129125", "-0.101894", "0.275353", "0.955841","velodyne_middle", "velodyne_right"])
                       
#    node_middle = Node(package = "tf2_ros", 
#    		namespace="lidar_middle",
#                       executable = "static_transform_publisher",
#                       arguments = ["6.20", "-0.195", "0.3", "-0.050905", "0.20070000000000002", "0","base_link", "velodyne_middle"])
                       
#    node_ins_corrected = Node(package = "tf2_ros", 
#    		namespace="ins_corrected",
#                       executable = "static_transform_publisher",
#                       arguments = ["-0.1062", "0.0", "-0.1129", "0.0", "-0.0871557", "0.0", "0.996194","velodyne_middle", "ins_corrected"])

    # node_ins_corrected_dummy = Node(package = "tf2_ros",
    # 		namespace="gnss",
    #                    executable = "static_transform_publisher",
    #                    arguments = ["0.124", "0.000", "0.093", "-0.0", "0.0871557", "-0.0", "0.996194","gnss", "velodyne_middle"])

    node_ins_corrected_dummy = Node(package = "tf2_ros",
                                    namespace="gnss",
                                    executable = "static_transform_publisher",
                                    arguments = ["0.124", "0.000", "0.093", "0.0", "0.174533", "-0.05","gnss", "velodyne_middle"])
    node_base_link_dummy = Node(package = "tf2_ros",
    		namespace="base_link",
                       executable = "static_transform_publisher",
                       arguments = ["6.055", "-0.188", "-0.527", "0.000", "0.013", "-0.025", "1.000","base_link", "gnss"])
           
    node_ins = Node(package = "tf2_ros", 
    		namespace="ins_raw",
                       executable = "static_transform_publisher",
                       arguments = ["0.0", "0.0", "0.0", "1.5708", "0", "-1.5708","gnss", "ins_raw"])


    node_gnss_base_link = Node(package = "tf2_ros",
                    namespace="ins_raw",
                    executable = "static_transform_publisher",
                    arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0","gnss_base_link", "base_link"])


    ld.add_action(node_left)
    ld.add_action(node_right)
#    ld.add_action(node_middle)
#    ld.add_action(node_ins_corrected)
    ld.add_action(node_ins_corrected_dummy)
    ld.add_action(node_base_link_dummy)
    ld.add_action(node_ins)
    # ld.add_action(node_gnss_base_link)
    
    return ld
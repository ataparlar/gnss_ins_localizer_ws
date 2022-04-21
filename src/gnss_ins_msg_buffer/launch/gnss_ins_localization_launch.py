from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    isuzu_map_params = {
        "latitude_map": 40.816617984672746,
        "longitude_map": 29.360491808334285,
        "altitude_map": 53.251157145314075

    }
    return LaunchDescription([
        Node(
            package='gnss_ins_localization_nodes',
            executable='gnss_ins_localization_nodes',
            name='gnss_ins_localization_nodes',
            parameters=[isuzu_map_params],
            output='screen'
        )

    ])
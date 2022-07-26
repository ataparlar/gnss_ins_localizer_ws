from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    isuzu_map_params = {
        "latitude_map": 40.81187906, #41.02297441,
        "longitude_map": 29.35810110, #28.88561442,
        "altitude_map": 47.62 #99.15

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
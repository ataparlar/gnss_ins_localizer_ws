from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    isuzu_map_params = {
        "latitude_map": 41.02297441,
        "longitude_map": 28.88561442,
        "altitude_map": 99.15

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
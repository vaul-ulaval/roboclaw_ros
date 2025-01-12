from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the diagnostic configuration file
    config_file = os.path.join(
        get_package_share_directory('roboclaw_ros'),  # Package name
        'config',                                     # Subdirectory
        'roboclaw_diag.yaml'                          # Config file name
    )

    return LaunchDescription([
        # Node for the diagnostic aggregator
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            parameters=[config_file],
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument('baud', default_value='460800'),
        DeclareLaunchArgument('max_speed', default_value='2.0'),
        DeclareLaunchArgument('ticks_per_rotation', default_value='2000'),
        DeclareLaunchArgument('gear_ratio', default_value='32'),
        DeclareLaunchArgument('wheel_circ', default_value='0.964'),
        DeclareLaunchArgument('base_width', default_value='0.69'),
        DeclareLaunchArgument('run_diag', default_value='true'),
        DeclareLaunchArgument('pub_odom_front', default_value='false'),
        DeclareLaunchArgument('pub_odom_back', default_value='false'),
        DeclareLaunchArgument('stop_movement', default_value='true'),
        DeclareLaunchArgument('max_accel_front', default_value='1.5'),
        DeclareLaunchArgument('max_accel_back', default_value='1.5'),
    ]

    # Get the path to the parameter file
    param_file_path = os.path.join(
        get_package_share_directory('roboclaw_ros'),
        'config',
        'roboclaw_diag.yaml'
    )

    # Define nodes
    nodes = [
        # Node(
        #     package='roboclaw_ros',
        #     executable='roboclaw_motors',
        #     name='roboclaw_motors_front',
        #     output='screen',
        #     respawn=True,
        #     condition=IfCondition(LaunchConfiguration('run_diag')),
        #     parameters=[{
        #         'dev': '/dev/rcFront',
        #         'baud': LaunchConfiguration('baud'),
        #         'address': 128,
        #         'max_speed': LaunchConfiguration('max_speed'),
        #         'ticks_per_rotation': LaunchConfiguration('ticks_per_rotation'),
        #         'gear_ratio': LaunchConfiguration('gear_ratio'),
        #         'wheel_circ': LaunchConfiguration('wheel_circ'),
        #         'base_width': LaunchConfiguration('base_width'),
        #         'pub_odom': LaunchConfiguration('pub_odom_front'),
        #         'stop_movement': LaunchConfiguration('stop_movement'),
        #         'max_accel': LaunchConfiguration('max_accel_front'),
        #         'name': 'roboclaw_front',
        #     }]
        # ),
        # Node(
        #     package='roboclaw_ros',
        #     executable='roboclaw_motors',
        #     name='roboclaw_motors_back',
        #     output='screen',
        #     respawn=True,
        #     condition=IfCondition(LaunchConfiguration('run_diag')),
        #     parameters=[{
        #         'dev': '/dev/rcBack',
        #         'baud': LaunchConfiguration('baud'),
        #         'address': 129,
        #         'max_speed': LaunchConfiguration('max_speed'),
        #         'ticks_per_rotation': LaunchConfiguration('ticks_per_rotation'),
        #         'gear_ratio': LaunchConfiguration('gear_ratio'),
        #         'wheel_circ': LaunchConfiguration('wheel_circ'),
        #         'base_width': LaunchConfiguration('base_width'),
        #         'pub_odom': LaunchConfiguration('pub_odom_back'),
        #         'stop_movement': LaunchConfiguration('stop_movement'),
        #         'max_accel': LaunchConfiguration('max_accel_back'),
        #         'name': 'roboclaw_back',
        #     }]
        # ),
        Node(
            package='roboclaw_ros',
            executable='roboclaw_plow',
            name='roboclaw_plow',
            output='screen',
            respawn=True,
            condition=IfCondition(LaunchConfiguration('run_diag')),
            parameters=[{
                'dev': '/dev/ttyACM0',
                'baud': LaunchConfiguration('baud'),
                'address': 130,
                'max_duty': 127,
                'name': 'roboclaw_plow',
            }]
        ),
        # Node(
        #     package='diagnostic_aggregator',
        #     executable='aggregator_node',
        #     name='diagnostic_aggregator',
        #     parameters=[param_file_path],
        # )
    ]

    return LaunchDescription(launch_arguments + nodes)
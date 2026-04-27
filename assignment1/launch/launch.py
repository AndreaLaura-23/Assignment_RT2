from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot_ex.launch.py'
            )
        )
    )

    container = ComposableNodeContainer(
        name='assignment1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1',
                plugin='assignment1::ActionServer',
                name='action_server',
                parameters=[{
                    'target_frame_name': LaunchConfiguration('target_frame_name'),
                    'world_frame_name': LaunchConfiguration('world_frame_name'),
                    'moved_frame_name': LaunchConfiguration('moved_frame_name'),
                }],
            ),
            ComposableNode(
                package='assignment1',
                plugin='assignment1::ActionClient',
                name='action_client',
                parameters=[{
                    'target_frame_name': LaunchConfiguration('target_frame_name'),
                    'world_frame_name': LaunchConfiguration('world_frame_name'),
                }],
            ),
        ],
        output='screen',
    )

    user_interface_node = Node(
        package='assignment1',
        executable='user_interface',
        name='user_interface',
        output='screen',
        prefix='xterm -title "User Interface" -e',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_frame_name',
            default_value='odom',
        ),

        DeclareLaunchArgument(
            'moved_frame_name',
            default_value='base_link',
        ),

        DeclareLaunchArgument(
            'target_frame_name',
            default_value='goal_frame',
        ),

        gazebo_launch,
        container,
        user_interface_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container = ComposableNodeContainer(
        name='assignment1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1',
                plugin='assignment1::ActionServer',
                name='action_server'
            ),
            ComposableNode(
                package='assignment1',
                plugin='assignment1::ActionClient',
                name='action_client'
            ),
        ],
        output='screen',
    )

    user_interface_node = Node(
        package='assignment1',
        executable='user_interface',
        name='user_interface',
        output='screen'
    )

    return LaunchDescription([
        container,
        user_interface_node
    ])
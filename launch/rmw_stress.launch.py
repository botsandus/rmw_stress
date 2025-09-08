from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch 100 publisher/subscriber pairs on different namespaces
    for i in range(100):
        ld.add_action(Node(
            package='examples_rclcpp_minimal_publisher',
            executable='publisher_member_function',
            name=f'pub_{i}',
            namespace=f'pub_sub_ns_{i}',
            arguments=['--ros-args', '--log-level', 'WARN']
        ))
        ld.add_action(Node(
            package='examples_rclcpp_minimal_subscriber',
            executable='subscriber_member_function',
            name=f'sub_{i}',
            namespace=f'pub_sub_ns_{i}',
            arguments=['--ros-args', '--log-level', 'WARN']
        ))

    # Launch 10 publisher/subscriber all communicating on the same topic
    for i in range(10):
        ld.add_action(Node(
            package='examples_rclcpp_minimal_publisher',
            executable='publisher_member_function',
            name=f'pub_{i}',
            arguments=['--ros-args', '--log-level', 'WARN']
        ))
        ld.add_action(Node(
            package='examples_rclcpp_minimal_subscriber',
            executable='subscriber_member_function',
            name=f'sub_{i}',
            arguments=['--ros-args', '--log-level', 'WARN']
        ))

    # Launch 10 action server/client pairs on different namespace
    # Clients will close after receiving their goal result
    for j in range(10):
        ld.add_action(Node(
            package='examples_rclcpp_minimal_action_server',
            executable='action_server_member_functions',
            name=f'action_server_{j}',
            namespace=f'action_ns_{j}',
        ))
        ld.add_action(Node(
            package='examples_rclcpp_minimal_action_client',
            executable='action_client_member_functions',
            name=f'action_client_{j}',
            namespace=f'action_ns_{j}',
        ))

    return ld

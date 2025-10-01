from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    message_size_bytes = 256
    # Up to 134MB
    for i in range(20):
        ld.add_action(Node(
            package='rmw_stress',
            executable='transient_local_publisher',
            name=f'transient_pub_{i}',
            namespace=f'transient_pub_sub_ns_{i}',
            parameters=[{'message_size_bytes': message_size_bytes}],
            arguments=['--ros-args', '--log-level', 'INFO']
        ))
        ld.add_action(Node(
            package='rmw_stress',
            executable='transient_local_subscriber',
            name=f'transient_sub_{i}',
            namespace=f'transient_pub_sub_ns_{i}',
            arguments=['--ros-args', '--log-level', 'INFO']
        ))
        message_size_bytes *= 2

    return ld

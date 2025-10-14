from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trajectory_publisher_node = Node(
        package='sorting_bot',
        executable='trajectory_publisher',
        output='screen',
    )

    nodes = [trajectory_publisher_node]

    return LaunchDescription(nodes) 

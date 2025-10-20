from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joint_publisher_node = Node(
        package='sorting_bot',
        executable='joint_trajectory_publisher',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    nodes = [joint_publisher_node]

    return LaunchDescription(nodes) 

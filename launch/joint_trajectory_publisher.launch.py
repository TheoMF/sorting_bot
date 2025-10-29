from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joint_trajectory_publisher_params = PathJoinSubstitution(
        [
            FindPackageShare("sorting_bot"),
            "config",
            "joint_trajectory_publisher_params.yaml",
        ]
    )
    joint_publisher_node = Node(
        package='sorting_bot',
        executable='joint_trajectory_publisher',
        output='screen',
        parameters=[{"use_sim_time": True}, joint_trajectory_publisher_params]
    )

    nodes = [joint_publisher_node]

    return LaunchDescription(nodes) 

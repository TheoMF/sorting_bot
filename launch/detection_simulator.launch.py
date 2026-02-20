from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    detection_simulator_params_file = PathJoinSubstitution(
        [
            FindPackageShare("sorting_bot"),
            "config",
            "detection_simulator_params.yaml",
        ]
    )
    detection_simulator_node = Node(
        package='sorting_bot',
        executable='detection_simulator',
        output='screen',
        parameters=[detection_simulator_params_file]
        
    )

    nodes = [detection_simulator_node]

    return LaunchDescription(nodes) 

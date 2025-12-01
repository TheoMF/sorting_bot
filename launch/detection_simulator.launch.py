from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detection_simulator_node = Node(
        package='sorting_bot',
        executable='detection_simulator',
        output='screen',
        parameters=[{"use_sim_time": True}]
        
    )

    nodes = [detection_simulator_node]

    return LaunchDescription(nodes) 

from launch import LaunchDescription, LaunchContext
from launch.actions import  DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) :
    robot_name = LaunchConfiguration("robot_name")
    robot_is_so101 = context.perform_substitution(robot_name).lower() == "so-101"
    if robot_is_so101:
        parameters_file = "joint_trajectory_publisher_params.yaml"
    else:
        parameters_file = "joint_trajectory_publisher_lekiwi_params.yaml"
    joint_trajectory_publisher_params = PathJoinSubstitution(
        [
            FindPackageShare("sorting_bot"),
            "config",
            parameters_file,
        ]
    )
    joint_publisher_node = Node(
        package='sorting_bot',
        executable='joint_trajectory_publisher',
        output='screen',
        parameters=[joint_trajectory_publisher_params],
        #arguments=["--ros-args", "--log-level", "debug"]
    )

    return [joint_publisher_node]
    
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="lekiwi",
            description="Name of the robot to use.",
            choices=["so-101", "lekiwi"],
        ),
        ]
    

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    camera_param = LaunchConfiguration("camera")
    camera = context.perform_substitution(camera_param).lower()
    if camera== "hand":
        param_camera_file = "param_hand_camera.yaml"
    else:
        param_camera_file = "param_base_camera.yaml"
    apriltag_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("sorting_bot"),
                "config",
                "tags_36h11.yaml",
            ]
        ),
    )
    camera_parameters_yaml = PathJoinSubstitution(
        [
            FindPackageShare("sorting_bot"),
            "config",
            param_camera_file,
        ]
    )
    container = ComposableNodeContainer(
        name=f"{camera}_image_container",
        namespace="/camera",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_cam",
                plugin="usb_cam::UsbCamNode",
                name=f"{camera}_usb_cam_node_exe",
                parameters=[
                    ParameterFile(
                        param_file=camera_parameters_yaml,
                        allow_substs=True,
                    )
                ],
                remappings=[
                    ("/camera_info", f"/{camera}/rectify/camera_info"),
                    ("/image_raw", f"/{camera}/image_raw"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=f"{camera}_rectify",
                namespace="rectify",
                remappings=[
                    ("/rectify/camera_info", f"/{camera}/rectify/camera_info"),
                    ("/rectify/image", f"/{camera}/image_raw"),
                    ("/rectify/image_rect", f"/{camera}/rectify/image_rect"),
                    
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name=f"{camera}_apriltag",
                namespace="apriltag",
                remappings=[
                    ("/rectify/camera_info", f"/{camera}/rectify/camera_info"),
                    ("/apriltag/image_rect", f"/{camera}/rectify/image_rect"),
                ],
                parameters=[
                    {"history": "keep_last"},
                    apriltag_params,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )
    return [container]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "camera",
            default_value="hand",
            description="Which camera to use.",
            choices=["hand", "base"],
        ),
        ]
    return LaunchDescription(declared_arguments+[OpaqueFunction(function=launch_setup)])

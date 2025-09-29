from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
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
            "param_camera.yaml",
        ]
    )
    container = ComposableNodeContainer(
        name="image_container",
        namespace="/camera",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_cam",
                plugin="usb_cam::UsbCamNode",
                name="usb_cam_node_exe",
                parameters=[
                    ParameterFile(
                        param_file=camera_parameters_yaml,
                        allow_substs=True,
                    )
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                namespace="rectify",
                remappings=[
                    ("/rectify/camera_info", "/camera_info"),
                    ("/rectify/image", "/image_raw"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                remappings=[
                    ("/rectify/camera_info", "/camera_info"),
                    ("/apriltag/image_rect", "/rectify/image_rect"),
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
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

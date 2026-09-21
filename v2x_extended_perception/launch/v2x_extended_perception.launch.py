import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.actions import RegisterEventHandler


def launch_setup(context, *args, **kwargs):

    launch_items = []

    params_etsi = os.path.join(
        get_package_share_directory("v2x_extended_perception"),
        "config",
        "etsi_its_messages.param.yaml",
    )
    params_udp_sender = os.path.join(
        get_package_share_directory("v2x_extended_perception"),
        "config",
        "udp_driver_sender.param.yaml",
    )
    params_udp_receiver = os.path.join(
        get_package_share_directory("v2x_extended_perception"),
        "config",
        "udp_driver_receiver.param.yaml",
    )
    params_v2x_cam_extended_perception = os.path.join(
        get_package_share_directory("v2x_cam_extended_perception"),
        "config",
        "v2x_cam_extended_perception.param.yaml",
    )

    udp_lifecycle_transitions_script_path = os.path.join(
        get_package_share_directory("v2x_extended_perception"),
        "scripts",
        "udp_lifecyclenode_transtions.sh",
    )
    
    if LaunchConfiguration("launch_cam_perception").perform(context).lower() == "true":

        v2x_cam_extended_perception = Node(
            package="v2x_cam_extended_perception",
            executable="v2x_cam_extended_perception",
            namespace="v2x",
            name="v2x_cam_extended_perception",
            parameters=[params_v2x_cam_extended_perception],
            remappings=[("cam/out", "/v2x/parser/etsi_parser/cam/out")],
            output="both",
        )
        launch_items.append(v2x_cam_extended_perception)

        launch_items.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=v2x_cam_extended_perception,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            )
        )

    if LaunchConfiguration("launch_drivers").perform(context).lower() == "true":

        etsi_its_message_converter = Node(
            package="etsi_its_conversion",
            executable="etsi_its_conversion_node",
            namespace="v2x/parser",
            name="etsi_parser",
            parameters=[params_etsi],
            remappings=[
                ("etsi_parser/udp/in", "/v2x/driver/udp_read"),
                ("etsi_parser/udp/out", "/v2x/driver/udp_write"),
            ],
            output="both",
        )
        launch_items.append(etsi_its_message_converter)

        udp_driver_sender = Node(
            package="udp_driver",
            executable="udp_sender_node_exe",
            namespace="v2x/driver",
            name="udp_sender",
            parameters=[params_udp_sender],
            output="both",
        )
        launch_items.append(udp_driver_sender)

        udp_driver_receiver = Node(
            package="udp_driver",
            executable="udp_receiver_node_exe",
            namespace="v2x/driver",
            name="udp_receiver",
            parameters=[params_udp_receiver],
            output="both",
        )
        launch_items.append(udp_driver_receiver)

        udp_lifecycle_transitions_script = ExecuteProcess(
            cmd=["bash", udp_lifecycle_transitions_script_path], output="screen"
        )
        launch_items.append(udp_lifecycle_transitions_script)

        launch_items.extend(
            [
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=udp_driver_sender,
                        on_exit=[EmitEvent(event=Shutdown())],
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=udp_driver_receiver,
                        on_exit=[EmitEvent(event=Shutdown())],
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=etsi_its_message_converter,
                        on_exit=[EmitEvent(event=Shutdown())],
                    )
                ),
            ]
        )

    return launch_items


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_drivers", default_value="true"),
            DeclareLaunchArgument("launch_cam_perception", default_value="true"),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )

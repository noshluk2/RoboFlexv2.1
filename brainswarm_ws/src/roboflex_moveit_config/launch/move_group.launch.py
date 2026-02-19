from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hardware_urdf = os.path.join(
        get_package_share_directory("roboflex_description"),
        "urdf",
        "real_hardware.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("roboflex_description", package_name="roboflex_moveit_config")
        .robot_description(file_path=hardware_urdf)
        .to_moveit_configs()
    )
    generated_ld = generate_move_group_launch(moveit_config)

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        )
    )
    ld.add_action(
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("use_sim_time"),
        )
    )
    for entity in generated_ld.entities:
        ld.add_action(entity)
    return ld

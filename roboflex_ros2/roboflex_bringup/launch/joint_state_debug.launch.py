from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("roboflex_bringup"), "launch", "urdf_joint_state_debug.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_description"), "urdf", "real_hardware.urdf.xacro"]
                ),
                description="Path to robot URDF/Xacro used for visualization",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_bringup"), "rviz", "urdf_joint_state_debug.rviz"]
                ),
                description="RViz config path",
            ),
            target_launch,
        ]
    )

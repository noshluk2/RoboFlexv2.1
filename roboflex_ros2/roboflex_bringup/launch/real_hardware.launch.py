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
    firmware_ip = LaunchConfiguration("firmware_ip")
    firmware_port = LaunchConfiguration("firmware_port")
    udp_keepalive_ms = LaunchConfiguration("udp_keepalive_ms")
    udp_command_change_epsilon_rad = LaunchConfiguration("udp_command_change_epsilon_rad")
    with_moveit = LaunchConfiguration("with_moveit")

    target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_bringup"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
            "firmware_ip": firmware_ip,
            "firmware_port": firmware_port,
            "udp_keepalive_ms": udp_keepalive_ms,
            "udp_command_change_epsilon_rad": udp_command_change_epsilon_rad,
            "with_moveit": with_moveit,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_description"), "urdf", "real_hardware.urdf.xacro"]
                ),
            ),
            DeclareLaunchArgument("firmware_ip", default_value="255.255.255.255"),
            DeclareLaunchArgument("firmware_port", default_value="9999"),
            DeclareLaunchArgument("udp_keepalive_ms", default_value="200"),
            DeclareLaunchArgument("udp_command_change_epsilon_rad", default_value="0.0001"),
            DeclareLaunchArgument("with_moveit", default_value="true"),
            target_launch,
        ]
    )

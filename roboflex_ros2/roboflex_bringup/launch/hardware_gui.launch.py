from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
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

    full_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_moveit"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
            "firmware_ip": firmware_ip,
            "firmware_port": firmware_port,
            "udp_keepalive_ms": udp_keepalive_ms,
            "udp_command_change_epsilon_rad": udp_command_change_epsilon_rad,
        }.items(),
        condition=IfCondition(with_moveit),
    )

    control_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_control"), "launch", "real_hardware.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
            "firmware_ip": firmware_ip,
            "firmware_port": firmware_port,
            "udp_keepalive_ms": udp_keepalive_ms,
            "udp_command_change_epsilon_rad": udp_command_change_epsilon_rad,
        }.items(),
        condition=UnlessCondition(with_moveit),
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
                description="Path to robot URDF/Xacro for hardware control",
            ),
            DeclareLaunchArgument(
                "firmware_ip",
                default_value="255.255.255.255",
                description="ESP32 firmware UDP target IP for motor commands",
            ),
            DeclareLaunchArgument(
                "firmware_port",
                default_value="9999",
                description="ESP32 firmware UDP listening port for motor commands",
            ),
            DeclareLaunchArgument(
                "udp_keepalive_ms",
                default_value="200",
                description="Send unchanged UDP command keepalive interval in milliseconds",
            ),
            DeclareLaunchArgument(
                "udp_command_change_epsilon_rad",
                default_value="0.0001",
                description="Minimum radian command delta that triggers UDP send",
            ),
            DeclareLaunchArgument(
                "with_moveit",
                default_value="true",
                description="Launch MoveIt and RViz with hardware",
            ),
            full_stack_launch,
            control_only_launch,
        ]
    )

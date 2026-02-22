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
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    micro_ros_transport = LaunchConfiguration("micro_ros_transport")
    micro_ros_port = LaunchConfiguration("micro_ros_port")
    with_moveit = LaunchConfiguration("with_moveit")

    target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_bringup"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
            "start_micro_ros_agent": start_micro_ros_agent,
            "micro_ros_transport": micro_ros_transport,
            "micro_ros_port": micro_ros_port,
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
            DeclareLaunchArgument("start_micro_ros_agent", default_value="false"),
            DeclareLaunchArgument("micro_ros_transport", default_value="udp4"),
            DeclareLaunchArgument("micro_ros_port", default_value="8888"),
            DeclareLaunchArgument("with_moveit", default_value="true"),
            target_launch,
        ]
    )

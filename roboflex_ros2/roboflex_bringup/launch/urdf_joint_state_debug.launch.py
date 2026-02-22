from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    robot_description_content = ParameterValue(
        Command(["xacro", " ", robot_description_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
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
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )

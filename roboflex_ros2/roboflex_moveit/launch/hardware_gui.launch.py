from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    firmware_ip = LaunchConfiguration("firmware_ip")
    firmware_port = LaunchConfiguration("firmware_port")
    udp_keepalive_ms = LaunchConfiguration("udp_keepalive_ms")
    udp_command_change_epsilon_rad = LaunchConfiguration("udp_command_change_epsilon_rad")

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                robot_description_file,
                " ",
                "udp_target_ip:=",
                firmware_ip,
                " ",
                "udp_target_port:=",
                firmware_port,
                " ",
                "udp_keepalive_ms:=",
                udp_keepalive_ms,
                " ",
                "udp_command_change_epsilon_rad:=",
                udp_command_change_epsilon_rad,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    ros2_controllers = PathJoinSubstitution(
        [FindPackageShare("roboflex_control"), "config", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("roboflex_moveit"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("roboflex_moveit"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    start_arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    start_gripper_controller_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    start_moveit_after_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_launch, moveit_rviz_launch],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
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
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_description"), "urdf", "real_hardware.urdf.xacro"]
                ),
                description="Path to robot URDF/Xacro for hardware control",
            ),
            control_node,
            robot_state_publisher,
            start_arm_controller_after_jsb,
            start_gripper_controller_after_arm,
            start_moveit_after_gripper_controller,
            joint_state_broadcaster_spawner,
        ]
    )

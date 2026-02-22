from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")

    robot_description_content = ParameterValue(
        Command(["xacro", " ", robot_description_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare("roboflex_control"),
            "config",
            "hardware_joint_position_test_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_config, {"use_sim_time": use_sim_time}],
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

    joint_1_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_1_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_2_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_2_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_3_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_3_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_4_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_4_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_gripper_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_gripper_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_position_controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                joint_1_position_spawner,
                joint_2_position_spawner,
                joint_3_position_spawner,
                joint_4_position_spawner,
                joint_gripper_position_spawner,
            ],
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
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_description"), "urdf", "real_hardware.urdf.xacro"]
                ),
                description="Path to robot URDF/Xacro",
            ),
            control_node,
            robot_state_publisher,
            start_position_controllers_after_jsb,
            joint_state_broadcaster_spawner,
        ]
    )

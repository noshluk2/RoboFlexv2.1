import json
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_group_state_values(srdf_path: str, group_state_name: str, group_name: str):
    try:
        root = ET.parse(srdf_path).getroot()
    except Exception as exc:
        return None, None, f"Failed to parse SRDF '{srdf_path}': {exc}"

    for group_state in root.findall("group_state"):
        if (
            group_state.get("name") == group_state_name
            and group_state.get("group") == group_name
        ):
            joint_names = []
            positions = []
            for joint in group_state.findall("joint"):
                joint_name = joint.get("name")
                joint_value = joint.get("value")
                if joint_name is None or joint_value is None:
                    continue
                joint_names.append(joint_name)
                positions.append(float(joint_value))

            if not joint_names:
                return None, None, (
                    f"SRDF group_state '{group_state_name}' in group '{group_name}' has no joints"
                )
            return joint_names, positions, None

    return None, None, (
        f"SRDF group_state '{group_state_name}' (group '{group_name}') was not found"
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    auto_move_to_arm_strech = LaunchConfiguration("auto_move_to_arm_strech")

    robot_description_content = ParameterValue(
        Command(["xacro", " ", robot_description_file]),
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

    srdf_path = os.path.join(
        get_package_share_directory("roboflex_moveit"),
        "config",
        "roboflex_description.srdf",
    )
    start_joint_names, start_positions, start_pose_error = _load_group_state_values(
        srdf_path=srdf_path,
        group_state_name="Arm Strech",
        group_name="arm",
    )

    move_to_start_pose_actions = []
    if start_joint_names and start_positions:
        # One-shot startup command that moves arm_controller to the SRDF "Arm Strech" state.
        goal_msg = {
            "trajectory": {
                "joint_names": start_joint_names,
                "points": [
                    {
                        "positions": start_positions,
                        "time_from_start": {"sec": 4, "nanosec": 0},
                    }
                ],
            }
        }
        move_to_start_pose_actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "action",
                            "send_goal",
                            "--wait",
                            "/arm_controller/follow_joint_trajectory",
                            "control_msgs/action/FollowJointTrajectory",
                            json.dumps(goal_msg),
                        ],
                        output="screen",
                        condition=IfCondition(auto_move_to_arm_strech),
                    )
                ],
            )
        )
    else:
        move_to_start_pose_actions.append(LogInfo(msg=f"[WARN] {start_pose_error}"))

    start_arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    start_moveit_after_arm_controller = RegisterEventHandler(
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

    start_pose_after_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=move_to_start_pose_actions,
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
                "auto_move_to_arm_strech",
                default_value="true",
                description=(
                    "Send a startup trajectory to arm_controller for SRDF group_state 'Arm Strech'"
                ),
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
            start_moveit_after_arm_controller,
            start_moveit_after_gripper_controller,
            start_pose_after_gripper_controller,
            joint_state_broadcaster_spawner,
        ]
    )

import os

import launch
import xacro
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_moveit = LaunchConfiguration("start_moveit")

    description_share = get_package_share_directory("roboflex_description")
    bringup_share = get_package_share_directory("roboflex_bringup")

    urdf_xacro_file = os.path.join(description_share, "urdf", "roboflex_robotic_arm.urdf.xacro")
    world_file = os.path.join(description_share, "worlds", "simulation_world_new.sdf")
    bridge_config = os.path.join(bringup_share, "config", "ros_gz_bridge.yaml")
    robot_controllers = os.path.join(
        get_package_share_directory("roboflex_control"),
        "config",
        "roboflex_controllers.yaml",
    )

    with open(urdf_xacro_file, "r", encoding="utf-8") as urdf_stream:
        doc = xacro.parse(urdf_stream)
    xacro.process_doc(doc)

    models_path = os.path.join(description_share, "models")
    description_share_parent = os.path.dirname(description_share)
    resource_paths = ":".join([models_path, description_share_parent, description_share])
    ign_resource_paths = resource_paths
    gz_resource_paths = resource_paths
    if os.environ.get("IGN_GAZEBO_RESOURCE_PATH"):
        ign_resource_paths = f"{ign_resource_paths}:{os.environ['IGN_GAZEBO_RESOURCE_PATH']}"
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        gz_resource_paths = f"{gz_resource_paths}:{os.environ['GZ_SIM_RESOURCE_PATH']}"

    try:
        gazebo_pkg = "ros_gz_sim"
        gazebo_launch_file = "gz_sim.launch.py"
        gazebo_launch_args = {"gz_args": f"-r -v 4 {world_file}"}
        get_package_share_directory(gazebo_pkg)
    except PackageNotFoundError:
        gazebo_pkg = "ros_ign_gazebo"
        gazebo_launch_file = "ign_gazebo.launch.py"
        gazebo_launch_args = {"ign_args": f"-r -v 4 {world_file}"}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(gazebo_pkg), "launch", gazebo_launch_file)]
        ),
        launch_arguments=gazebo_launch_args.items(),
    )

    spawn_robot = Node(
        package=gazebo_pkg,
        executable="create",
        arguments=[
            "-string",
            doc.toxml(),
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-x",
            "0.6",
            "-y",
            "0",
            "-z",
            "1.0155",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": doc.toxml()}, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_node",
        arguments=["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", robot_controllers],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--param-file", robot_controllers],
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("roboflex_moveit"),
                    "launch",
                    "move_group_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_moveit),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("roboflex_moveit"),
                    "launch",
                    "moveit_rviz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_moveit),
    )

    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    start_moveit_after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_launch, moveit_rviz_launch],
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="roboflex_v2_1",
                description="Name used when spawning robot in Gazebo",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "start_moveit",
                default_value="true",
                description="Start MoveIt move_group and RViz after controllers",
            ),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_paths),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_paths),
            gazebo,
            bridge,
            camera_bridge,
            robot_state_publisher,
            spawn_robot,
            start_arm_after_jsb,
            start_gripper_after_arm,
            start_moveit_after_gripper,
            joint_state_broadcaster_spawner,
        ]
    )

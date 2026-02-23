from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_moveit = LaunchConfiguration("with_moveit")

    firmware_ip = LaunchConfiguration("firmware_ip")
    firmware_port = LaunchConfiguration("firmware_port")
    udp_keepalive_ms = LaunchConfiguration("udp_keepalive_ms")
    udp_command_change_epsilon_rad = LaunchConfiguration("udp_command_change_epsilon_rad")

    start_realsense = LaunchConfiguration("start_realsense")
    realsense_camera_namespace = LaunchConfiguration("realsense_camera_namespace")
    realsense_camera_name = LaunchConfiguration("realsense_camera_name")

    scene_config = LaunchConfiguration("scene_config")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    publish_camera_static_tf = LaunchConfiguration("publish_camera_static_tf")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    record_bag = LaunchConfiguration("record_bag")
    bag_output_dir = LaunchConfiguration("bag_output_dir")
    bag_name = LaunchConfiguration("bag_name")

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_bringup"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "with_moveit": with_moveit,
            "firmware_ip": firmware_ip,
            "firmware_port": firmware_port,
            "udp_keepalive_ms": udp_keepalive_ms,
            "udp_command_change_epsilon_rad": udp_command_change_epsilon_rad,
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ),
        launch_arguments={
            "camera_namespace": realsense_camera_namespace,
            "camera_name": realsense_camera_name,
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
        }.items(),
        condition=IfCondition(start_realsense),
    )

    camera_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_pick_place"), "launch", "external_camera_scene.launch.py"])
        ),
        launch_arguments={
            "scene_config": scene_config,
            "pointcloud_topic": pointcloud_topic,
            "base_frame": base_frame,
            "camera_frame": camera_frame,
            "publish_camera_static_tf": publish_camera_static_tf,
            "camera_x": camera_x,
            "camera_y": camera_y,
            "camera_z": camera_z,
            "camera_roll": camera_roll,
            "camera_pitch": camera_pitch,
            "camera_yaw": camera_yaw,
        }.items(),
    )

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            PathJoinSubstitution([bag_output_dir, bag_name]),
            pointcloud_topic,
            "/camera/color/image_raw",
            "/camera/color/camera_info",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/aligned_depth_to_color/camera_info",
            "/tf",
            "/tf_static",
            "/joint_states",
            "/planning_scene",
            "/move_group/monitored_planning_scene",
            "/display_planned_path",
            "/arm_controller/joint_trajectory",
            "/arm_controller/state",
            "/gripper_controller/joint_trajectory",
            "/gripper_controller/state",
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "with_moveit",
                default_value="true",
                description="Start MoveIt move_group + RViz in hardware_gui launch.",
            ),
            DeclareLaunchArgument("firmware_ip", default_value="255.255.255.255"),
            DeclareLaunchArgument("firmware_port", default_value="9999"),
            DeclareLaunchArgument("udp_keepalive_ms", default_value="200"),
            DeclareLaunchArgument("udp_command_change_epsilon_rad", default_value="0.0001"),
            DeclareLaunchArgument(
                "start_realsense",
                default_value="true",
                description="Start RealSense camera driver using rs_launch.py.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_namespace",
                default_value="/",
                description="Camera namespace for rs_launch.py.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_name",
                default_value="camera",
                description="Camera name for rs_launch.py.",
            ),
            DeclareLaunchArgument(
                "scene_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_pick_place"), "config", "scene_integration.yaml"]
                ),
                description="Path to scene integration parameter YAML.",
            ),
            DeclareLaunchArgument("pointcloud_topic", default_value="/camera/depth/color/points"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("publish_camera_static_tf", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.24"),
            DeclareLaunchArgument("camera_y", default_value="0.00"),
            DeclareLaunchArgument("camera_z", default_value="0.265"),
            DeclareLaunchArgument("camera_roll", default_value="0.00"),
            DeclareLaunchArgument("camera_pitch", default_value="0.261799"),
            DeclareLaunchArgument("camera_yaw", default_value="3.14159"),
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description="Record perception + planning + controller topics while operating.",
            ),
            DeclareLaunchArgument(
                "bag_output_dir",
                default_value=".",
                description="Absolute directory where rosbag folder will be created.",
            ),
            DeclareLaunchArgument("bag_name", default_value="pick_place_operation"),
            robot_stack,
            realsense_launch,
            camera_scene,
            bag_record,
        ]
    )

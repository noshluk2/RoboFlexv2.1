import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_realsense_launch_file() -> tuple[str, bool]:
    realsense_share = get_package_share_directory("realsense2_camera")
    launch_dir = os.path.join(realsense_share, "launch")
    legacy_d455 = os.path.join(launch_dir, "rs_d455_pointcloud_launch.py")
    generic_rs = os.path.join(launch_dir, "rs_launch.py")

    if os.path.exists(legacy_d455):
        return legacy_d455, False
    if os.path.exists(generic_rs):
        return generic_rs, True

    raise FileNotFoundError(
        "No supported realsense launch file found. "
        "Expected rs_d455_pointcloud_launch.py or rs_launch.py."
    )


def generate_launch_description() -> LaunchDescription:
    start_realsense = LaunchConfiguration("start_realsense")
    start_moveit = LaunchConfiguration("start_moveit")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
    scene_config = LaunchConfiguration("scene_config")

    record_bag = LaunchConfiguration("record_bag")
    bag_output_dir = LaunchConfiguration("bag_output_dir")
    bag_name = LaunchConfiguration("bag_name")
    realsense_camera_namespace = LaunchConfiguration("realsense_camera_namespace")
    realsense_camera_name = LaunchConfiguration("realsense_camera_name")

    realsense_launch_file, use_generic_rs_launch = _resolve_realsense_launch_file()

    if use_generic_rs_launch:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                "camera_namespace": realsense_camera_namespace,
                "camera_name": realsense_camera_name,
                "pointcloud.enable": "true",
                "align_depth.enable": "true",
            }.items(),
            condition=IfCondition(start_realsense),
        )
    else:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            condition=IfCondition(start_realsense),
        )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_moveit"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_moveit),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_moveit"), "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_rviz),
    )

    camera_scene_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("roboflex_pick_place"), "launch", "external_camera_scene.launch.py"]
            )
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
            "/tf",
            "/tf_static",
            "/joint_states",
            "/planning_scene",
            "/move_group/monitored_planning_scene",
            "/camera/color/image_raw",
            "/camera/color/camera_info",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/aligned_depth_to_color/camera_info",
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_pick_place"), "config", "scene_integration.yaml"]
                ),
                description="Path to scene integration parameter YAML.",
            ),
            DeclareLaunchArgument(
                "start_realsense",
                default_value="true",
                description="Launch realsense2_camera D455 pointcloud launch.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_namespace",
                default_value="/",
                description="Namespace passed to rs_launch.py fallback.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_name",
                default_value="camera",
                description="Camera name passed to rs_launch.py fallback.",
            ),
            DeclareLaunchArgument(
                "start_moveit",
                default_value="true",
                description="Launch MoveIt move_group (provides /apply_planning_scene).",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Launch MoveIt RViz (set true when camera launch does not publish its own model RViz).",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/color/points",
                description="PointCloud2 topic for perception and bagging.",
            ),
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
                description="Record dataset topics with ros2 bag.",
            ),
            DeclareLaunchArgument(
                "bag_output_dir",
                default_value=".",
                description="Directory where bag folder will be created.",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value="pick_place_prep",
                description="Name of bag directory under bag_output_dir.",
            ),
            realsense_launch,
            move_group_launch,
            moveit_rviz_launch,
            camera_scene_launch,
            bag_record,
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bag_path = LaunchConfiguration("bag_path")
    start_bag_playback = LaunchConfiguration("start_bag_playback")
    bag_rate = LaunchConfiguration("bag_rate")

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    segmentation_config = LaunchConfiguration("segmentation_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    publish_camera_static_tf = LaunchConfiguration("publish_camera_static_tf")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    segmentation_node = Node(
        package="roboflex_pick_place",
        executable="object_segmentation_node",
        name="object_segmentation_node",
        output="screen",
        parameters=[
            segmentation_config,
            {
                "input_topic": pointcloud_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d455_to_robot_base_tf_replay",
        arguments=[
            "--x",
            camera_x,
            "--y",
            camera_y,
            "--z",
            camera_z,
            "--roll",
            camera_roll,
            "--pitch",
            camera_pitch,
            "--yaw",
            camera_yaw,
            "--frame-id",
            base_frame,
            "--child-frame-id",
            camera_frame,
        ],
        output="screen",
        condition=IfCondition(publish_camera_static_tf),
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            bag_rate,
        ],
        output="screen",
        condition=IfCondition(start_bag_playback),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "segmentation_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboflex_pick_place"), "config", "object_segmentation.yaml"]
                ),
                description="Path to object segmentation parameter YAML.",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/color/points",
                description="PointCloud2 topic to segment from recorded bag.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use bag /clock while replaying data.",
            ),
            DeclareLaunchArgument(
                "start_bag_playback",
                default_value="true",
                description="Start ros2 bag playback from this launch.",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="/tmp/your_bag",
                description="Path to bag folder for playback.",
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument(
                "publish_camera_static_tf",
                default_value="false",
                description="Publish static transform base_frame -> camera_frame when bag does not contain TF.",
            ),
            DeclareLaunchArgument("camera_x", default_value="0.24"),
            DeclareLaunchArgument("camera_y", default_value="0.00"),
            DeclareLaunchArgument("camera_z", default_value="0.265"),
            DeclareLaunchArgument("camera_roll", default_value="0.00"),
            DeclareLaunchArgument("camera_pitch", default_value="0.261799"),
            DeclareLaunchArgument("camera_yaw", default_value="3.14159"),
            static_tf_node,
            segmentation_node,
            bag_play,
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
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

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d455_to_robot_base_tf",
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

    scene_integration_node = Node(
        package="roboflex_pick_place",
        executable="scene_integration_node",
        name="scene_integration_node",
        output="screen",
        parameters=[
            scene_config,
            {
                "pointcloud_topic": pointcloud_topic,
                "base_frame": base_frame,
            },
        ],
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
                "pointcloud_topic",
                default_value="/camera/depth/color/points",
                description="PointCloud2 topic from external D455.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame used by MoveIt.",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_link",
                description="External D455 frame id (RealSense default root is camera_link).",
            ),
            DeclareLaunchArgument(
                "publish_camera_static_tf",
                default_value="true",
                description="Publish static transform base_frame -> camera_frame from launch args.",
            ),
            DeclareLaunchArgument("camera_x", default_value="0.24"),
            DeclareLaunchArgument("camera_y", default_value="0.00"),
            DeclareLaunchArgument("camera_z", default_value="0.265"),
            DeclareLaunchArgument("camera_roll", default_value="0.00"),
            DeclareLaunchArgument("camera_pitch", default_value="0.261799"),
            DeclareLaunchArgument("camera_yaw", default_value="3.14159"),
            static_tf_node,
            scene_integration_node,
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    firmware_ip = LaunchConfiguration("firmware_ip")
    firmware_port = LaunchConfiguration("firmware_port")
    udp_keepalive_ms = LaunchConfiguration("udp_keepalive_ms")
    udp_command_change_epsilon_rad = LaunchConfiguration("udp_command_change_epsilon_rad")
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

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_bringup"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "with_moveit": "true",
            "firmware_ip": firmware_ip,
            "firmware_port": firmware_port,
            "udp_keepalive_ms": udp_keepalive_ms,
            "udp_command_change_epsilon_rad": udp_command_change_epsilon_rad,
        }.items(),
    )

    camera_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_pick_place"), "launch", "external_camera_scene.launch.py"])
        ),
        launch_arguments={
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("firmware_ip", default_value="255.255.255.255"),
            DeclareLaunchArgument("firmware_port", default_value="9999"),
            DeclareLaunchArgument("udp_keepalive_ms", default_value="200"),
            DeclareLaunchArgument("udp_command_change_epsilon_rad", default_value="0.0001"),
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
            robot_stack,
            camera_scene,
        ]
    )

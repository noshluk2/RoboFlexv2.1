from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    micro_ros_transport = LaunchConfiguration("micro_ros_transport")
    micro_ros_port = LaunchConfiguration("micro_ros_port")
    with_moveit = LaunchConfiguration("with_moveit")

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=[micro_ros_transport, "--port", micro_ros_port],
        output="screen",
        condition=IfCondition(start_micro_ros_agent),
    )

    full_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_moveit"), "launch", "hardware_gui.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
        }.items(),
        condition=IfCondition(with_moveit),
    )

    control_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("roboflex_control"), "launch", "real_hardware.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_description_file": robot_description_file,
        }.items(),
        condition=UnlessCondition(with_moveit),
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
                description="Path to robot URDF/Xacro for hardware control",
            ),
            DeclareLaunchArgument(
                "start_micro_ros_agent",
                default_value="false",
                description="Start micro-ROS agent in bringup (requires micro_ros_agent package)",
            ),
            DeclareLaunchArgument(
                "micro_ros_transport",
                default_value="udp4",
                description="micro-ROS transport argument",
            ),
            DeclareLaunchArgument(
                "micro_ros_port",
                default_value="8888",
                description="micro-ROS agent port",
            ),
            DeclareLaunchArgument(
                "with_moveit",
                default_value="true",
                description="Launch MoveIt and RViz with hardware",
            ),
            micro_ros_agent_node,
            full_stack_launch,
            control_only_launch,
        ]
    )

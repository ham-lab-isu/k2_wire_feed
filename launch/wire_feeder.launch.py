from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    teknic_node = Node(
        package="k2_wire_feed",
        executable="k2_wf",
        name="teknic_node",
        output="screen"
    )

    nodes = [
        teknic_node
    ]

    return LaunchDescription(declared_arguments + nodes)

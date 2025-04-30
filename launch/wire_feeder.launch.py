from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, Unlesscondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false')
    sim = LaunchConfiguration('sim')

    # Real hardware node
    teknic_node = Node(
        package="k2_wire_feed",
        executable="k2_wf",
        name="teknic_node",
        output="screen",
        condition=UnlessCondition(sim)
    )

    # Simulated node
    teknic_node_sim = Node(
        package="k2_wire_feed",
        executable="k2_wf_sim",
        name="teknic_node_sim",
        output="screen",
        condition=IfCondition(sim)
    )

    return LaunchDescription([
        sim_arg,
        teknic_node,
        teknic_node_sim
    ])

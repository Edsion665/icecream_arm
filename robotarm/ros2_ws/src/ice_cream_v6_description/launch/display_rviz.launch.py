"""RViz2 + robot_state_publisher + joint_state_publisher_gui for ice_cream v6."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ice_cream_v6_description")
    urdf_path = os.path.join(pkg_share, "urdf", "ice_cream_v6.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(pkg_share, "rviz", "icecream_arm_v6.rviz"),
                ],
            ),
        ]
    )

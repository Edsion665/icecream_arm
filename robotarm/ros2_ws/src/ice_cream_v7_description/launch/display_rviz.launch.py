"""RViz2 + robot_state_publisher + joint_state_publisher_gui for ice_cream v7."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ice_cream_v7_description")
    urdf_path = os.path.join(pkg_share, "urdf", "ice_cream_v7.urdf")
    rviz_cfg = os.path.join(pkg_share, "rviz", "icecream_arm_v7.rviz")
    if not os.path.isfile(rviz_cfg):
        raise FileNotFoundError(f"RViz 配置不存在: {rviz_cfg}（请先 colcon build 并 source workspace）")
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
                arguments=["-d", rviz_cfg],
            ),
        ]
    )

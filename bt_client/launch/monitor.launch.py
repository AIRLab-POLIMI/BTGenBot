import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  pkg_bt_client = get_package_share_directory('bt_client')

  bt_monitor_node_cmd = Node(
      package="bt_client",
      executable="bt_monitor",
      name="bt_monitor",
      output='screen'
  )

  ld = LaunchDescription()

  ld.add_action(bt_monitor_node_cmd)

  return ld

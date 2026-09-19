"""
cell_bringup.launch.py -- ONE-COMMAND launch for the UR5e pick-and-place cell.

Starts everything in the correct order with delays so the Modbus bridge is
up before the PLC-facing nodes (avoids the coil-write binding problem):

  0s  : Gazebo + UR5e + controllers (cell_sim.launch.py)
  6s  : modbus_bridge      (must be up before OpenPLC starts)
  8s  : gazebo_robot        (UR5e motion node)
  9s  : conveyor
 10s  : grasp_manager
 11s  : gripper_follower    (glues the visual gripper to tool0)
 12s  : part_spawner        (continuous flow; comment out for single-cube)

You still start the OpenPLC Runtime yourself, LAST, after this is up
(Connect -> transfer -> Start), then Reset -> Start to run cycles.

Usage:
  ros2 launch pick_place_cell_gazebo cell_bringup.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('pick_place_cell_gazebo')

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'cell_sim.launch.py')))

    def ctrl(exe):
        return Node(package='pick_place_cell_controller',
                    executable=exe, output='screen')

    return LaunchDescription([
        cell,                                                     # 0s
        TimerAction(period=6.0,  actions=[ctrl('modbus_bridge')]),
        TimerAction(period=8.0,  actions=[ctrl('gazebo_robot')]),
        TimerAction(period=9.0,  actions=[ctrl('conveyor')]),
        TimerAction(period=10.0, actions=[ctrl('grasp_manager')]),
        TimerAction(period=12.0, actions=[ctrl('part_spawner')]),
        TimerAction(period=11.0, actions=[ctrl('label_detector')]),
        TimerAction(period=12.0, actions=[ctrl('sorter')]),
        TimerAction(period=13.0, actions=[ctrl('part_spawner')]),
        
    ])

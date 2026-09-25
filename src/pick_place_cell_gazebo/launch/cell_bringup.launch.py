"""
cell_bringup.launch.py -- ONE-COMMAND launch for the UR5e pick-and-place cell.

Starts everything in the correct order with delays so the Modbus bridge is
up before the PLC-facing nodes (avoids the coil-write binding problem):

  0s  : Gazebo + UR5e + controllers (cell_sim.launch.py)
  6s  : modbus_bridge      (must be up before OpenPLC starts)
  8s  : gazebo_robot        (UR5e motion node)
  9s  : conveyor
 10s  : grasp_manager
 11s  : label_detector
 12s  : sorter
 13s  : part_spawner        (single randomized feeder)

You still start the OpenPLC Runtime yourself, LAST, after this is up
(Connect -> transfer -> Start), then Reset -> Start to run cycles.

Usage:
  ros2 launch pick_place_cell_gazebo cell_bringup.launch.py

Faster robot example:
  ros2 launch pick_place_cell_gazebo cell_bringup.launch.py robot_speed_scale:=0.75
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('pick_place_cell_gazebo')

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'cell_sim.launch.py')))

    robot_speed_scale = LaunchConfiguration('robot_speed_scale')

    def ctrl(exe, parameters=None):
        return Node(package='pick_place_cell_controller',
                    executable=exe, output='screen',
                    parameters=parameters or [])

    robot = ctrl(
        'gazebo_robot',
        parameters=[{
            'speed_scale': ParameterValue(robot_speed_scale, value_type=float)
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_speed_scale',
            default_value='0.60',
            description='UR5e trajectory speed scale, from 0.10 to 1.00'),
        cell,                                                     # 0s
        TimerAction(period=6.0,  actions=[ctrl('modbus_bridge')]),
        TimerAction(period=8.0,  actions=[robot]),
        TimerAction(period=9.0,  actions=[ctrl('conveyor')]),
        TimerAction(period=10.0, actions=[ctrl('grasp_manager')]),
        TimerAction(period=11.0, actions=[ctrl('label_detector')]),
        TimerAction(period=12.0, actions=[ctrl('sorter')]),
        TimerAction(period=13.0, actions=[ctrl('part_spawner')]),
    ])

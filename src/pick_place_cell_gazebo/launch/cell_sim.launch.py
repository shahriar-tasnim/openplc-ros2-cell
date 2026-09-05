import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---- your existing world -------------------------------------------
    pkg_share = get_package_share_directory('pick_place_cell_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'cell_world.sdf')

    # ---- UR5e robot description (official ur_simulation_gz) -------------
    ur_xacro = PathJoinSubstitution(
        [FindPackageShare('ur_simulation_gz'), 'urdf', 'ur_gz.urdf.xacro'])
    #ur_controllers = PathJoinSubstitution(
        #[FindPackageShare('ur_simulation_gz'), 'config', 'ur_controllers.yaml'])
    ur_controllers = PathJoinSubstitution(
        [FindPackageShare('pick_place_cell_gazebo'), 'config', 'ur5e_controllers.yaml'])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', ur_xacro,
        ' ', 'ur_type:=ur5e',
        ' ', 'name:=ur',
        ' ', 'simulation_controllers:=', ur_controllers,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content,
                                            value_type=str),
        'use_sim_time': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ---- Gazebo with your world ----------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    # ---- spawn the UR5e at the origin (reach-verified base) ------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ur',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.40',
        ],
    )

    # ---- your existing bridges (clock + set_pose) ----------------------
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/cell_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )

    # ---- controllers (UR's own names) ----------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
    )

    # start joint_trajectory_controller only AFTER the broadcaster is up
    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gazebo_bridge,
        joint_state_broadcaster_spawner,
        delay_jtc,
    ])

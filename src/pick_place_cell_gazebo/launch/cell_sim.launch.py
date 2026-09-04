import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'pick_place_cell_gazebo'
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'cell_robot.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'cell_world.sdf'
    )

    #robot_description = Command(
        #['xacro ', urdf_file]
    #)
    
    robot_description = ParameterValue(
    Command(['xacro ', urdf_file]),
    value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        parameters=[
            {
                'robot_description':
                    robot_description,

                'use_sim_time':
                    True
            }
        ],

        output='screen'
    )
    
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/cell_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(

                get_package_share_directory(
                    'ros_gz_sim'
                ),

                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'gz_args':
                '-r ' + world_file
        }.items()
    )

    spawn_robot = Node(

        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name',
            'cell_robot',

            '-topic',
            'robot_description',

            '-x',
            '0',

            '-y',
            '0',

            '-z',
            '0.05'
        ],

        output='screen'
    )

    joint_state_spawner = Node(

        package='controller_manager',
        executable='spawner',

        arguments=[
            'joint_state_broadcaster',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '120'
        ]
    )

    arm_controller_spawner = Node(

        package='controller_manager',
        executable='spawner',

        arguments=[
            'arm_controller',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '120'
        ]
    )

    return LaunchDescription([

        robot_state_publisher,

        gazebo,

        spawn_robot,
        
        gazebo_bridge,
    
        joint_state_spawner,

        arm_controller_spawner
    ])


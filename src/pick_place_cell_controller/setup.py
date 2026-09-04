from setuptools import find_packages, setup

package_name = 'pick_place_cell_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cxt13',
    maintainer_email='cxt13@example.com',
    description='OpenPLC and ROS 2 pick-and-place cell controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_robot = pick_place_cell_controller.fake_robot_node:main',
            'modbus_bridge = pick_place_cell_controller.modbus_bridge_node:main',
            'gazebo_robot = pick_place_cell_controller.gazebo_robot_node:main',
            'conveyor = pick_place_cell_controller.conveyor_node:main',
            'grasp_manager = pick_place_cell_controller.grasp_manager:main',
            'part_spawner = pick_place_cell_controller.part_spawner:main',
        ],
    },
)

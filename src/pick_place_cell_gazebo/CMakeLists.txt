cmake_minimum_required(VERSION 3.8)

project(pick_place_cell_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    config
    worlds
    launch

  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()

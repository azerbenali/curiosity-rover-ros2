cmake_minimum_required(VERSION 3.10)
project(curiosity_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config worlds rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()

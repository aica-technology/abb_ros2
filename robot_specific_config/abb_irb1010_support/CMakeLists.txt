cmake_minimum_required(VERSION 3.10.2)

project(abb_irb1010_support)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf_examples
  DESTINATION share/${PROJECT_NAME})
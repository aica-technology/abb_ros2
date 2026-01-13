cmake_minimum_required(VERSION 3.10.2)

project(abb_crb15000_12_support)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf_examples
  DESTINATION share/${PROJECT_NAME})
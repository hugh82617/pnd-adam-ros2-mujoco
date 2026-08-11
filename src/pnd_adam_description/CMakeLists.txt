cmake_minimum_required(VERSION 3.8)
project(pnd_adam_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

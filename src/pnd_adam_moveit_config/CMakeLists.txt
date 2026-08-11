cmake_minimum_required(VERSION 3.8)
project(pnd_adam_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES .setup_assistant README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(combined_fer_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()

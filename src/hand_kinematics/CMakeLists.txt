cmake_minimum_required(VERSION 3.8)
project(hand_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Debug>:-g>)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

# Find Ceres
find_package(Ceres REQUIRED)

# Find TinyXML2
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# Generate custom services
set(srv_files
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TINYXML2_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

# Add executable
add_executable(hand_kinematics_node
  src/hand_kinematics_node.cpp
)

# Add test client executable
add_executable(hand_kinematics_test_client
  src/hand_kinematics_test_client.cpp
)

# Specify libraries to link executable to
ament_target_dependencies(hand_kinematics_node
  rclcpp
  geometry_msgs
  std_msgs
  Eigen3
)

ament_target_dependencies(hand_kinematics_test_client
  rclcpp
  geometry_msgs
  std_msgs
)

# Link custom message/service dependencies
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hand_kinematics_node "${cpp_typesupport_target}")
target_link_libraries(hand_kinematics_test_client "${cpp_typesupport_target}")

# Link external libraries
target_link_libraries(hand_kinematics_node
  ${TINYXML2_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
)

target_compile_options(hand_kinematics_node PRIVATE
  $<$<CONFIG:Release>:-O3>
)

# Install executables
install(TARGETS
  hand_kinematics_node
  hand_kinematics_test_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF and other resource files (if any)
install(DIRECTORY
  srv
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
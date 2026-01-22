cmake_minimum_required(VERSION 3.5)
project(sorting_bot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# Find dependencies for the nodes
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
ament_target_dependencies(joint_trajectory_publisher rclcpp std_msgs trajectory_msgs geometry_msgs nav_msgs nav2_msgs sensor_msgs tf2_ros)

add_executable(detection_simulator src/detection_simulator.cpp)
ament_target_dependencies(detection_simulator tf2_ros)

# Generate the ROS2 parameters interface automatically.
generate_parameter_library(joint_trajectory_publisher_parameters
  src/joint_trajectory_publisher.yaml
)


add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio joint_trajectory_publisher_parameters)
target_link_libraries(joint_trajectory_publisher
  joint_trajectory_publisher_parameters
  pinocchio::pinocchio
  ${PROJECT_NAME}
)
target_include_directories(
  joint_trajectory_publisher
  PRIVATE
  include
)
target_include_directories(joint_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/joint_trajectory_publisher_parameters>
)
#target_include_directories(sorting_bot PUBLIC include)


ament_export_include_directories(include)


install(
  DIRECTORY
    include
  DESTINATION 
    share/${PROJECT_NAME}
)

# Install nodes
install(TARGETS
  joint_trajectory_publisher
  detection_simulator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY 
    launch
    include
    config
  DESTINATION share/${PROJECT_NAME})
ament_package()

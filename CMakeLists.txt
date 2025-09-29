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
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(sorting src/sorting_bot.cpp)
ament_target_dependencies(sorting rclcpp std_msgs trajectory_msgs sensor_msgs)


# set(HEADER_FILES include/sorting_bot/motion_planner.hpp)
add_library(${PROJECT_NAME} SHARED src/motion_planner.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
ament_export_libraries(${PROJECT_NAME})

target_link_libraries(sorting pinocchio::pinocchio ${PROJECT_NAME})
target_include_directories(
  sorting
  PRIVATE
  include
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
  ${PROJECT_NAME}
  sorting
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY 
    launch
    include
    config
  DESTINATION share/${PROJECT_NAME})
ament_package()

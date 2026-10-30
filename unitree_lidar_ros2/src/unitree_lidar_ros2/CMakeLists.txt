cmake_minimum_required(VERSION 3.27)
project(unitree_lidar_ros2)

# Jazzy (rclcpp) requires C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common io kdtree search filters registration range_image)
find_package(Threads REQUIRED)


add_definitions(${PCL_DEFINITIONS})

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_include_directories(unitree_lidar_ros2_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_lidar_sdk/include
)

ament_target_dependencies(unitree_lidar_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

target_link_libraries(unitree_lidar_ros2_node
  ${PCL_LIBRARIES}
  libunitree_lidar_sdk.a
  Threads::Threads
)

install(TARGETS unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# install rviz file and launch file
install(FILES rviz/view.rviz launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

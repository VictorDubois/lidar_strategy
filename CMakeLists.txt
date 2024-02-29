cmake_minimum_required(VERSION 3.5)
project(lidar_strategy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(goal_strategy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(krabi_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)


add_executable(lidar_strategy_node 
  src/main.cpp
  src/lidarStrat.cpp
  #src/Krabi/src/position.cpp
  #src/Krabi/src/positionPlusAngle.cpp
  #src/Krabi/src/vec2d.cpp
  #src/Krabi/src/vec3d.cpp
  #src/Krabi/src/angle.cpp
  #src/Krabi/src/distance.cpp
)
ament_target_dependencies(lidar_strategy_node rclcpp geometry_msgs std_msgs sensor_msgs goal_strategy tf2_ros krabi_msgs visualization_msgs)



install(TARGETS lidar_strategy_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(goal_strategy)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(krabi_msgs)
ament_export_dependencies(visualization_msgs)
ament_package()
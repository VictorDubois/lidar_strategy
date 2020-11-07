cmake_minimum_required(VERSION 2.8.3)
project(lidar_strategy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
	src/main.cpp
	src/lidarStrat.cpp
	src/Krabi/src/position.cpp
	src/Krabi/src/positionPlusAngle.cpp
	src/Krabi/src/vec2d.cpp
	src/Krabi/src/vec3d.cpp
	src/Krabi/src/angle.cpp
	src/Krabi/src/distance.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

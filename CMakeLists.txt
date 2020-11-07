cmake_minimum_required(VERSION 2.8.3)
project(lidar_strategy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs
                                        goal_strategy message_generation)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  goal_strategy)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/lidarStrat.cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

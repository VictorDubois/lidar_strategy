#include "lidarStrat.h"

void LidarStrat::create_publishers()
{
    // Most threatening obstacle in the robot's forward direction → consumed by main_strategy.
    m_obstacle_posestamped_pub
      = this->create_publisher<geometry_msgs::msg::PoseStamped>("obstacle_pose_stamped", 5);
    // Most threatening obstacle in the robot's backward direction → consumed by main_strategy.
    m_obstacle_behind_posestamped_pub
      = this->create_publisher<geometry_msgs::msg::PoseStamped>("obstacle_behind_pose_stamped", 5);
    // Full obstacle list as coloured markers for visualisation in RViz / Foxglove.
    m_obstacle_debug_pub
      = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_debug", 5);
    // LiDAR + ArUco obstacles in map frame (without static ones), throttled to 1 Hz.
    m_dynamic_pose_array_pub
      = this->create_publisher<geometry_msgs::msg::PoseArray>("dynamic_obstacles", 5);
}
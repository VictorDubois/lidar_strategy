#include "lidarStrat.h"

void LidarStrat::create_publishers()
{
    m_obstacle_posestamped_pub
      = this->create_publisher<geometry_msgs::msg::PoseStamped>("obstacle_pose_stamped", 5);
    m_obstacle_behind_posestamped_pub
      = this->create_publisher<geometry_msgs::msg::PoseStamped>("obstacle_behind_pose_stamped", 5);
    m_obstacle_debug_pub
      = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_debug", 5);
    m_dynamic_pose_array_pub
      = this->create_publisher<geometry_msgs::msg::PoseArray>("dynamic_obstacles", 5);
}
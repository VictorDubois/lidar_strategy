#include "lidarStrat.h"

void LidarStrat::create_subscribers()
{
    m_lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_obstacles", 1000, std::bind(&LidarStrat::updateLidarScan, this, std::placeholders::_1));
    m_aruco_obstacles_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "aruco_obstacles",
      5,
      std::bind(&LidarStrat::updateArucoObstacles, this, std::placeholders::_1));
    m_remaining_time_sub = this->create_subscription<builtin_interfaces::msg::Duration>(
      "remaining_time",
      1000,
      std::bind(&LidarStrat::updateRemainingTime, this, std::placeholders::_1));

    // Individual ArUco tag subscribers — used with the overhead camera from 2020/2022.
    // Tags 1-5 belong to the blue team's opponents (yellow); tags 6-10 to yellow's opponents (blue).
    // Disabled by default; compile with -DUSE_ARUCO to re-enable.
#ifdef USE_ARUCO

    if (m_is_blue)
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_6_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 6);
        m_arucos_sub[6] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r6", 5, l_arucos_6_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_7_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 7);
        m_arucos_sub[7] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r7", 5, l_arucos_7_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_8_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 8);
        m_arucos_sub[8] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r8", 5, l_arucos_8_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_9_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 9);
        m_arucos_sub[9] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r9", 5, l_arucos_9_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_10_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 10);
        m_arucos_sub[10] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r10", 5, l_arucos_10_func); //, l_sub_options);
    }
    else
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_1_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 1);
        m_arucos_sub[1] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r1", 5, l_arucos_1_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_2_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 2);
        m_arucos_sub[2] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r2", 5, l_arucos_2_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_3_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 3);
        m_arucos_sub[3] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r3", 5, l_arucos_3_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_4_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 4);
        m_arucos_sub[4] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r4", 5, l_arucos_4_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_5_func
          = std::bind(&LidarStrat::updateAruco, this, std::placeholders::_1, 5);
        m_arucos_sub[5] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r5", 5, l_arucos_5_func); //, l_sub_options);
    }
#endif // USE_ARUCO
}
#pragma once

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <map>
#include <ros/ros.h>
#include <vector>

#include "krabilib/position.h"
#include "krabilib/positionPlusAngle.h"

class LidarStrat
{

public:
    LidarStrat(int argc, char* argv[]);
    LidarStrat();
    void run();
    static float speed_inhibition(float distance, float angle, float distanceCoeff);

private:
    typedef std::pair<float, float> PolarPosition; // distance, angle

    void sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance, bool reverseGear);
    void updateLidarScan(const sensor_msgs::LaserScan& new_scan);
    void updateLidarScanSimu(const sensor_msgs::LaserScan& new_scan);
    void static closest_point_of_segment(const Position& segment1,
                                         const Position& segment2,
                                         Position& closestPoint);
    void static closest_point_of_segment(const float x,
                                         const float y,
                                         const float x1,
                                         const float y1,
                                         const float x2,
                                         const float y2,
                                         float& xx,
                                         float& yy);
    size_t computeMostThreatening(const std::vector<PolarPosition>& points,
                                  float distanceCoeff,
                                  bool reverseGear);

    void updateCurrentPose(const geometry_msgs::Pose& newPose);
    void updateArucoObstacles(const geometry_msgs::PoseArray& newPoses);
    void updateAruco(const boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id);
    bool static isInsideTable(const Position& input);
    Position toAbsolute(const Position& input);

    bool m_is_blue;
    float m_min_distance;
    float m_max_distance;
    float m_min_intensity;
    int m_nb_measures_lidar;

    std::vector<float> m_raw_sensors_dists;  // in m
    std::vector<int> m_lidar_sensors_angles; // in deg
    std::vector<PolarPosition> m_aruco_obstacles;
    PositionPlusAngle m_currentPose;
    sensor_msgs::LaserScan m_obstacle_dbg; // used to display debug stuff
    std::array<geometry_msgs::PoseStamped, 10> m_arucos;

    ros::Publisher m_obstacle_danger_debuger;
    ros::Publisher m_obstacle_posestamped_pub;
    ros::Publisher m_obstacle_behind_posestamped_pub;

    ros::Subscriber m_lidar_sub;
    ros::Subscriber m_lidar_simu_sub;
    ros::Subscriber m_current_pose_sub;
    ros::Subscriber m_aruco_obstacles_sub;
    std::map<int, ros::Subscriber> m_arucos_sub;
};
#ifndef _LIDAR_H_
#define _LIDAR_H_

#define NB_MEASURES_LIDAR 360

#define TRUE 1
#define FALSE 0
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/ros.h>
#include <vector>

#include "Krabi/position.h"
#include "Krabi/positionPlusAngle.h"

class LidarStrat
{
public:
    LidarStrat(int argc, char* argv[]);
    LidarStrat();
    ~LidarStrat();
    void run();
    static float speed_inhibition(float distance, float angle, float distanceCoeff);

private:
    std::vector<float> raw_sensors_dists;  // in m
    std::vector<int> lidar_sensors_angles; // in deg
    sensor_msgs::LaserScan obstacle_dbg;   // used to display debug stuff
    void sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance);
    void updateLidarScan(sensor_msgs::LaserScan new_scan);
    void ClosestPointOfSegment(const Position& segment1,
                               const Position& segment2,
                               Position& closestPoint);
    void static ClosestPointOfSegment(const float x,
                                      const float y,
                                      const float x1,
                                      const float y1,
                                      const float x2,
                                      const float y2,
                                      float& xx,
                                      float& yy);
    size_t computeMostThreatening(const std::vector<std::pair<float, float>> points,
                                  float distanceCoeff);

    void updateCurrentPose(geometry_msgs::Pose newPose);

    ros::Publisher obstacle_pose_pub;
    ros::Publisher obstacle_danger_debuger;
    ros::Publisher obstacle_posestamped_pub;
    ros::Subscriber lidar_sub;
    ros::Subscriber current_pose_sub;
    PositionPlusAngle currentPose;
};

#endif

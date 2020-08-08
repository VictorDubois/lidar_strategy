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

class LidarStrat
{
public:
    LidarStrat(int argc, char* argv[]);
    ~LidarStrat();
    void run();

private:
    float speed_inhibition(float distance, float angle, float distanceCoeff);
    std::vector<float> raw_sensors_dists;  // in m
    std::vector<int> lidar_sensors_angles; // in deg
    sensor_msgs::LaserScan obstacle_dbg;   // used to display debug stuff
    void sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance);
    void updateLidarScan(sensor_msgs::LaserScan new_scan);

    ros::Publisher obstacle_pose_pub;
    ros::Publisher obstacle_danger_debuger;
    ros::Publisher obstacle_posestamped_pub;
    ros::Subscriber lidar_sub;
};

#endif

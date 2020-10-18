#ifndef _LIDAR_H_
#define _LIDAR_H_

#define NB_MEASURES_LIDAR 360

#define TRUE 1
#define FALSE 0
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
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
    typedef std::pair<float, float> PolarPosition; // distance, angle
    std::vector<float> raw_sensors_dists;          // in m
    std::vector<int> lidar_sensors_angles;         // in deg
    std::vector<PolarPosition> aruco_obstacles;
    PositionPlusAngle currentPose;
    sensor_msgs::LaserScan obstacle_dbg; // used to display debug stuff
    std::array<geometry_msgs::PoseStamped, 10> arucos;

    void sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance, bool reverseGear);
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
    size_t computeMostThreatening(const std::vector<PolarPosition> points,
                                  float distanceCoeff,
                                  bool reverseGear);

    void updateCurrentPose(geometry_msgs::Pose newPose);
    void updateArucoObstacles(geometry_msgs::PoseArray newPoses);
    void updateAruco1(geometry_msgs::PoseStamped arucoPose);
    void updateAruco2(geometry_msgs::PoseStamped arucoPose);
    void updateAruco3(geometry_msgs::PoseStamped arucoPose);
    void updateAruco4(geometry_msgs::PoseStamped arucoPose);
    void updateAruco5(geometry_msgs::PoseStamped arucoPose);
    void updateAruco6(geometry_msgs::PoseStamped arucoPose);
    void updateAruco7(geometry_msgs::PoseStamped arucoPose);
    void updateAruco8(geometry_msgs::PoseStamped arucoPose);
    void updateAruco9(geometry_msgs::PoseStamped arucoPose);
    void updateAruco10(geometry_msgs::PoseStamped arucoPose);
    bool isInsideTable(Position input);
    Position toAbsolute(Position input);
    bool is_blue;

    ros::Publisher obstacle_danger_debuger;
    ros::Publisher obstacle_posestamped_pub;
    ros::Publisher obstacle_behind_posestamped_pub;
    ros::Subscriber lidar_sub;
    ros::Subscriber current_pose_sub;
    ros::Subscriber aruco_obstacles_sub;
    ros::Subscriber aruco_1_sub;
    ros::Subscriber aruco_2_sub;
    ros::Subscriber aruco_3_sub;
    ros::Subscriber aruco_4_sub;
    ros::Subscriber aruco_5_sub;
    ros::Subscriber aruco_6_sub;
    ros::Subscriber aruco_7_sub;
    ros::Subscriber aruco_8_sub;
    ros::Subscriber aruco_9_sub;
    ros::Subscriber aruco_10_sub;
};

#endif

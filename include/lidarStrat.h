#ifndef _LIDAR_H_
#define _LIDAR_H_

#define NB_MEASURES_LIDAR 360

#define TRUE 1
#define FALSE 0
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
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
    sensor_msgs::msg::LaserScan obstacle_dbg; // used to display debug stuff

    void sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance, bool reverseGear);
    void updateLidarScan(sensor_msgs::msg::LaserScan new_scan);
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

    void updateCurrentPose(geometry_msgs::msg::Pose newPose);
    void updateArucoObstacles(geometry_msgs::msg::PoseArray newPoses);
    bool isInsideTable(Position input);
    Position toAbsolute(Position input);

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_danger_debuger;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_posestamped_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_behind_posestamped_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_Absolute_posestamped_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_obstacles_sub;
    rclcpp::Node::SharedPtr node;
};

#endif

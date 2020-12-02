#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "krabilib/pose.h"
#include "krabilib/position.h"

class LidarStrat
{

public:
    LidarStrat(ros::NodeHandle& nh);
    LidarStrat();
    void run();
    static float speed_inhibition(Distance distance, Angle angle, float distanceCoeff);

private:
    void sendObstaclePose(PolarPosition pp, bool reverseGear);
    void updateLidarScan(const sensor_msgs::LaserScan& new_scan);
    void static closest_point_of_segment(const Position& point,
                                         const Position& segment1,
                                         const Position& segment2,
                                         Position& closestPoint);
    void static closest_point_of_segment(const Distance x,
                                         const Distance y,
                                         const Distance x1,
                                         const Distance y1,
                                         const Distance x2,
                                         const Distance y2,
                                         Distance& xx,
                                         Distance& yy);
    int computeMostThreatening(const std::vector<PolarPosition>& points,
                                  float distanceCoeff,
                                  bool look_in_front);

    void updateCurrentPose();
    void updateArucoObstacles(const geometry_msgs::PoseArray& newPoses);
    void updateAruco(const boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id);
    bool static isInsideTable(const Position& input);
    Angle idToAngle(unsigned int id);
    unsigned int angleToId(Angle a);

    bool m_is_blue;
    Distance m_min_distance;
    Distance m_max_distance;
    float m_min_intensity;
    int m_nb_angular_steps;
    Distance m_aruco_obs_offset;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
    Transform3D m_laser_to_map;
    Transform3D m_baselink_to_map;
    Transform3D m_map_to_baselink;
    Pose m_current_pose;

    std::vector<Distance> m_lidar_sensors_dists; // in m
    std::vector<Angle> m_lidar_sensors_angles; // in rad
    std::vector<PolarPosition> m_aruco_obstacles;
    sensor_msgs::LaserScan m_obstacle_dbg; // used to display debug stuff
    std::array<geometry_msgs::PoseStamped, 10> m_arucos;

    ros::Publisher m_obstacle_danger_debuger;
    ros::Publisher m_obstacle_posestamped_pub;
    ros::Publisher m_obstacle_behind_posestamped_pub;
    ros::Publisher m_obstacle_absolute_posestamped_pub;

    ros::Subscriber m_lidar_sub;
    ros::Subscriber m_current_pose_sub;
    ros::Subscriber m_aruco_obstacles_sub;
    std::map<int, ros::Subscriber> m_arucos_sub;
};
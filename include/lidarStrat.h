#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <visualization_msgs/msg/marker_array.hpp>


#include <vector>



#include "krabilib/pose.h"
#include "krabilib/position.h"

class LidarStrat : public rclcpp::Node
{

public:
    LidarStrat();
    void run();
    //static float speed_inhibition(Distance distance, Angle angle, float distanceCoeff);

    /**
     * @brief speed_inhibition the speed inhibition caused by an obstacle. For more information see the
     * geogebra file
     * @param distance the distance of the obstacle
     * @param angle the angle of the obstacle
     * @param distanceCoeff
     * @return the speed inhibition coefficient (1 = full speed, 0 = stop)
     */
    static float speed_inhibition(Distance distance, Angle angle, float distanceCoeff)
    {
        float slope_max = 0.3f;  // m. max slope of the braking distance modulation
        float slope_min = 0.1f;  // m. min slope of the braking distance modulation
        float slope = slope_max; // m. slope of the braking distance modulation
        float half_stop_min
        = 0.45f * distanceCoeff; // m. min distance at which we limit to half the full speed
        float half_stop_max
        = 0.55f * distanceCoeff;       // m. max distance at which we limit to half the full speed
        float half_stop = half_stop_max; // m distance at which we limit to half the full speed
        // We modulate the intensity of the inhibition based on the angle at which the obstacle is
        // seen: in front it is more dangerous than on the sides
        float angle_factor = compute_dangerousness_from_angle(angle);
        half_stop = half_stop_min + (half_stop_max - half_stop_min) * angle_factor;
        slope = slope_min + (slope_max - slope_min) * angle_factor;

        return 0.5 * (1 + tanh((distance - half_stop) / slope));
    };

private:
    static float compute_dangerousness_from_angle(Angle a)
    {
        if (a == 0.f)
        {
            return 1;
        }
        if (abs(a) > M_PI / 2)
        {
            return 0;
        }
        return sin(a) / a;
    };
    void sendObstaclePose(PolarPosition pp, bool reverseGear);
    void updateLidarScan(const sensor_msgs::msg::LaserScan& new_scan);

    
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

    void sendDynamicObstacles(std::vector<PolarPosition> obstacles);

    void updateCurrentPose();
    void updateArucoObstacles(const geometry_msgs::msg::PoseArray& newPoses);
    void updateAruco(const std::shared_ptr<geometry_msgs::msg::PoseStamped const> arucoPose, int id);
    bool static isInsideTable(const Position& input);
    Angle idToAngle(unsigned int id);
    unsigned int angleToId(Angle a);

    bool m_is_blue;
    Distance m_min_distance;
    Distance m_max_distance;
    float m_min_intensity;
    int m_nb_angular_steps;
    Distance m_lidar_obs_offset;
    Distance m_aruco_obs_offset;
    Distance m_border_obs_offset;
    Distance m_fixes_obs_offset;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    
    Transform3D m_laser_to_map;
    Transform3D m_laser_to_map_at_last_lidar_scan;
    Transform3D m_baselink_to_map;
    Transform3D m_map_to_baselink;
    Pose m_current_pose;

    std::vector<Distance> m_lidar_sensors_dists; // in m
    std::vector<Angle> m_lidar_sensors_angles;   // in rad
    std::vector<PolarPosition> m_aruco_obstacles;
    sensor_msgs::msg::LaserScan m_obstacle_dbg; // used to display debug stuff
    std::array<geometry_msgs::msg::PoseStamped, 10> m_arucos;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_obstacle_posestamped_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_obstacle_behind_posestamped_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_obstacle_debug_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_dynamic_pose_array_pub;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_current_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_aruco_obstacles_sub;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> m_arucos_sub;

    rclcpp::Time m_timeout_next_publish_dynamic_obst;

    rclcpp::Node::SharedPtr node;
    rclcpp::TimerBase::SharedPtr timer_;
};

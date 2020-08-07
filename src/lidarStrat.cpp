#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/ros.h>

#include "../include/lidarStrat.h"

#define NB_NEURONS 360
//#define NB_MEASURES_LIDAR 360

#define CRITICAL_DISTANCE 0.100      // in m
#define CRITCIAL_DISTANCE_TRIG 0.120 // in m
#define LIDAR_DIST_OFFSET                                                                          \
    0.080 // in m, the lidar is much more deep inside the robot, vs the US/IR near the outside
#define MAX_DISTANCE 6.0 // in m
#define MIN_DISTANCE 0.1 // in m
#define MIN_INTENSITY 10

// The smoothing factor for the obstacles' intensity
#define ALPHA 1

std::vector<float> raw_sensors_dists(NB_NEURONS, 0);

sensor_msgs::LaserScan obstacle_dbg;

void updateLidarScan(sensor_msgs::LaserScan new_scan)
{
    obstacle_dbg = new_scan;
    for (int i = 0; i < NB_MEASURES_LIDAR; i++)
    {
        if (new_scan.intensities[i] < MIN_INTENSITY || new_scan.ranges[i] < MIN_DISTANCE)
        {
            // Unreliable, do not take into account
            raw_sensors_dists[i] = MAX_DISTANCE;
        }
        else
        {
            raw_sensors_dists[i] = new_scan.ranges[i];
        }
    }
}

unsigned int get_idx_of_max(const float vector[], const size_t len)
{
    unsigned int curr_max = 0, i;

    for (i = 1; i < len; i += 1)
    {
        if (vector[i] > vector[curr_max])
            curr_max = i;
    }
    return curr_max;
}

float sin_card(float x)
{
    if (x == 0.f)
    {
        return 1;
    }
    if (x < -90 || x > 90)
    {
        return 0;
    }
    // x =2.0* M_PI * x / 180;
    x = M_PI * x / 180;
    return sin(x) / x;
}

/**
 * Convert a polar position to a cartesian one
 * @param posX the X position, in m
 * @param posY the Y position, in m
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
void polar_to_cart(float& posX, float& posY, const float theta, const float distance)
{
    posX = distance * cos(theta * M_PI / 180.f);
    posY = distance * sin(theta * M_PI / 180.f);
}

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in m
 * @param posY the Y position, in m
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void cart_to_polar(const float posX, const float posY, float& theta, float& distance)
{
    theta = (180. / M_PI) * atan2(posX, posY);
    distance = sqrt(posX * posX + posY * posY);

    // fix angular ambiguity
    if (posY < 0)
    {
        theta += 180;
    }

#ifdef DEBUG_cart_to_polar
    std::cout << "posX = " << posX << "posY = " << posY << "theta = " << theta
              << ", distance = " << distance << std::endl;
    float posXafter = distance * cos(theta * M_PI / 180.f);
    float posYafter = distance * sin(theta * M_PI / 180.f);
    std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
#endif
}

float speed_inhibition(float distance, float angle, float distanceCoeff)
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
    // seen
    float angle_factor = sin_card((angle)-180.0f);
    half_stop = half_stop_min + (half_stop_max - half_stop_min) * angle_factor;
    slope = slope_min + (slope_max - slope_min) * angle_factor;

    //@TODO replace this in main strat
    return (50.f * tanh((distance - half_stop) / slope) + 49.f) / 100.f;
}

// Entry point
int main(int argc, char* argv[])
{
    printf("[LIDAR] Begin main\n");
    fflush(stdout);

    ros::init(argc, argv, "lidarStrat");
    ros::start();

    float obstacle_distance = 2000; // dist of the nearest obstacle, in m
    unsigned int nearest_obstacle_angle;
    std::vector<int> lidar_sensors_angles; // in deg
    for (int i = 0; i < NB_NEURONS; i++)
    {
        lidar_sensors_angles.push_back((i + 180) % 360); // conversion from loop index to degrees
        obstacle_dbg.intensities.push_back(0);
        obstacle_dbg.ranges.push_back(0);
    }

    float distanceCoeff = 1;
    std::vector<float> sensors_dists(NB_NEURONS, 0);
    ros::Rate loop_rate(5);

    // Initialize nearest's obstacle in front of us
    nearest_obstacle_angle = 0;
    std::vector<float> pre_output(NB_NEURONS, 0);

    ros::NodeHandle n;
    ros::Publisher obstacle_pose_pub = n.advertise<geometry_msgs::Pose>("obstacle_pose", 1000);
    ros::Publisher obstacle_danger_debuger = n.advertise<sensor_msgs::LaserScan>("obstacle_dbg", 5);
    ros::Publisher obstacle_posestamped_pub
      = n.advertise<geometry_msgs::PoseStamped>("obstacle_pose_stamped", 1000);
    ros::Subscriber lidar_sub = n.subscribe("scan", 1000, &updateLidarScan);

    /*************************************************
     *                   Main loop                   *
     *************************************************/
    while (ros::ok())
    {
        // Modulate the stop/brake distance from other strats' inputs

        /*if (distanceCoeff != oldDistanceCoeff) {
                            std::cout << "Stop distance coeff read = " << distanceCoeff <<
           std::endl; std::cout << "brake_max = " << brake_max << ", brake_min = " << brake_min <<
           ", brake = " << brake << ", stop_min = " << stop_min << "stop_max = " << stop_max << ",
           stop = " << stop << std::endl; fflush(stdout);
                    }*/

        for (int i = 0; i < NB_MEASURES_LIDAR; i += 1)
        {
            sensors_dists[i] = raw_sensors_dists[i];

            // Smooth output
            /*else if (raw_sensors_dists[i] > sensors_dists[i]) {
              sensors_dists[i] = (1. - ALPHA) * sensors_dists[i] + ALPHA * raw_sensors_dists[i];
                              }
                              else {
              sensors_dists[i] = (1. - ALPHA) * sensors_dists[i] + ALPHA * raw_sensors_dists[i];
            }*/

            // std::cout << "Smoothed uS sensors n°" << i << ": " << sensors_dists[i] << std::endl;
        }

        /*
         * Reactive approach to obstacles:
         * - Simplified: obstacles on the right, left or both, output on the neural field
         * - Linear speed is linearly modulated by the distance of the obstacle and it comes
         *   to a stop if too close
         */

        // Linear modulation of the linear speed
        // 0) Reset obstacle_distance & position

        obstacle_distance = 6.0;

        // 1) Find the distance to the nearest obstacle and its relative position compared to the
        // robot
        float currentMostThreatening = std::numeric_limits<float>::infinity();

        for (int i = 0; i < NB_MEASURES_LIDAR; i += 1)
        {
            obstacle_dbg.intensities[i] = 1;

            // Disable detection from behind
            if (lidar_sensors_angles[i] < 120 || lidar_sensors_angles[i] > 240)
            {
                continue;
            }

            float danger
              = speed_inhibition(sensors_dists[i], lidar_sensors_angles[i], distanceCoeff);

            obstacle_dbg.intensities[i] = danger;
            // obstacle_dbg.ranges[i] = sin_card((lidar_sensors_angles[i]) - 180.0f);// display
            // angle factor

            if (currentMostThreatening > danger)
            {
                /*std::cout << "distance = " << sensors_dists[i] << "\tangle = " << i
                          << "\tdanger=" << danger << std::endl;*/
                obstacle_distance = sensors_dists[i];
                currentMostThreatening = danger;
                nearest_obstacle_angle = i; // lidar_sensors_angles[i]; ?
            }
        }

        /*
         * 2) Compute linear speed
         * TODO: modulate the speed by the angle of the us sensors!
         */

        std::cout << "nearest_obstacle_angle = " << nearest_obstacle_angle << ", at "
                  << obstacle_distance << " m " << std::endl;

        geometry_msgs::Pose obstacle_pose;
        float posX;
        float posY;
        polar_to_cart(posX, posY, nearest_obstacle_angle, obstacle_distance);
        obstacle_pose.position.x = posX;
        obstacle_pose.position.y = posY;
        obstacle_pose_pub.publish(obstacle_pose);
        geometry_msgs::PoseStamped obstacle_pose_stamped;
        obstacle_pose_stamped.pose = obstacle_pose;
        obstacle_pose_stamped.header.frame_id
          = "base_link"; // "neato_laser"; // base_link for robot, neato_laser for logs/debug
        obstacle_posestamped_pub.publish(obstacle_pose_stamped);
        std::cout << "cart X = " << obstacle_pose.position.x << ", Y = " << obstacle_pose.position.y
                  << std::endl;

        obstacle_danger_debuger.publish(obstacle_dbg);

        // printf("Distance: %d cm @ %d deg-> Linear Speed Inhibition: %d\n",
        // obstacle_distance, nearest_obstacle_angle, strategy.output->speed_inhibition);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Print a message to let the user know the script exited cleanly
    printf("Obstacles script exited cleanly\n");

    return 0;
}

#undef NB_NEURONS
#undef ALPHA

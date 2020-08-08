

#include "../include/lidarStrat.h"

//#define NB_MEASURES_LIDAR 360

#define CRITICAL_DISTANCE 0.100f      // in m
#define CRITCIAL_DISTANCE_TRIG 0.120f // in m
#define LIDAR_DIST_OFFSET                                                                          \
    0.080f // in m, the lidar is much more deep inside the robot, vs the US/IR near the outside
#define MAX_DISTANCE 6.0f // in m
#define MIN_DISTANCE 0.1f // in m
#define MIN_INTENSITY 10

// The smoothing factor for the obstacles' intensity
#define ALPHA 1

void LidarStrat::updateLidarScan(sensor_msgs::LaserScan new_scan)
{
    obstacle_dbg = new_scan;
    for (size_t i = 0; i < NB_MEASURES_LIDAR; i++)
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
    x = static_cast<float>(M_PI) * x / 180;
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
    posX = distance * cosf(theta * static_cast<float>(M_PI) / 180.f);
    posY = distance * sinf(theta * static_cast<float>(M_PI) / 180.f);
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
    theta = (180.f / static_cast<float>(M_PI)) * atan2f(posX, posY);
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

/**
 * @brief speed_inhibition the speed inhibition caused by an obstacle. For more information see the
 * geogebra file
 * @param distance the distance of the obstacle
 * @param angle the angle of the obstacle
 * @param distanceCoeff
 * @return the speed inhibition coefficient (1 = full speed, 0 = stop)
 */
float LidarStrat::speed_inhibition(float distance, float angle, float distanceCoeff)
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
    float angle_factor = sin_card((angle)-180.0f);
    half_stop = half_stop_min + (half_stop_max - half_stop_min) * angle_factor;
    slope = slope_min + (slope_max - slope_min) * angle_factor;

    return (50.f * tanh((distance - half_stop) / slope) + 49.f) / 100.f;
}

void LidarStrat::sendObstaclePose(float nearest_obstacle_angle, float obstacle_distance)
{
    geometry_msgs::Pose obstacle_pose;
    float posX;
    float posY;
    polar_to_cart(posX, posY, nearest_obstacle_angle, obstacle_distance);
    obstacle_pose.position.x = static_cast<double>(posX);
    obstacle_pose.position.y = static_cast<double>(posY);
    obstacle_pose_pub.publish(obstacle_pose);
    geometry_msgs::PoseStamped obstacle_pose_stamped;
    obstacle_pose_stamped.pose = obstacle_pose;
    obstacle_pose_stamped.header.frame_id
      = "neato_laser"; // base_link for robot, neato_laser for logs/debug
    obstacle_posestamped_pub.publish(obstacle_pose_stamped);
    // std::cout << "cart X = " << obstacle_pose.position.x << ", Y = " <<
    // obstacle_pose.position.y << std::endl;
}

int main(int argc, char* argv[])
{
    LidarStrat* my_lidar_strat = new LidarStrat(argc, argv);

    my_lidar_strat->run();

    return 0;
}

LidarStrat::LidarStrat(int argc, char* argv[])
{
    printf("[LIDAR] Begin main\n");
    fflush(stdout);

    ros::init(argc, argv, "lidarStrat");
    ros::start();

    for (int i = 0; i < NB_MEASURES_LIDAR; i++)
    {
        raw_sensors_dists.push_back(0);
        lidar_sensors_angles.push_back((i + 180) % 360); // conversion from loop index to degrees
        obstacle_dbg.intensities.push_back(0);
        obstacle_dbg.ranges.push_back(0);
    }

    ros::NodeHandle n;
    obstacle_pose_pub = n.advertise<geometry_msgs::Pose>("obstacle_pose", 1000);
    obstacle_danger_debuger = n.advertise<sensor_msgs::LaserScan>("obstacle_dbg", 5);
    obstacle_posestamped_pub
      = n.advertise<geometry_msgs::PoseStamped>("obstacle_pose_stamped", 1000);
    lidar_sub = n.subscribe("scan", 1000, &LidarStrat::updateLidarScan, this);
}

void LidarStrat::run()
{
    float distanceCoeff = 1;
    ros::Rate loop_rate(5);
    std::vector<float> sensors_dists(NB_MEASURES_LIDAR, 0);

    /*************************************************
     *                   Main loop                   *
     *************************************************/
    while (ros::ok())
    {

        for (size_t i = 0; i < NB_MEASURES_LIDAR; i += 1)
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

        // Modulation of the linear speed
        // 0) Reset obstacle_distance & position

        float obstacle_distance = std::numeric_limits<float>::infinity();
        float nearest_obstacle_angle = 0;

        // 1) Find the distance to the most threatening obstacle and its relative position compared
        // to the robot
        float currentMostThreatening = std::numeric_limits<float>::infinity();

        for (size_t i = 0; i < NB_MEASURES_LIDAR; i += 1)
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

            // display angle factor for debug
            // obstacle_dbg.ranges[i] = sin_card((lidar_sensors_angles[i]) - 180.0f);

            if (currentMostThreatening > danger)
            {
                /*std::cout << "distance = " << sensors_dists[i] << "\tangle = " << i
                          << "\tdanger=" << danger << std::endl;*/
                obstacle_distance = sensors_dists[i];
                currentMostThreatening = danger;
                nearest_obstacle_angle = static_cast<float>(i); // lidar_sensors_angles[i]; ?
            }
        }

        std::cout << "nearest_obstacle_angle = " << nearest_obstacle_angle << ", at "
                  << obstacle_distance << " m " << std::endl;

        sendObstaclePose(nearest_obstacle_angle, obstacle_distance);

        obstacle_danger_debuger.publish(obstacle_dbg);

        // printf("Distance: %d cm @ %d deg-> Linear Speed Inhibition: %d\n",
        // obstacle_distance, nearest_obstacle_angle, strategy.output->speed_inhibition);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Print a message to let the user know the script exited cleanly
    printf("Obstacles script exited cleanly\n");
}

//#undef NB_NEURONS
#undef ALPHA

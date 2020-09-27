#include <utility>

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

#ifndef MAX
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

void LidarStrat::updateCurrentPose(geometry_msgs::Pose newPose)
{
    currentPose = PositionPlusAngle(newPose);
    std::cout << "updateCurrentPose: x = " << currentPose.getPosition().getX()
              << ", y = " << currentPose.getPosition().getY() << std::endl;
}

void LidarStrat::updateLidarScan(sensor_msgs::LaserScan new_scan)
{
    // obstacle_dbg = new_scan;
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

void LidarStrat::updateArucoObstacles(geometry_msgs::PoseArray newPoses)
{
    aruco_obstacles.clear();
    for (auto pose : newPoses.poses)
    {
        float theta, distance;
        cart_to_polar(currentPose.getPosition().getX() - static_cast<float>(pose.position.x),
                      currentPose.getPosition().getY() - static_cast<float>(pose.position.y),
                      theta,
                      distance);

        // the center of the aruco is probably farther than the edge of the robot
        distance = MAX(0, distance - 0.2f);

        aruco_obstacles.push_back(
          std::make_pair<float, float>(static_cast<float>(distance), static_cast<float>(theta)));

        std::cout << "arucoObstacle: distance = " << distance << ", theta = " << theta << std::endl;
    }
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
    geometry_msgs::PoseStamped obstacle_pose_stamped;
    obstacle_pose_stamped.pose = obstacle_pose;
    obstacle_pose_stamped.header.frame_id
      = "base_link"; // base_link for robot, neato_laser for logs/debug
    obstacle_posestamped_pub.publish(obstacle_pose_stamped);
    // std::cout << "cart X = " << obstacle_pose.position.x << ", Y = " <<
    // obstacle_pose.position.y << std::endl;
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
    aruco_obstacles.clear();

    ros::NodeHandle n;
    obstacle_danger_debuger = n.advertise<sensor_msgs::LaserScan>("obstacle_dbg", 5);
    obstacle_posestamped_pub
      = n.advertise<geometry_msgs::PoseStamped>("obstacle_pose_stamped", 1000);
    lidar_sub = n.subscribe("scan", 1000, &LidarStrat::updateLidarScan, this);
    current_pose_sub = n.subscribe("current_pose", 5, &LidarStrat::updateCurrentPose, this);
    aruco_obstacles_sub
      = n.subscribe("aruco_obstacles", 5, &LidarStrat::updateArucoObstacles, this);
}

void LidarStrat::ClosestPointOfSegment(const Position& segment1,
                                       const Position& segment2,
                                       Position& closestPoint)
{
    float xx, yy;
    ClosestPointOfSegment(0, // currentPose.getPosition().getX(),
                          0, // currentPose.getPosition().getY(),
                          segment1.getX(),
                          segment1.getY(),
                          segment2.getX(),
                          segment2.getY(),
                          xx,
                          yy);
    closestPoint.setX(xx);
    closestPoint.setY(yy);
}

// Thanks to https://stackoverflow.com/a/6853926/10680963
void LidarStrat::ClosestPointOfSegment(const float x,
                                       const float y,
                                       const float x1,
                                       const float y1,
                                       const float x2,
                                       const float y2,
                                       float& xx,
                                       float& yy)
{
    float A = x - x1;
    float B = y - y1;
    float C = x2 - x1;
    float D = y2 - y1;

    float dot = A * C + B * D;
    float len_sq = C * C + D * D;
    float param = -1;
    if (len_sq != 0.f) // in case of 0 length line
    {
        param = dot / len_sq;
    }

    if (param < 0)
    {
        xx = x1;
        yy = y1;
    }
    else if (param > 1)
    {
        xx = x2;
        yy = y2;
    }
    else
    {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    std::cout << "xx = " << xx << ", yy = " << yy << std::endl;
}

size_t LidarStrat::computeMostThreatening(const std::vector<PolarPosition> points,
                                          float distanceCoeff)
{
    size_t currentMostThreateningId = 0;

    // 1) Find the distance to the most threatening obstacle and its relative position compared
    // to the robot
    float currentMostThreatening = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < points.size(); i++)
    {

        float danger = speed_inhibition(points[i].first, points[i].second, distanceCoeff);

        // obstacle_dbg.intensities[i] = danger;

        // display angle factor for debug

        if (currentMostThreatening > danger)
        {
            // std::cout << "distance = " << points[i].first << "\tangle = " <<
            // points[i].second << "\tdanger=" << danger << std::endl;
            currentMostThreatening = danger;
            currentMostThreateningId = i;
        }
    }
    return currentMostThreateningId;
}

void LidarStrat::run()
{
    float distanceCoeff = 1;
    ros::Rate loop_rate(5);
    std::vector<float> sensors_dists;

    /*************************************************
     *                   Main loop                   *
     *************************************************/
    while (ros::ok())
    {
        std::vector<PolarPosition> obstacles;

        for (size_t i = 0; i < NB_MEASURES_LIDAR; i += 1)
        {
            // obstacle_dbg.intensities[i]
            //  = speed_inhibition(raw_sensors_dists[i], lidar_sensors_angles[i], 1);
            // obstacle_dbg.ranges[i] = sin_card((lidar_sensors_angles[i]) - 180.0f);

            // Disable detection from behind
            if (lidar_sensors_angles[i] < 120 || lidar_sensors_angles[i] > 240)
            {
                continue;
            }

            obstacles.push_back(std::make_pair<float, float>(
              static_cast<float>(raw_sensors_dists[i]), lidar_sensors_angles[i]));
        }

        std::vector<std::pair<Position, Position>> maps_segments;
        maps_segments.push_back(std::make_pair(Position(0, 0), Position(3000, 0)));
        maps_segments.push_back(std::make_pair(Position(0, 2000), Position(3000, 2000)));
        maps_segments.push_back(std::make_pair(Position(0, 0), Position(0, 2000)));
        maps_segments.push_back(std::make_pair(Position(3000, 0), Position(3000, 2000)));

        for (auto segment : maps_segments)
        {
            Position closestPoint;
            ClosestPointOfSegment(currentPose.getPosition() - segment.first,
                                  currentPose.getPosition() - segment.second,
                                  closestPoint);

            float theta, distance;
            cart_to_polar(closestPoint.getX() / 1000, closestPoint.getY() / 1000, theta, distance);

            /*obstacles.push_back(std::make_pair<float, float>(
              static_cast<float>(distance), static_cast<float>(theta - currentPose.getAngle())));*/
            // @Todo check sign of angle
        }

        //obstacles.insert(obstacles.end(), aruco_obstacles.begin(), aruco_obstacles.end());

        size_t most_threateningId = computeMostThreatening(obstacles, distanceCoeff);

        float obstacle_distance = obstacles[most_threateningId].first;
        float nearest_obstacle_angle = obstacles[most_threateningId].second;

        std::cout << "nearest_obstacle_angle = " << nearest_obstacle_angle << ", at "
                  << obstacle_distance << " m " << std::endl;

        sendObstaclePose(180-nearest_obstacle_angle, obstacle_distance);
        // @Todo check transformation: "+180" might be just needed because we use "neato_lidar" as
        // frame

        obstacle_danger_debuger.publish(obstacle_dbg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Print a message to let the user know the script exited cleanly
    printf("Obstacles script exited cleanly\n");
}

//#undef NB_NEURONS
#undef ALPHA

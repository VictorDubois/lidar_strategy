#include "lidarStrat.h"
#include <tf/transform_listener.h>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

void LidarStrat::updateCurrentPose()
{
    try
    {
        auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
        auto laser_id = tf::resolve(ros::this_node::getNamespace(), "neato_laser");
        const auto& transform
          = m_tf_buffer.lookupTransform("map", base_link_id, ros::Time(0)).transform;
        m_laser_to_map = transform3DFromMsg(
          m_tf_buffer.lookupTransform("map", laser_id, ros::Time(0)).transform);
        m_map_to_baselink = transform3DFromMsg(
          m_tf_buffer.lookupTransform(base_link_id, "map", ros::Time(0)).transform);
        m_baselink_to_map = transform3DFromMsg(transform);
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }

    ROS_DEBUG_STREAM("updateCurrentPose: " << m_current_pose << std::endl);
}
Angle LidarStrat::idToAngle(unsigned int id)
{
    return Angle((double)id * 2 * M_PI / m_nb_angular_steps - M_PI);
}
unsigned int LidarStrat::angleToId(Angle a)
{
    return (unsigned int)((a + M_PI) * double(m_nb_angular_steps) / (2 * M_PI))
           % m_nb_angular_steps;
}

void LidarStrat::updateLidarScan(const sensor_msgs::LaserScan& new_scan)
{
    updateCurrentPose();
    m_obstacle_dbg = new_scan;
    std::fill(m_lidar_sensors_dists.begin(), m_lidar_sensors_dists.end(), m_max_distance);

    unsigned int i = 0;
    for (float angle = new_scan.angle_min; angle < new_scan.angle_max;
         angle += new_scan.angle_increment)
    {
        unsigned int id = angleToId(Angle(angle));
        if (new_scan.intensities[i] >= m_min_intensity
            && Distance(new_scan.ranges[i]) > m_min_distance
            && Distance(new_scan.ranges[i]) < m_max_distance)
        {
            m_lidar_sensors_dists[id] = new_scan.ranges[i];
        }
        i++;
    }
    m_laser_to_map_at_last_lidar_scan = m_laser_to_map;
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

float compute_dangerousness_from_angle(Angle a)
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
}

void LidarStrat::updateArucoObstacles(const geometry_msgs::PoseArray& newPoses)
{
    m_aruco_obstacles.clear();
    for (auto pose : newPoses.poses)
    {
        Distance distance;
        PolarPosition other_robot(m_current_pose.getPosition() - pose.position);

        // the center of the aruco is probably farther than the edge of the robot

        distance = std::max(Distance(0), Distance(other_robot.getDistance() - 0.2));

        m_aruco_obstacles.emplace_back(distance, other_robot.getAngle());

        ROS_DEBUG_STREAM("arucoObstacle:" << m_aruco_obstacles.back() << std::endl);
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
float LidarStrat::speed_inhibition(Distance distance, Angle angle, float distanceCoeff)
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
}

void LidarStrat::sendObstaclePose(PolarPosition pp, bool reverseGear)
{
    geometry_msgs::PoseStamped obstacle_pose_stamped;
    obstacle_pose_stamped.pose.position = Position(pp);
    obstacle_pose_stamped.header.frame_id
      = tf::resolve(ros::this_node::getNamespace(), "base_link");
    if (reverseGear)
    {
        m_obstacle_behind_posestamped_pub.publish(obstacle_pose_stamped);
    }
    else
    {
        m_obstacle_posestamped_pub.publish(obstacle_pose_stamped);
    }
}

void LidarStrat::updateAruco(boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id)
{
    m_arucos[id] = *arucoPose;
}

LidarStrat::LidarStrat(ros::NodeHandle& nh)
  : m_tf_listener(m_tf_buffer)
{
    printf("[LIDAR] Begin main\n");
    fflush(stdout);

    float max_dist;
    float min_dist;
    float aruco_offset;
    float border_offset;
    float fixes_offset;
    nh.param<bool>("isBlue", m_is_blue, true);
    nh.param<float>("/strategy/lidar/max_distance", max_dist, 6.0f);
    nh.param<float>("/strategy/lidar/min_distance", min_dist, 0.2f);
    nh.param<float>("/strategy/lidar/min_intensity", m_min_intensity, 10.f);
    nh.param<float>("/strategy/aruco/offset", aruco_offset, 0.20f);
    nh.param<float>("/strategy/border/offset", border_offset, -0.20f);
    nh.param<float>("/strategy/fixes/offset", fixes_offset, 0.0f);
    nh.param<int>("/strategy/obstacle/nb_angular_steps", m_nb_angular_steps, 360);

    m_max_distance = Distance(max_dist);
    m_min_distance = Distance(min_dist);
    m_aruco_obs_offset = Distance(aruco_offset);
    m_fixes_obs_offset = Distance(fixes_offset);
    m_border_obs_offset = Distance(border_offset);

    m_arucos
      = { geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(),
          geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(),
          geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped(),
          geometry_msgs::PoseStamped() }; // std::array<geometry_msgs::PoseStamped, 10>

    for (int i = 0; i < m_nb_angular_steps; i++)
    {
        m_lidar_sensors_dists.push_back(Distance(0));
        m_lidar_sensors_angles.push_back(idToAngle(i)); // conversion from loop index to degrees
        m_obstacle_dbg.intensities.push_back(0);
        m_obstacle_dbg.ranges.push_back(Distance(0));
    }
    m_aruco_obstacles.clear();

    m_obstacle_posestamped_pub
      = nh.advertise<geometry_msgs::PoseStamped>("obstacle_pose_stamped", 5);
    m_obstacle_behind_posestamped_pub
      = nh.advertise<geometry_msgs::PoseStamped>("obstacle_behind_pose_stamped", 5);
    m_obstacle_debug_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_debug", 5);

    m_lidar_sub = nh.subscribe("scan", 1000, &LidarStrat::updateLidarScan, this);
    m_aruco_obstacles_sub
      = nh.subscribe("aruco_obstacles", 5, &LidarStrat::updateArucoObstacles, this);

    if (m_is_blue)
    {
        m_arucos_sub[6] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/6", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 6));
        m_arucos_sub[7] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/7", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 7));
        m_arucos_sub[8] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/8", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 8));
        m_arucos_sub[9] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/9", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 9));
        m_arucos_sub[10] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/10", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 10));
    }
    else
    {
        m_arucos_sub[1] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/1", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 1));
        m_arucos_sub[2] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/2", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 2));
        m_arucos_sub[3] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/3", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 3));
        m_arucos_sub[4] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/4", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 4));
        m_arucos_sub[5] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/5", 5, boost::bind(&LidarStrat::updateAruco, this, _1, 5));
    }
}

void LidarStrat::closest_point_of_segment(const Position& point,
                                          const Position& segment1,
                                          const Position& segment2,
                                          Position& closestPoint)
{
    Distance xx, yy;
    closest_point_of_segment(point.getX(),
                             point.getY(),
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
void LidarStrat::closest_point_of_segment(const Distance x,
                                          const Distance y,
                                          const Distance x1,
                                          const Distance y1,
                                          const Distance x2,
                                          const Distance y2,
                                          Distance& xx,
                                          Distance& yy)
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
}

bool is_in_front(Angle a)
{
    return abs(AngleTools::diffAngle(a, Angle(0))) < AngleTools::deg2rad(AngleDeg(60));
}

bool is_in_back(Angle a)
{
    return abs(AngleTools::diffAngle(a, Angle(M_PI))) < AngleTools::deg2rad(AngleDeg(60));
}

int LidarStrat::computeMostThreatening(const std::vector<PolarPosition>& obstacles,
                                       float distanceCoeff,
                                       bool look_in_front)
{
    int currentMostThreateningId = -1;

    // 1) Find the distance to the most threatening obstacle and its relative position compared
    // to the robot
    float currentMostThreateningSpeedInhibition = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < obstacles.size(); i++)
    {
        const auto& obstacle = obstacles[i];
        // Only detect in front of the current direction
        if ((!look_in_front && !is_in_back(obstacle.getAngle()))
            || (look_in_front && !is_in_front(obstacle.getAngle())))
        {
            continue;
        }

        Angle normalized_angle = look_in_front
                                   ? obstacles[i].getAngle()
                                   : AngleTools::wrapAngle(Angle(obstacles[i].getAngle() + M_PI));

        float speed_inhibition_coeff
          = speed_inhibition(obstacles[i].getDistance(), normalized_angle, distanceCoeff);

        if (speed_inhibition_coeff < currentMostThreateningSpeedInhibition)
        {
            currentMostThreateningSpeedInhibition = speed_inhibition_coeff;
            currentMostThreateningId = i;
        }
    }
    return currentMostThreateningId;
}

bool LidarStrat::isInsideTable(const Position& input)
{
    return input.getX() < 1.45 && input.getX() > -1.4 && input.getY() < 0.95
           && input.getY() > -0.95;
}

void debugObstacle(visualization_msgs::MarkerArray& ma, const std::vector<PolarPosition>& obstacles)
{
    uint i = ma.markers.size();
    auto frame_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    if (i == 0)
    {
        visualization_msgs::Marker m;
        m.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(m);
        m.id = i++;
    }
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.seq = 0;
    m.ns = "debug_obstacles";
    m.id = i++;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.10;
    m.scale.y = 0.10;
    m.scale.z = 0.10;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0;
    m.color.g = 1;
    m.color.b = 0;
    m.color.a = 1;
    m.lifetime = ros::Duration(0); // Does not disapear
    m.frame_locked = true;
    for (const auto& obs : obstacles)
    {
        m.points.push_back(Position(obs));
    }
    ma.markers.push_back(m);
}

void debugSegments(visualization_msgs::MarkerArray& ma,
                   const std::vector<std::pair<Position, Position>>& segments)
{
    uint i = ma.markers.size();
    if (i == 0)
    {
        visualization_msgs::Marker m;
        m.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(m);
        m.id = i++;
    }
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.seq = 0;
    m.ns = "debug_obstacles";
    m.id = i++;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.022;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0;
    m.color.g = 1;
    m.color.b = 1;
    m.color.a = 1;
    m.lifetime = ros::Duration(0); // Does not disapear
    m.frame_locked = true;
    for (const auto& seg : segments)
    {
        m.points.push_back(seg.first);
        m.points.push_back(seg.second);
    }
    ma.markers.push_back(m);
}

void LidarStrat::run()
{
    float distanceCoeff = 1;
    ros::Rate loop_rate(5);
    std::vector<Distance> sensors_dists;

    /*************************************************
     *                   Main loop                   *
     *************************************************/
    while (ros::ok())
    {
        updateCurrentPose();
        std::vector<PolarPosition> obstacles;

        visualization_msgs::MarkerArray debug_obstacles_msg;

        for (size_t i = 0; i < m_nb_angular_steps; i += 1)
        {
            if (m_lidar_sensors_dists[i] < m_max_distance
                && m_lidar_sensors_dists[i] > m_min_distance)
            {
                PolarPosition obs_polar_local(m_lidar_sensors_dists[i], m_lidar_sensors_angles[i]);
                Position obs_local(obs_polar_local);
                Position obs_global = obs_local.transform(m_laser_to_map_at_last_lidar_scan);
                Position obs_in_baselink = obs_global.transform(m_map_to_baselink);

                bool allowed = isInsideTable(obs_global);
                ROS_DEBUG_STREAM("Current Pose: " << m_current_pose);
                ROS_DEBUG_STREAM("Obstacle local position: " << obs_in_baselink << std::endl);
                ROS_DEBUG_STREAM("Obstacle global position: " << obs_global << ", Inside table = "
                                                              << allowed << std::endl);

                if (allowed)
                {
                    obstacles.push_back(obs_in_baselink);
                }
            }
        }

        std::vector<std::pair<Position, Position>> border_segments;
        std::vector<std::pair<Position, Position>> fixes_segments;
        // Edges
        border_segments.push_back(std::make_pair(Position({ -1.5, -1. }), Position({ -1.5, 1 })));
        border_segments.push_back(std::make_pair(Position({ -1.5, 1 }), Position({ 1.5, 1 })));
        border_segments.push_back(std::make_pair(Position({ 1.5, 1 }), Position({ 1.5, -1 })));
        border_segments.push_back(std::make_pair(Position({ 1.5, -1 }), Position({ -1.5, -1 })));
        // Rocky zone
        fixes_segments.push_back(std::make_pair(Position({ -0, -1. }), Position({ 0, -0.7 })));
        fixes_segments.push_back(
          std::make_pair(Position({ -0.6, -1 }), Position({ -0.6, -0.850 })));
        fixes_segments.push_back(std::make_pair(Position({ 0.6, -1 }), Position({ 0.6, -0.850 })));

        for (const auto& aruco_pose : m_arucos)
        {
            // If the tag has been seen in the last two seconds
            if (ros::Time::now() - aruco_pose.header.stamp > ros::Duration(2, 0))
            {
                continue;
            }
            ROS_DEBUG_STREAM("aruco obstacle seen");
            auto position_local = Pose(aruco_pose.pose).getPosition().transform(m_map_to_baselink);
            auto shifted_position
              = PolarPosition(Distance(max(position_local.getNorme() - m_aruco_obs_offset, 0.)),
                              position_local.getAngle());
            Position closest_point(shifted_position);
            obstacles.push_back(closest_point);
        }

        for (auto segment : border_segments)
        {
            Position closestPointSegment;
            closest_point_of_segment(
              m_current_pose.getPosition(), segment.first, segment.second, closestPointSegment);
            auto closestPointSegmentLocal = closestPointSegment.transform(m_map_to_baselink);
            auto shifted_position = PolarPosition(
              Distance(max(closestPointSegmentLocal.getNorme() - m_border_obs_offset, 0.)),
              closestPointSegmentLocal.getAngle());
            Position closest_point(shifted_position);
            obstacles.push_back(closest_point);
        }

        for (auto segment : fixes_segments)
        {
            Position closestPointSegment;
            closest_point_of_segment(
              m_current_pose.getPosition(), segment.first, segment.second, closestPointSegment);
            auto closestPointSegmentLocal = closestPointSegment.transform(m_map_to_baselink);
            auto shifted_position = PolarPosition(
              Distance(max(closestPointSegmentLocal.getNorme() - m_fixes_obs_offset, 0.)),
              closestPointSegmentLocal.getAngle());
            Position closest_point(shifted_position);
            obstacles.push_back(closest_point);
        }

        size_t most_threateningId = computeMostThreatening(obstacles, distanceCoeff, true);
        size_t most_threateningBehindId = computeMostThreatening(obstacles, distanceCoeff, false);

        if (most_threateningId >= 0)
        {
            const auto& obstacle_front = obstacles[most_threateningId];
            ROS_DEBUG_STREAM("Nearest obstacle front = " << obstacle_front << std::endl);
            sendObstaclePose(obstacle_front, false);
        }
        else
        {
            sendObstaclePose(PolarPosition(Distance(1000), Angle(0)), false);
        }

        if (most_threateningBehindId >= 0)
        {
            const auto& obstacle_behind = obstacles[most_threateningBehindId];
            ROS_DEBUG_STREAM("Nearest obstacle behind = " << obstacle_behind << std::endl);
            sendObstaclePose(obstacle_behind, true);
        }
        else
        {
            sendObstaclePose(PolarPosition(Distance(1000), Angle(0)), true);
        }

        if (m_obstacle_debug_pub.getNumSubscribers())
        {
            debugObstacle(debug_obstacles_msg, obstacles);
            debugSegments(debug_obstacles_msg, border_segments);
            debugSegments(debug_obstacles_msg, fixes_segments);
            m_obstacle_debug_pub.publish(debug_obstacles_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Print a message to let the user know the script exited cleanly
    ROS_INFO_STREAM("Obstacles Node exited cleanly");
}

#include "lidarStrat.h"
#include <krabilib/position.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;

#define YEAR_2026

void LidarStrat::updateCurrentPose()
{
    try
    {
        // auto base_link_id = tf::resolve(rclcpp::this_node::getNamespace(), "base_link"); 1.7
        // Removal of support for tf_prefix
        auto base_link_id = "base_link";

        // auto laser_id = tf::resolve(rclcpp::this_node::getNamespace(), "tim_top"); 1.7 Removal of
        // support for tf_prefix
        auto laser_id = "ldlidar_top";

        const auto& transform
          = m_tf_buffer_->lookupTransform("map", base_link_id, rclcpp::Time(0)).transform;
        m_laser_to_map = transform3DFromMsg(
          m_tf_buffer_->lookupTransform("map", laser_id, rclcpp::Time(0)).transform);
        m_map_to_baselink = transform3DFromMsg(
          m_tf_buffer_->lookupTransform(base_link_id, "map", rclcpp::Time(0)).transform);
        m_baselink_to_map = transform3DFromMsg(transform);
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "updateCurrentPose: " << m_current_pose << std::endl);
}
// Maps a bin index → angle in [-π, π]. Inverse of angleToId.
Angle LidarStrat::idToAngle(unsigned int id)
{
    return Angle((double)id * 2 * M_PI / m_nb_angular_steps - M_PI);
}
// Maps an angle in [-π, π] → bin index in [0, m_nb_angular_steps).
unsigned int LidarStrat::angleToId(Angle a)
{
    return (unsigned int)((a + M_PI) * double(m_nb_angular_steps) / (2 * M_PI))
           % m_nb_angular_steps;
}

void LidarStrat::updateLidarScan(const sensor_msgs::msg::LaserScan& new_scan)
{
    updateCurrentPose();
    m_obstacle_dbg = new_scan;
    // Reset all bins to max_distance so that missing returns don't leave stale obstacle data.
    std::fill(m_lidar_sensors_dists.begin(), m_lidar_sensors_dists.end(), m_max_distance);
    m_lidar_sensors_stamp = new_scan.header.stamp;

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
    // Snapshot the transform at scan time; the robot may have moved by the time run() executes.
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

// Called when the overhead camera sends a batch of opponent poses detected via ArUco tags.
// Converts absolute poses (map frame) to robot-relative polar positions.
void LidarStrat::updateArucoObstacles(const geometry_msgs::msg::PoseArray& newPoses)
{
    m_aruco_obstacles.clear();
    for (auto pose : newPoses.poses)
    {
        Distance distance;
        PolarPosition other_robot(m_current_pose.getPosition() - pose.position);

        // The ArUco tag is on top of the opponent's mast; shrink the distance by 20 cm
        // so the obstacle represents the robot's edge rather than its center.
        distance = std::max(Distance(0), Distance(other_robot.getDistance() - 0.2));

        m_aruco_obstacles.emplace_back(distance, other_robot.getAngle());

        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "arucoObstacle:" << m_aruco_obstacles.back() << std::endl);
    }
}

void LidarStrat::sendObstaclePose(PolarPosition pp, bool reverseGear)
{
    geometry_msgs::msg::PoseStamped obstacle_pose_stamped;
    obstacle_pose_stamped.pose.position = Position(pp);
    // obstacle_pose_stamped.header.frame_id = tf::resolve(rclcpp::this_node::getNamespace(),
    // "base_link");  1.7 Removal of support for tf_prefix
    obstacle_pose_stamped.header.frame_id = "base_link";

    obstacle_pose_stamped.header.stamp = m_lidar_sensors_stamp;

    if (reverseGear)
    {
        m_obstacle_behind_posestamped_pub->publish(obstacle_pose_stamped);
    }
    else
    {
        m_obstacle_posestamped_pub->publish(obstacle_pose_stamped);
    }
}

void LidarStrat::updateAruco(std::shared_ptr<geometry_msgs::msg::PoseStamped const> arucoPose,
                             int id)
{
    m_arucos[id] = *arucoPose;
}

LidarStrat::LidarStrat()
  : Node("lidar_strat")
// : m_tf_listener(m_tf_buffer)
{

    printf("[LIDAR] Begin main\n");
    fflush(stdout);

    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);

    float max_dist;
    float min_dist;
    float lidar_offset;
    float aruco_offset;
    float border_offset;
    float static_offset;

    this->declare_parameter("isBlue", true);
    this->get_parameter("isBlue", m_is_blue);

    this->declare_parameter("/strategy/lidar/max_distance", 6.0f);
    this->get_parameter("/strategy/lidar/max_distance", max_dist);

    this->declare_parameter("/strategy/lidar/min_distance", 0.2f);
    this->get_parameter("/strategy/lidar/min_distance", min_dist);

    this->declare_parameter("/strategy/lidar/min_intensity", 10.f);
    this->get_parameter("/strategy/lidar/min_intensity", m_min_intensity);

    bool is_sim = this->get_parameter("use_sim_time").as_bool();
    if (is_sim)
    {
        m_min_intensity = 0; // Gazebo cannot produce intensities
    }

    this->declare_parameter("/strategy/lidar/offset", 0.20f);
    this->get_parameter("/strategy/lidar/offset", lidar_offset);

    this->declare_parameter("/strategy/aruco/offset", 0.20f);
    this->get_parameter("/strategy/aruco/offset", aruco_offset);

    this->declare_parameter("/strategy/border/offset", -0.35f);
    this->get_parameter("/strategy/border/offset", border_offset);

    this->declare_parameter("/strategy/static/offset", -0.08f);
    this->get_parameter("/strategy/static/offset", static_offset);

    this->declare_parameter("/strategy/obstacle/nb_angular_steps", 360);
    this->get_parameter("/strategy/obstacle/nb_angular_steps", m_nb_angular_steps);

    m_max_distance = Distance(max_dist);
    m_min_distance = Distance(min_dist);
    m_aruco_obs_offset = Distance(aruco_offset);
    m_lidar_obs_offset = Distance(lidar_offset);
    m_static_obs_offset = Distance(static_offset);
    m_border_obs_offset = Distance(border_offset);

    m_arucos = {
        geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped(),
        geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped(),
        geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped(),
        geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped(),
        geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped()
    }; // std::array<geometry_msgs::msg::PoseStamped, 10>

    for (unsigned int i = 0; i < m_nb_angular_steps; i++)
    {
        m_lidar_sensors_dists.push_back(Distance(0));
        m_lidar_sensors_angles.push_back(idToAngle(i)); // conversion from loop index to degrees
        m_obstacle_dbg.intensities.push_back(0);
        m_obstacle_dbg.ranges.push_back(Distance(0));
    }
    m_aruco_obstacles.clear();

    create_publishers();

    create_subscribers();

    m_timeout_next_publish_dynamic_obst = this->now();
    timer_
      = this->create_wall_timer(std::chrono::milliseconds{ 66 }, std::bind(&LidarStrat::run, this));
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

// "In front" means within ±60° of the robot's forward direction (angle 0).
bool is_in_front(Angle a)
{
    return abs(AngleTools::diffAngle(a, Angle(0))) < AngleTools::deg2rad(AngleDeg(60));
}

// "In back" means within ±60° of the robot's backward direction (angle π).
bool is_in_back(Angle a)
{
    return abs(AngleTools::diffAngle(a, Angle(M_PI))) < AngleTools::deg2rad(AngleDeg(60));
}

// Returns the index of the most threatening obstacle in the given direction,
// or -1 if the list is empty. "Most threatening" = lowest speed_inhibition value
// (closest to 0 = must stop). Returns -1 if no obstacle qualifies.
int LidarStrat::computeMostThreatening(const std::vector<PolarPosition>& obstacles,
                                       float distanceCoeff,
                                       bool look_in_front)
{
    int currentMostThreateningId = -1;

    // Higher initial value ensures any real obstacle will beat it.
    float currentMostThreateningSpeedInhibition = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < obstacles.size(); i++)
    {
        const auto& obstacle = obstacles[i];
        // Skip obstacles that are not in the direction we are looking.
        if ((!look_in_front && !is_in_back(obstacle.getAngle()))
            || (look_in_front && !is_in_front(obstacle.getAngle())))
        {
            continue;
        }

        // For reverse gear, remap the angle so that "straight behind" becomes 0
        // (i.e., most dangerous), matching the convention expected by speed_inhibition.
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

// The game table is 3 m × 2 m, centred at the map origin.
// We use slightly tighter bounds (1.45 / 0.95) to reject LiDAR returns
// that hit obstacles outside the table (referees, team members, etc.) and would otherwise cause
// unnecessary braking.
bool LidarStrat::isInsideTable(const Position& input)
{
    return input.getX() < 1.45 && input.getX() > -1.45 && input.getY() < 0.95
           && input.getY() > -0.95;
}

void LidarStrat::debugObstacle(visualization_msgs::msg::MarkerArray& ma,
                               const std::vector<PolarPosition>& obstacles)
{
    uint i = ma.markers.size();
    // auto frame_id = tf::resolve(ros::this_node::getNamespace(), "base_link"); 1.7 Removal of
    // support for tf_prefix
    auto frame_id = "base_link";
    if (i == 0)
    {
        visualization_msgs::msg::Marker m;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(m);
        m.id = i++;
    }
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = this->now();
    // m.header.seq = 0;
    m.ns = "debug_obstacles";
    m.id = i++;
    m.action = visualization_msgs::msg::Marker::MODIFY;
    m.scale.x = 0.10;
    m.scale.y = 0.10;
    m.scale.z = 0.10;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.color.r = 0;
    m.color.g = 1;
    m.color.b = 0;
    m.color.a = 1;
    m.lifetime = rclcpp::Duration(0, 0); // Does not disapear
    m.frame_locked = true;
    for (const auto& obs : obstacles)
    {
        m.points.push_back(Position(obs));
    }
    ma.markers.push_back(m);
}

void debugSegments(visualization_msgs::msg::MarkerArray& ma,
                   const std::vector<std::pair<Position, Position>>& segments)
{
    uint i = ma.markers.size();
    if (i == 0)
    {
        visualization_msgs::msg::Marker m;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(m);
        m.id = i++;
    }
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    //  m.header.seq = 0;
    m.ns = "debug_obstacles";
    m.id = i++;
    m.action = visualization_msgs::msg::Marker::MODIFY;
    m.scale.x = 0.022;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.color.r = 0;
    m.color.g = 1;
    m.color.b = 1;
    m.color.a = 1;
    m.lifetime = rclcpp::Duration(0, 0); // Does not disapear
    m.frame_locked = true;
    for (const auto& seg : segments)
    {
        m.points.push_back(seg.first);
        m.points.push_back(seg.second);
    }
    ma.markers.push_back(m);
}

void LidarStrat::sendDynamicObstacles(std::vector<PolarPosition> obstacles)
{
    if (m_dynamic_pose_array_pub->get_subscription_count() == 0)
    {
        // Do not waste time if no one subscribes
        return;
    }
    geometry_msgs::msg::PoseArray dynamic_obstacles_poses = geometry_msgs::msg::PoseArray();
    dynamic_obstacles_poses.header.frame_id = "map";

    for (auto position : obstacles)
    {
        Position l_obstacle_position = Position(position); // cartesian, local to robot

        l_obstacle_position = l_obstacle_position.transform(m_baselink_to_map); // in map frame
        geometry_msgs::msg::Pose dynamic_obstacle_pose = Pose(l_obstacle_position, Angle(0));

        dynamic_obstacles_poses.poses.push_back(dynamic_obstacle_pose);
    }
    dynamic_obstacles_poses.header.stamp = m_lidar_sensors_stamp;

    m_dynamic_pose_array_pub->publish(dynamic_obstacles_poses);
}

void LidarStrat::run()
{
    float distanceCoeff = 1;
    std::vector<Distance> sensors_dists;

    /*************************************************
     *                   Main loop                   *
     *************************************************/

    updateCurrentPose();
    std::vector<PolarPosition> obstacles;

    // Sentinel obstacles placed infinitely far away in 4 directions so that
    // computeMostThreatening always returns a valid index even when the field is clear.
    obstacles.push_back(PolarPosition(Distance(10000), Angle(0)));
    obstacles.push_back(PolarPosition(Distance(10000), Angle(90)));
    obstacles.push_back(PolarPosition(Distance(10000), Angle(180)));
    obstacles.push_back(PolarPosition(Distance(10000), Angle(270)));

    // The field is symmetric: blue team plays on one side, yellow on the other.
    // This coefficient flips X-coordinates of game-specific positions for the yellow team.
    int coeffIsBlue = 1;
    if (!m_is_blue) // todo: check si c'est pas l'inverse
    {
        coeffIsBlue = -1;
    }

#ifdef YEAR_2025
    centre_petite_depose_coin = Position({ -coeffIsBlue * 1.25f, 0.92f });
    centre_petite_depose_vers_public = Position({ coeffIsBlue * 0.75f, 0.92f });
    centre_aire_de_depart_vers_publique = Position({ coeffIsBlue * 0.25f, 0.75f });
    centre_aire_de_depart_cote_loin = Position({ -coeffIsBlue * 1.25f, 0.15f });
#endif
    visualization_msgs::msg::MarkerArray debug_obstacles_msg;

    for (size_t i = 0; i < m_nb_angular_steps; i += 1)
    {
        if (m_lidar_sensors_dists[i] < m_max_distance && m_lidar_sensors_dists[i] > m_min_distance)
        {
            // Compute position a first time, to see if the obstacle is inside the table
            PolarPosition obs_polar_local(m_lidar_sensors_dists[i], m_lidar_sensors_angles[i]);
            Position obs_local(obs_polar_local);
            Position obs_global = obs_local.transform(m_laser_to_map_at_last_lidar_scan);
            Position obs_in_baselink = obs_global.transform(m_map_to_baselink);

            bool allowed = isInsideTable(obs_global);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Current Pose: " << m_current_pose);
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                                "Obstacle local position: " << obs_in_baselink << std::endl);
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                                "Obstacle global position: " << obs_global << ", Inside table = "
                                                             << allowed << std::endl);

            if (allowed)
            {
#ifdef YEAR_2025
                // Check if Obstacle is close from
                if (!petite_depose_coin_activated
                    && (obs_global - centre_petite_depose_coin).getNorme() < Distance(200))
                {
                    petite_depose_coin_activated = true;
                }
                if (!petite_depose_vers_public_activated
                    && (obs_global - centre_petite_depose_vers_public).getNorme() < Distance(200))
                {
                    petite_depose_vers_public_activated = true;
                }
                if (!aire_de_depart_vers_publique_activated
                    && (obs_global - centre_aire_de_depart_vers_publique).getNorme()
                         < Distance(200))
                {
                    aire_de_depart_vers_publique_activated = true;
                }
                if (!aire_de_depart_cote_loin_activated
                    && (obs_global - centre_aire_de_depart_cote_loin).getNorme() < Distance(200))
                {
                    aire_de_depart_cote_loin_activated = true;
                }
#endif

                // Recompute with an offset (=margin if the robot is coming toward us)
                PolarPosition obs_polar_local_with_offset(
                  Distance(m_lidar_sensors_dists[i] - m_lidar_obs_offset),
                  m_lidar_sensors_angles[i]);

                Position obs_local_with_offset(obs_polar_local);
                [[maybe_unused]] Position obs_global_with_offset
                  = obs_local.transform(m_laser_to_map_at_last_lidar_scan);

                // The LiDAR only sees the opponent's mast (a single point), but their robot
                // body is much larger. We add 8 points evenly spread 20 cm around the detected
                // point to approximate the opponent's footprint and trigger avoidance earlier.
                const Distance l_rayon_robot_adverse = Distance(0.2);
                std::vector<Position> l_tour_robot_adverse;
                for (float l_angle = 0; l_angle < 2 * M_PI; l_angle += M_PI / 4)
                {
                    Position l_point_tour_robot_adverse = Position(obs_global);
                    l_point_tour_robot_adverse.setX(Distance(
                      l_point_tour_robot_adverse.getX() + l_rayon_robot_adverse * sin(l_angle)));
                    l_point_tour_robot_adverse.setY(Distance(
                      l_point_tour_robot_adverse.getY() + l_rayon_robot_adverse * cos(l_angle)));
                    l_tour_robot_adverse.push_back(l_point_tour_robot_adverse);

                    Position l_point_tour_robot_adverse_in_baselink
                      = l_point_tour_robot_adverse.transform(m_map_to_baselink);
                    obstacles.push_back(l_point_tour_robot_adverse_in_baselink);
                }

                [[maybe_unused]] Position obs_in_baselink_with_offset
                  = obs_global.transform(m_map_to_baselink);
                obstacles.push_back(obs_in_baselink);
            }
        }
    }

    // --- ArUco obstacles (legacy overhead camera, not used since 2022) ---
    for (const auto& aruco_pose : m_arucos)
    {
        // Discard stale detections; a tag unseen for >2 s is no longer reliable.
        if (this->now() - aruco_pose.header.stamp > rclcpp::Duration(2, 0))
        {
            continue;
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "aruco obstacle seen");
        auto position_local = Pose(aruco_pose.pose).getPosition().transform(m_map_to_baselink);
        auto shifted_position
          = PolarPosition(Distance(max(position_local.getNorme() - m_aruco_obs_offset, 0.)),
                          position_local.getAngle());
        Position closest_point(shifted_position);
        obstacles.push_back(closest_point);
    }

    // Dynamic obstacles (LiDAR + ArUco) are published here, before static ones are added,
    // so that the dynamic_obstacles topic only reflects what the sensors actually see.
    if (this->now() > m_timeout_next_publish_dynamic_obst)
    {
        // Rate-limited to 1 Hz because the PoseArray can be large.
        sendDynamicObstacles(obstacles);
        m_timeout_next_publish_dynamic_obst = this->now() + rclcpp::Duration(1, 0);
    }

    // --- Static obstacles ---
    // For each segment we compute the closest point to the robot and add it as an obstacle,
    // so the robot slows down when approaching any wall or fixed game structure.
    std::vector<std::pair<Position, Position>> border_segments;
    std::vector<std::pair<Position, Position>> static_segments;
    // The four edges of the 3 m × 2 m table (map frame, centred at origin).
    border_segments.push_back(std::make_pair(Position({ -1.5, -1. }), Position({ -1.5, 1 })));
    border_segments.push_back(std::make_pair(Position({ -1.5, 1 }), Position({ 1.5, 1 })));
    border_segments.push_back(std::make_pair(Position({ 1.5, 1 }), Position({ 1.5, -1 })));
    border_segments.push_back(std::make_pair(Position({ 1.5, -1 }), Position({ -1.5, -1 })));

    // Game-specific fixed obstacles. Compiled in via the YEAR_XXXX macro defined at the top.
    // Each year's competition has a different table layout with different structures to avoid.
#ifdef YEAR_2025
    // Scène
    static_segments.push_back(
      std::make_pair(Position({ 0.45f, -0.55f }), Position({ -0.45f, -0.55f })));
    static_segments.push_back(
      std::make_pair(Position({ 0.45f, -0.55f }), Position({ 0.45f, -1.0f })));
    static_segments.push_back(
      std::make_pair(Position({ -0.45f, -1.0f }), Position({ -0.45f, -0.55f })));

    // Rampes
    static_segments.push_back(
      std::make_pair(Position({ -0.45f, -0.8f }), Position({ -0.85f, -0.8f })));
    static_segments.push_back(
      std::make_pair(Position({ -0.85f, -1.0f }), Position({ -0.85f, -0.8f })));
    static_segments.push_back(
      std::make_pair(Position({ 0.45f, -0.8f }), Position({ 0.85f, -0.8f })));
    static_segments.push_back(
      std::make_pair(Position({ 0.85f, -1.0f }), Position({ 0.85f, -0.8f })));

    // Arrière-scène
    static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.45f, -0.55f }),
                                             Position({ coeffIsBlue * 1.5f, -0.55f })));
    static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.45f, -0.55f }),
                                             Position({ coeffIsBlue * 0.45f, -1.0f })));

    //  (idéalement il faudrait ne l'activer que si on y détecte le robot adverse)

    // petite dépose coin
    if (petite_depose_coin_activated)
    {
        static_segments.push_back(std::make_pair(Position({ -coeffIsBlue * 1.05f, 0.85f }),
                                                 Position({ -coeffIsBlue * 1.5f, 0.85f })));
        static_segments.push_back(std::make_pair(Position({ -coeffIsBlue * 1.05f, 0.85f }),
                                                 Position({ -coeffIsBlue * 1.05f, 0.15f })));
    }

    // petite dépose vers public
    if (petite_depose_vers_public_activated)
    {
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.5f, 0.85f }),
                                                 Position({ coeffIsBlue * 0.5f, 1.0f })));
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.5f, 0.85f }),
                                                 Position({ coeffIsBlue * 0.95f, 0.85f })));
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.95f, 0.85f }),
                                                 Position({ coeffIsBlue * 0.95f, 1.0f })));
    }

    // aire de départ vers publique
    if (aire_de_depart_vers_publique_activated)
    {
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.05f, 0.55f }),
                                                 Position({ coeffIsBlue * 0.05f, 1.0f })));
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.05f, 0.55f }),
                                                 Position({ coeffIsBlue * 0.5f, 0.55f })));
        static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.5f, 0.55f }),
                                                 Position({ coeffIsBlue * 0.5f, 1.0f })));
    }

    // aire de départ côté loin
    if (aire_de_depart_cote_loin_activated)
    {
        static_segments.push_back(std::make_pair(Position({ -coeffIsBlue * 1.05f, -0.1f }),
                                                 Position({ -coeffIsBlue * 1.5f, -0.1f })));
        static_segments.push_back(std::make_pair(Position({ -coeffIsBlue * 1.05f, -0.1f }),
                                                 Position({ -coeffIsBlue * 1.05f, 0.35f })));
        static_segments.push_back(std::make_pair(Position({ -coeffIsBlue * 1.05f, 0.35f }),
                                                 Position({ -coeffIsBlue * 1.5f, 0.35f })));
    }
#elif defined(YEAR_2026)
    // Grenier
    static_segments.push_back(
      std::make_pair(Position({ 0.9, -0.55f }), Position({ -0.9f, -0.55f })));
    static_segments.push_back(
      std::make_pair(Position({ 0.9f, -0.55f }), Position({ 0.9f, -1.0f })));
    static_segments.push_back(
      std::make_pair(Position({ -0.9f, -1.0f }), Position({ -0.9f, -0.55f })));

    // Nid adverse
    static_segments.push_back(std::make_pair(Position({ coeffIsBlue * 0.9f, -0.55f }),
                                             Position({ coeffIsBlue * 1.5f, -0.55f })));

#endif

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

    for (auto segment : static_segments)
    {
        Position closestPointSegment;
        closest_point_of_segment(
          m_current_pose.getPosition(), segment.first, segment.second, closestPointSegment);
        auto closestPointSegmentLocal = closestPointSegment.transform(m_map_to_baselink);
        auto shifted_position = PolarPosition(
          Distance(max(closestPointSegmentLocal.getNorme() - m_static_obs_offset, 0.)),
          closestPointSegmentLocal.getAngle());
        Position closest_point(shifted_position);
        obstacles.push_back(closest_point);
    }

    int most_threateningId = computeMostThreatening(obstacles, distanceCoeff, true);
    int most_threateningBehindId = computeMostThreatening(obstacles, distanceCoeff, false);

    if (most_threateningId >= 0)
    {
        const auto& obstacle_front = obstacles[most_threateningId];
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "Nearest obstacle front = " << obstacle_front << std::endl);
        sendObstaclePose(obstacle_front, false);
    }
    else
    {
        sendObstaclePose(PolarPosition(Distance(1000), Angle(0)), false);
    }

    if (most_threateningBehindId >= 0)
    {
        const auto& obstacle_behind = obstacles[most_threateningBehindId];
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "Nearest obstacle behind = " << obstacle_behind << std::endl);
        sendObstaclePose(obstacle_behind, true);
    }
    else
    {
        sendObstaclePose(PolarPosition(Distance(1000), Angle(0)), true);
    }

    if (m_obstacle_debug_pub->get_subscription_count())
    {
        debugObstacle(debug_obstacles_msg, obstacles);
        debugSegments(debug_obstacles_msg, border_segments);
        debugSegments(debug_obstacles_msg, static_segments);
        m_obstacle_debug_pub->publish(debug_obstacles_msg);
    }
}

void LidarStrat::updateRemainingTime(builtin_interfaces::msg::Duration a_remaining_time_match)
{
    m_remainig_time = rclcpp::Duration(a_remaining_time_match);

#ifdef YEAR_2025
    // A match lasts 100 s, but the robot only plays for 85s.
    // After 82 s, we stop considering the "petite dépose" and "aire de départ" zones as obstacles,
    // and try to score there as a last measure
    if (m_remainig_time.seconds() > 82)
    {
        petite_depose_coin_activated = false;
        petite_depose_vers_public_activated = false;
        aire_de_depart_vers_publique_activated = false;
        aire_de_depart_cote_loin_activated = false;
    }
#endif
}
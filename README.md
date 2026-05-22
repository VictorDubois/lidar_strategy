# Obstacle handling ROS2 node for [the robot Krabi](https://github.com/VictorDubois/krabi)

<img width="1896" height="737" alt="image" src="https://github.com/user-attachments/assets/14064a03-3aae-42b0-9148-b06af27e4d84" />

## Inputs

### scan_obstacles
Pointcloud from a lidar, to detect obstalces (type: [LaserScan](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))

### aruco_obstacles
Poses of arUco tags on top of opponents robots (type: [PoseArray](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseArray.html)). This was sent by the camera above the paly area in 2020/2022, but is not used anymore. Could be used again.

### remaining_time
The remaining time in the match, in seconds (type: [Duration](https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html))

## Outputs

### obstable_pose_stamped
The pose of the most threatening obstacle in front of the robot (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

### obstable_behind_pose_stamped
The pose of the most threatening obstacle behind the robot (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

### obstacle_debug
Visualization (rviz/foxglove) of obstacles considered by the robot, both static (ex: edges of the play area), and dynamic (i.e. seen by the lidar). (type: [MakerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

### dynamic_obstacles
The poses of the obstacles built from what the lidar sees. (type: [PoseArray](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseArray.html)). There are two differences from scan_obstacles:
  - The points outside of the table are excluded, see [isInsideTable](https://github.com/search?q=repo%3AVictorDubois/lidar_strategy%20LidarStrat%3A%3AisInsideTable&type=code)
  - The opponent's robot is reconstructed. We consider that each point seen by the lidar is just the mast, and the real robot is much larger => 8 points are added 20cm around the lidar impact. See [l_rayon_robot_adverse](https://github.com/search?q=repo%3AVictorDubois%2Flidar_strategy%20l_rayon_robot_adverse&type=code))

# Visualization
<img width="1322" height="800" alt="image" src="https://github.com/user-attachments/assets/db5610a6-62d4-447d-bf9a-c040f21c7849" />

- Krabi is the red/green/blue arrows on the left
- Pink: scan_obstacles
- Cyan: edges of the area, part of obstacle_debug
- arrows on the bottom right: dynamic_obstacles (from the lidar points inside the table) (it is the big blob on the bottom right, there are also green points and pink squares on top)
- Green: obstacles considered (one per edge in Cyan + dynamic obstacles), part of obstacle_debug

From the green points, are selected:
- Red: obstable_pose_stamped
- Yellow: obstable_behind_pose_stamped

=> only those two are sent to main_strategy

# How to launch
`ros2 launch lidar_strategy lidar_strat_launch.py`

or usually, as part of [Krabi bringup](https://github.com/VictorDubois/krabi/blob/main/krabi_bringup/launch/krabi_main_launch.py)

# What to do for a new year?

Once the official rules for a new year of Eurobot are available, the tasks are:
- [Mandatory] define the static obstacles, [see here for 2026](https://github.com/search?q=repo%3AVictorDubois%2Flidar_strategy+defined%28YEAR_2026%29&type=code)
- [Optional] define zones to activate/deactivate/change the priority based on whether the other robto has been there. ex:
    - [in 2025](https://github.com/VictorDubois/lidar_strategy/blob/d31d567f61ba2a71b4844f6958a3e79efc0669dc/src/lidarStrat.cpp#L471), forbid going throw a zone where another robot has potentially dropped a construction
    - most years, lower the priority of resource zones where the other robot has already been (in 2026, it would have been nice to avoid trying to get noisettes if the other robot has already gotten them)

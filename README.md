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
<img width="1123" height="857" alt="image" src="https://github.com/user-attachments/assets/a9137e0a-d07d-4fe7-8811-9279cc8cb531" />

- Pink: scan_obstacles
- Cyan: edges of the area, part of obstacle_debug
- arrows on the bottom right: dynamic_obstacles (from the lidar points inside the table)
- Green: obstacles considered (one per edge in Cyan + dynamic obstacles), part of obstacle_debug

From the green points, are selected:
- Red: obstable_pose_stamped
- Yellow: obstable_behind_pose_stamped

# How to launch
`ros2 launch lidar_strategy lidar_strat_launch.py`

or usually, as part of [Krabi bringup](https://github.com/VictorDubois/krabi/blob/main/krabi_bringup/launch/krabi_main_launch.py)

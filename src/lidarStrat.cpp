#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include "../include/lidarStrat.h"

#define NB_NEURONS 360
//#define NB_MEASURES_LIDAR 360

#define CRITICAL_DISTANCE 0.100 // in m
#define CRITCIAL_DISTANCE_TRIG 0.120 // in m
#define LIDAR_DIST_OFFSET 0.080 // in m, the lidar is much more deep inside the robot, vs the US/IR near the outside
#define MAX_DISTANCE 6.0 // in m
#define MIN_DISTANCE 0.1 // in m
#define MIN_INTENSITY 10

// The smoothing factor for the obstacles' intensity
#define ALPHA 1

std::vector<float> raw_sensors_dists(NB_NEURONS, 0);

void updateLidarScan(sensor_msgs::LaserScan new_scan) {
	for (int i = 0; i < NB_MEASURES_LIDAR ; i++) {
		if (new_scan.intensities[i] < MIN_INTENSITY) {
			// Unreliable, do not take into account
			raw_sensors_dists[i] = MAX_DISTANCE;
		}
		else {
      raw_sensors_dists[i] = new_scan.ranges[i];
		}
	}
}

unsigned int get_idx_of_max (const float vector[], const size_t len) {
	unsigned int curr_max = 0, i;

	for (i = 1; i < len; i += 1) {
		if (vector[i] > vector[curr_max])
			curr_max = i;
	}
	return curr_max;
}

float sin_card(float x) {
	if (x == 0) {
		return 1;
	}
	if (x < -90 || x > 90) {
		return 0;
	}
	//x =2.0* M_PI * x / 180;
	x = M_PI * x / 180;
	return sin(x)/x;
}

/**
 * Convert a polar position to a cartesian one
 * @param posX the X position, in m
 * @param posY the Y position, in m
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
void polar_to_cart(float &posX, float &posY, const float theta, const float distance) {
    posX = distance*cos(theta * M_PI/180.f);
    posY = distance*sin(theta * M_PI/180.f);
}

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in m
 * @param posY the Y position, in m
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void cart_to_polar(const float posX, const float posY, float& theta, float& distance) {
        theta = ((180./M_PI) * atan2((float) posX, (float) posY));
        distance = sqrt((float)(posX * posX + posY * posY));

        // fix angular ambiguity
        if (posY < 0) {
                theta += 180;
        }

        #ifdef DEBUG_cart_to_polar
                std::cout << "posX = " << posX << "posY = " <<  posY<< "theta = " << theta << ", distance = " << distance << std::endl;
                float posXafter = distance*cos(theta * M_PI/180.f);
                float posYafter = distance*sin(theta * M_PI/180.f);
                std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
        #endif
}

// Entry point
int main (int argc, char * argv[]) {
	printf("[LIDAR] Begin main\n");
	fflush(stdout);

	ros::init(argc, argv, "lidarStrat");
	ros::start();

  float obstacle_distance = 2000; // dist of the nearest obstacle, in m
  unsigned int nearest_obstacle_angle;
	std::vector<int> lidar_sensors_angles; // in deg
	for(int i = 0; i < NB_NEURONS; i++) {
		lidar_sensors_angles.push_back(i);
	}

  	float distanceCoeff = 1;
	std::vector<float> sensors_dists(NB_NEURONS, 0);
  ros::Rate loop_rate(5);
	
	// Initialize nearest's obstacle in front of us
	nearest_obstacle_angle = 0;
	std::vector<float> pre_output(NB_NEURONS, 0);

	ros::NodeHandle n;
	ros::Publisher obstacle_pose_pub = n.advertise<geometry_msgs::Pose>("obstacle_pose", 1000);
	ros::Publisher obstacle_posestamped_pub = n.advertise<geometry_msgs::PoseStamped>("obstacle_pose_stamped", 1000);
	ros::Subscriber lidar_sub = n.subscribe("scan", 1000, &updateLidarScan);

	/*************************************************
	 *                   Main loop                   *
	 *************************************************/
	while (ros::ok()) {
		// Modulate the stop/brake distance from other strats' inputs

    float brake_max = 0.65;//m. Distance at which we start to brake, (was in dm)
    float brake_min = 0.5;//m. Distance at which we start to brake, (was in dm)
    float brake = brake_max;//m. Distance at which we start to brake, (was in dm)
    float stop_min = 0.45 * distanceCoeff;//m. Distance at which we stop, (was in cm)
    float stop_max = 0.55 * distanceCoeff;//m. Distance at which we stop, (was in cm)
    float stop = stop_max;//m Distance at which we stop, (was in cm)

    /*if (distanceCoeff != oldDistanceCoeff) {
			std::cout << "Stop distance coeff read = " << distanceCoeff << std::endl;
			std::cout << "brake_max = " << brake_max << ", brake_min = " << brake_min << ", brake = " << brake << ", stop_min = " << stop_min << "stop_max = " << stop_max << ", stop = " << stop << std::endl;
			fflush(stdout);
		}*/

		// Update the values with a smoothing factor
    for (int i = 0; i < NB_MEASURES_LIDAR; i += 1) {
			if (raw_sensors_dists[i] < MIN_DISTANCE) {
        raw_sensors_dists[i] = MAX_DISTANCE;
      }
      sensors_dists[i] = raw_sensors_dists[i];
      // Smooth output
      /*else if (raw_sensors_dists[i] > sensors_dists[i]) {
        sensors_dists[i] = (1. - ALPHA) * sensors_dists[i] + ALPHA * raw_sensors_dists[i];
			}
			else {
        sensors_dists[i] = (1. - ALPHA) * sensors_dists[i] + ALPHA * raw_sensors_dists[i];
      }*/

			//std::cout << "Smoothed uS sensors n°" << i << ": " << sensors_dists[i] << std::endl;
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

		// 1) Find the distance to the nearest obstacle and its relative position compared to the robot
		float currentMostThreatening = 0.;
    for (int i = 0; i < NB_MEASURES_LIDAR; i += 1) {
			float angle_factor = 1.0 - sin_card(((float)i) - 180.0);

			/*
			 * Output each sensor's obstacle distance on the output neural field
			 * Range is from 0 (obstacle very far) to 1 (obstacle very close)
			 *
			 */
      float close_factor = 15.f;// min distance for complete stop?
      float far_factor = 40.f;// distance from which the object is not taken into account?
			float d = angle_factor * ( (close_factor - (sensors_dists[i] - LIDAR_DIST_OFFSET) / close_factor) / far_factor + 1.);

			if (currentMostThreatening < d) {
				obstacle_distance = sensors_dists[i];
				currentMostThreatening = d;
				nearest_obstacle_angle = lidar_sensors_angles[i];
			}
		}

		/*
		 * 2) Compute linear speed
		 * TODO: modulate the speed by the angle of the us sensors!
		 */

		// We modulate the intensity of the inhibition based on the angle at which the obstacle is seen
		float angle_factor = sin_card(((float)nearest_obstacle_angle) - 180.0);
		stop = stop_min + (stop_max-stop_min)*angle_factor;
		brake = brake_min + (brake_max-brake_min)*angle_factor;

		//@TODO replace this in main strat
    //strategy.output->speed_inhibition = MAX_ALLOWED_SPEED * (50. * tanh((obstacle_distance - middle) / slope) + 49) / 100.);

		//if (obstacle_distance < CRITICAL_DISTANCE) {
		//	emergency_stop[2] = 0.0;
			//} else if(obstacle_distance >= CRITCIAL_DISTANCE_TRIG) {
			//emergency_stop[2] = 1.0;
		//}

		//printf("Distance: %d mm @ %d deg-> Linear Speed Inhibition: %d \n", obstacle_distance, nearest_obstacle_angle, strategy.output->speed_inhibition);

	/*	if (strategy.output->speed_inhibition > MAX_ALLOWED_SPEED)
			strategy.output->speed_inhibition = MAX_ALLOWED_SPEED;
		if (strategy.output->speed_inhibition < 0)
			strategy.output->speed_inhibition = 0;*/

		// Now that the work is done on 'pre_output', actually output it
		//std::cout << "nearest_obstacle_angle = " << nearest_obstacle_angle << ", at " << obstacle_distance << " mm " << std::endl;

		geometry_msgs::Pose obstacle_pose;
    float posX;
    float posY;
    polar_to_cart(posX, posY, nearest_obstacle_angle, obstacle_distance);
    obstacle_pose.position.x = posX;
        obstacle_pose.position.y = posY;
		obstacle_pose_pub.publish(obstacle_pose);
		geometry_msgs::PoseStamped obstacle_pose_stamped;
		obstacle_pose_stamped.pose = obstacle_pose;
    obstacle_pose_stamped.header.frame_id = "base_link";//"neato_laser";//base_link for robot, neato_laser for logs/debug
		obstacle_posestamped_pub.publish(obstacle_pose_stamped);
		//std::cout << "cart X = " << obstacle_pose.position.x << ", Y = " << obstacle_pose.position.y << std::endl;

		//printf("Distance: %d cm @ %d deg-> Linear Speed Inhibition: %d\n", obstacle_distance, nearest_obstacle_angle, strategy.output->speed_inhibition);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Print a message to let the user know the script exited cleanly
	printf("Obstacles script exited cleanly\n");

	return 0;
}

#undef NB_NEURONS
#undef ALPHA

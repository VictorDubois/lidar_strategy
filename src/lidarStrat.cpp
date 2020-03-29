#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

#include "../include/lidarStrat.h"

#define NB_NEURONS 360
//#define NB_MEASURES_LIDAR 360

#define CRITICAL_DISTANCE 100 // in mm
#define CRITCIAL_DISTANCE_TRIG 120 // in mm
#define LIDAR_DIST_OFFSET 80 // in mm, the lidar is much more deep inside the robot, vs the US/IR near the outside
#define MAX_DISTANCE 6000 // in mm

// The smoothing factor for the obstacles' intensity
#define ALPHA 1

std::vector<float> raw_sensors_dists;

void updateLidarScan(sensor_msgs::LaserScan new_scan) {
	raw_sensors_dists = new_scan.ranges;
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
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
void polar_to_cart(int &posX, int &posY, const float theta, const float distance) {
        posX = (int)1000* distance*cos(theta * M_PI/180.f);
        posY = (int)1000* distance*sin(theta * M_PI/180.f);
}

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void cart_to_polar(const int posX, const int posY, float& theta, float& distance) {
        theta = ((180./M_PI) * atan2((float) posX, (float) posY));
        distance = sqrt((float)(posX * posX + posY * posY))/1000.f;

        // fix angular ambiguity
        if (posY < 0) {
                theta += 180;
        }

        #ifdef DEBUG_cart_to_polar
                std::cout << "posX = " << posX << "posY = " <<  posY<< "theta = " << theta << ", distance = " << distance << std::endl;
                int posXafter =(int)1000* distance*cos(theta * M_PI/180.f);
                int posYafter =(int)1000* distance*sin(theta * M_PI/180.f);
                std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
        #endif
}

// Entry point
int main (int argc, char * argv[]) {
	printf("[LIDAR] Begin main\n");
	fflush(stdout);

	ros::init(argc, argv, "lidarStrat");


	int obstacle_distance = 2000; // dist of the nearest obstacle, in cm
	unsigned int i, nearest_obstacle_angle;
	//unsigned int us_sensors_angles[NB_US_SENSORS + NB_TOF_SENSORS]; // in deg
	unsigned int lidar_sensors_angles[NB_MEASURES_LIDAR]; // in deg

	float a, d;
	float brake_max = 6.5;//dm. Distance at which we start to brake, in decimeters!!!
	float brake_min = 5;//dm. Distance at which we start to brake, in decimeters!!!
	float brake = brake_max;//dm. Distance at which we start to brake, in decimeters!!!
	float stop_min = 45;//cm. Distance at which we stop, in cm
	float stop_max = 55;//cm. Distance at which we stop, in cm
	float stop = stop_max;//cm. Distance at which we stop, in cm
  	float distanceCoeff = 1;
	float* sensors_dists;
	
	// Initialize nearest's obstacle in front of us
	nearest_obstacle_angle = 0;
	std::vector<float> pre_output;

	ros::NodeHandle n;
	ros::Publisher obstacle_pose_pub = n.advertise<geometry_msgs::Pose>("obstacle_pose", 1000);
	ros::Subscriber lidar_sub = n.subscribe("scan", 1000, &updateLidarScan);

	/*************************************************
	 *                   Main loop                   *
	 *************************************************/
	while (ros::ok()) {
		// Modulate the stop/brake distance from other strats' inputs

		brake_max = 6.5 * distanceCoeff;//dm. Distance at which we start to brake, in decimeters!!!
		brake_min = 5 * distanceCoeff;//dm. Distance at which we start to brake, in decimeters!!!
		brake = brake_max;//dm. Distance at which we start to brake, in decimeters!!!
		stop_min = 45 * distanceCoeff;//cm. Distance at which we stop, in cm
		stop_max = 55 * distanceCoeff;//cm. Distance at which we stop, in cm
		stop = stop_max;//cm. Distance at which we stop, in cm
	
		/*if (distanceCoeff != oldDistanceCoeff) {	
			std::cout << "Stop distance coeff read = " << distanceCoeff << std::endl;
			std::cout << "brake_max = " << brake_max << ", brake_min = " << brake_min << ", brake = " << brake << ", stop_min = " << stop_min << "stop_max = " << stop_max << ", stop = " << stop << std::endl;
			fflush(stdout);
		}*/

		// Update the values with a smoothing factor
		for (i = 0; i < NB_MEASURES_LIDAR; i += 1) {
			if (raw_sensors_dists[i] == 0) {
				raw_sensors_dists[i] = (uint16_t) MAX_DISTANCE;
			}
			else if (raw_sensors_dists[i] > sensors_dists[i]) {
				sensors_dists[i] = (uint16_t)ceil((1. - a) * sensors_dists[i] + a * raw_sensors_dists[i]);
			}
			else {
				sensors_dists[i] = (uint16_t)floor((1. - a) * sensors_dists[i] + a * raw_sensors_dists[i]);
			}

			//printf("Smoothed uS sensors #%d: %d mm\n", i, sensors_dists[i]);
		}
		//printf("\n\n");
		
		/*
		 * Reactive approach to obstacles:
		 * - Simplified: obstacles on the right, left or both, output on the neural field
		 * - Linear speed is linearly modulated by the distance of the obstacle and it comes
		 *   to a stop if too close
		 */
		
		// Linear modulation of the linear speed
		// 0) Reset obstacle_distance & position
		// TODO: Try using double buffering here: maybe the output of the obstacle is dirty because we keep resetting the output vector!

		obstacle_distance = 6000;

		// 1) Find the distance to the nearest obstacle and its relative position compared to the robot
		float currentMostThreatening = 0.;
		for (i = 0; i < NB_MEASURES_LIDAR; i += 1) {
			float angle_factor = 1.0 - sin_card(((float)i) - 180.0);

			/*
			 * Output each sensor's obstacle distance on the output neural field
			 * Range is from 0 (obstacle very far) to 1 (obstacle very close)
			 *
			 */
			float close_factor = 15.f;// min distance for complete stop?
			float far_factor = 40.f;// distance from which the object is not taken into account?
			d = angle_factor * ( (close_factor - (sensors_dists[i] - LIDAR_DIST_OFFSET) / close_factor) / far_factor + 1.);

			if (currentMostThreatening < d) {
				obstacle_distance = sensors_dists[i];
				currentMostThreatening = d;
				nearest_obstacle_angle = lidar_sensors_angles[i];
			}

			if (d < 0.)
				d = 0.;
			if (d > 1.0)
				d = 1.0;

			// Test displaying obstacles centered on 180 deg
			pre_output[(360 + 180 - lidar_sensors_angles[i]) % 360] *= (float) (1. - ALPHA);
			pre_output[(360 + 180 - lidar_sensors_angles[i]) % 360] += (float) ALPHA * d;

			// bypass the rest
			//pre_output[(360 + 180 - lidar_sensors_angles[i]) % 360] = (float) raw_sensors_dists[i];

		}

		/*
		 * 2) Compute linear speed
		 * TODO: modulate the speed by the angle of the us sensors!
		 */

		// We modulate the intensity of the inhibition based on the angle at which the obstacle is seen
		float angle_factor = sin_card(((float)nearest_obstacle_angle) - 180.0);
		stop = stop_min + (stop_max-stop_min)*angle_factor;
		brake = brake_min + (brake_max-brake_min)*angle_factor;
		
		/*if (nearest_obstacle_angle == 0) {
			brake = 3.5;
			stop = 25.0;
		
		} else if (nearest_obstacle_angle == 40 || nearest_obstacle_angle == 325) {
			brake = 6.5;
			stop = 29;
		} else if (nearest_obstacle_angle == 60 || nearest_obstacle_angle == 300) {
		        brake = 4.0; //9.0;
			stop = 19.0; //30.0;
		}*/

		//@TODO replace this in main strat
		//strategy.output->speed_inhibition = (int) round(MAX_ALLOWED_SPEED * (50. * tanh((obstacle_distance / 10. - stop) / brake) + 49) / 100.);

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

		geometry_msgs::Pose obstacle_pose;
		int obstacle_x_in_mm;
		int obstacle_y_in_mm;
		polar_to_cart(obstacle_x_in_mm, obstacle_y_in_mm, nearest_obstacle_angle, obstacle_distance);
		obstacle_pose.position.x = obstacle_x_in_mm;
		obstacle_pose.position.y = obstacle_y_in_mm;
		obstacle_pose_pub.publish(obstacle_pose);

		//printf("Distance: %d cm @ %d deg-> Linear Speed Inhibition: %d\n", obstacle_distance, nearest_obstacle_angle, strategy.output->speed_inhibition);
		
		//usleep(30000); // No need to sleep, receiving packets does that
		ros::spinOnce();
	}

	// Print a message to let the user know the script exited cleanly
	printf("Obstacles script exited cleanly\n");

	return 0;
}

#undef NB_NEURONS
#undef ALPHA

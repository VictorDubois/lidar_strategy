#ifndef _LIDAR_H_
#define _LIDAR_H_

#define TRUE 1
#define FALSE 0

// Angular speed at which we will turn to avoid an obstacle
#define ANGULAR_SPEED_FLIGHT 9 // in %

// Maximum speed allowed by the obstacles
//#define MAX_ALLOWED_SPEED 20 comment because already declare in strategie.h
//#define NB_US_SENSORS 5
//#define NB_TOF_SENSORS 0 //6
#define NB_MEASURES_LIDAR 360 

// Maximum amplitude of the obstacles gaussian derivative
#define MAX_GAUSSIAN_AMPLITUDE 3.0


#endif

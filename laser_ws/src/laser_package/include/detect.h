#ifndef DETECT
#define DETECT

#include <ros/ros.h>
#include <math.h>
#include <rosbag/bag.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>


#define MIN_TARGET_WIDTH 0.1	//10cm
#define MAX_TARGET_WIDTH 0.7  //60cm
#define MAX_TARGET_DEPTH 0.7  //40cm
#define THETA_DELTA 0.006283185 //angular resolution of lidar in radians

extern std::vector<float>  filtered_ranges, point_targets;
extern int place_keeper,new_series,num_points_min, num_points_max,num_points, mean_index;
extern sensor_msgs::LaserScan scan_msg;
extern float mean_range;

#endif

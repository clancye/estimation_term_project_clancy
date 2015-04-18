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
#define START_STEP 44
#define END_STEP 725
#define MIN_VAR 0.05  //how far the mean of the points should be away from the median. rough approximation. 
#define INITIALIZING 1
#define WAITING_FOR_MOVING_TARGET 2

class Detector
{
	public:
		Detector()
			{
				//std::vector<float>  filtered_ranges (682);
				//std::vector<float>  point_targets (682);
				//std::vector<float>  initial_target_vec (682);
			}
	
		
	
		std::vector<float> detectTargets(const std::vector<float>& ranges);
		void setState(int some_state);
	
	private:
	
		int place_keeper,new_series, num_points_min, num_points_max,num_points,mean_index,state;
		std::vector<float>  filtered_ranges, point_targets,initial_target_vec;
		sensor_msgs::LaserScan scan_msg;
		float mean_range;
};

#endif

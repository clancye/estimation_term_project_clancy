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
#define INDEX_SHIFT  127 //44 + (pi/6)*THETA_DELTA
#define START_STEP 44
#define END_STEP 725
#define MIN_VAR 0.05  //how far the mean of the points should be away from the median. rough approximation. 
#define MAX_MOVEMENT_PER_SCAN 0.35
#define VELOCITY_THRESHOLD_MIN 0.0014 // heuristic
#define VELOCITY_THRESHOLD_MAX 0.15
#define WALL_SLOPE_THRESHOLD 0.5
#define INITIALIZING 1
#define WAITING_FOR_MOVING_TARGET 2
#define TRACKING 3

class Detector
{
	public:
	
		Detector()
			:point_targets (682),initial_targets (682)
			{
				state = INITIALIZING;
				zoneBeingTracked = -1;
			}
	
		
	
		std::vector<float> detectTargets(std::vector<float> ranges);
		void setState(int some_state);
		int getState();
		void setScanTime();
		int getNumberOfZones();
		int getZoneBeingTracked();
	
	private:
	
		int new_series, num_points_min, num_points_max,num_points,state, zone_counter,zoneBeingTracked;
		double scan_time;
		std::vector<float>  point_targets,initial_targets, zone_range_min, zone_range_max,possible_target_range, possible_target_x, possible_target_y, possible_target_time;
		std::vector<int> zone_index_min, zone_index_max,possible_target_index;
		int getZone(float range,int index);
		void createZone(float mean_range,int mean_index,int zoneBeingTracked);
		void updateVelocity(int zone, float range, int index, double time);
		float getCartesianX(float range, float index);
		float getCartesianY(float range, float index);
		void trackZone(int zone);
		bool isThisAWall(float first_x, float first_y, float mean_x, float mean_y, float last_x, float last_y);
		float getSlope(float x_1, float y_1, float x_2, float y_2);
		
};

#endif

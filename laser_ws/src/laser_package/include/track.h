#ifndef TRACK
#define TRACK

#include <ros/ros.h>
#include <math.h>
#include <rosbag/bag.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "laser_package/update_tracker.h"




class Tracker
{
	public:
	
		Tracker()
			//INITIALIZE VECTORS HERE
			{
			}
	
		void predictState();
	
	private:
	

		
};

#endif

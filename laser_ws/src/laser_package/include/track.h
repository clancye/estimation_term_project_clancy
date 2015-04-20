#ifndef TRACK
#define TRACK

#include <ros/ros.h>
#include <math.h>
#include <rosbag/bag.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>




class Tracker
{
	public:
	
		Tracker()
			://INITIALIZE VECTORS HERE
			{
			}
	
		void predictState();
	
	private:
	

		
};

#endif

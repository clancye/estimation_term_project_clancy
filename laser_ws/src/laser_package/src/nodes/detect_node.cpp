#include "../../include/detect.h"


Detector target_detector = Detector();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			ROS_INFO("Constructing SAP...");
			ros::Duration(10).sleep(); // sleep for 10 seconds
			laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,&SubscribeAndPublish::laserCallBack,this);
			target_pub = n.advertise<sensor_msgs::LaserScan>("/targets",1000); //publish targets to new topic
			std::vector<float> filtered_ranges (682);
			std::vector<float> initial_targets (682);
			scan_counter = 0;
			state = INITIALIZING;
			ROS_INFO("Go!");
		}
		

		void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
		{ 
			target_detector.setScanTime();// need this to calculate velocities
			targets_msg = *msg;
			target_detector.setState(state);
			targets_msg.ranges = target_detector.detectTargets(msg->ranges);
			target_pub.publish(targets_msg);//publish so we can see on RViz
			if(scan_counter<=10)scan_counter++;//time to gather some possible targets
			//if(scan_counter>10&&target_detector.getNumberOfZones() >0 )state = WAITING_FOR_MOVING_TARGET;//keep initializing if no targets
			else if (scan_counter > 10 && target_detector.getNumberOfZones() == 0){scan_counter = 0;}//once we have some targets, wait until they move
			//if (target_detector.getZoneBeingTracked() != -1) state = TRACKING;
		}
	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub;
		ros::Subscriber laser_sub;
		int scan_counter, state;//NEW CODE
		std::vector<float> filtered_ranges, initial_targets;
		sensor_msgs::LaserScan targets_msg;
		float time;
};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "detect_node");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPDetect;
	ros::spin();

	return 0;
}



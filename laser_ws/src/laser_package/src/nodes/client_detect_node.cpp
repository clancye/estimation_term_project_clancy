#include "../../include/detect.h"


Detector target_detector = Detector();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			ROS_INFO("Constructing SAP");
			laser_sub = n.subscribe<sensor_msgs::LaserScan>("/client_scan",1000,&SubscribeAndPublish::laserCallBack,this);
			target_pub = n.advertise<sensor_msgs::LaserScan>("/client_targets",1000); //publish targets to new topic
			std::vector<float> filtered_ranges (682);
			std::vector<float> initial_targets (682);
			scan_counter = 0;
			state = INITIALIZING;
			//ros::Duration(10).sleep(); // sleep for 10 seconds
		}
		

		void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
		{ 
			targets_msg = *msg;
			target_detector.setState(state);
			filtered_ranges = target_detector.detectTargets(msg->ranges);
			targets_msg.ranges = filtered_ranges;
			target_pub.publish(targets_msg);//NEW CODE ADDED CONITIONAL
			if(scan_counter<=10)scan_counter++;//NEW CODE
			else{state = WAITING_FOR_MOVING_TARGET;};
		}//dfd
	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub;
		ros::Subscriber laser_sub;
		int scan_counter, state;//NEW CODE
		std::vector<float> filtered_ranges, initial_targets;
		sensor_msgs::LaserScan targets_msg;
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



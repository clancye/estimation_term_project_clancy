#include "../../include/track.h"


Tracker tracker = Tracker();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			ROS_INFO("Constructing SAP for tracking node...");
			target_pub = n.advertise<geometry_msgs::PointStamped>("/target_topic",1000); //publish targets to new topic
			service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateTrackerCallBack,this);
			real_msg.point = msg;
			real_msg.header.frame_id = "/my_frame";
		}
		

		bool updateTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			msg.x = req.x;
			msg.y = req.y;
			real_msg.point = msg;
			target_pub.publish(real_msg);
		}
	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub;	
		ros::ServiceServer service; 
		geometry_msgs::Point msg;
		geometry_msgs::PointStamped real_msg;
		
};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "track_node");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPDetect;
	ros::spin();

	return 0;
}






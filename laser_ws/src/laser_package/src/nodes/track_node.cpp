#include "../../include/track.h"


//Tracker tracker = Tracker();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			Eigen::MatrixXf initial_state(2,1);	
			noise_data << 0.0,1.0,0.0,1.0;//mu_v, sigma_v, mu_w, sigma_w
			initial_state << 0,10;tracker = Tracker(noise_data,DWNA_X,initial_state);
			ROS_INFO("Constructing SAP for tracking node...");
			target_pub = n.advertise<geometry_msgs::PointStamped>("/target_topic",1000); //publish targets to new topic
			state_pub = n.advertise<laser_package::state>("/state_topic",1000); //publish targets to new topic
			service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateTrackerCallBack,this);
			real_msg.point = msg;
			real_msg.header.frame_id = "/my_frame";
		}
		

		bool updateTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			//msg.x = req.x;
			//msg.y = req.y;
			tracker.updateX(req.measured_x, req.update_time);//change this command based on how many variables you are updating
			state_msg.Measured_X = req.measured_x;
			state_msg.Measured_Y = req.measured_y;
			state_msg.Real_X = req.real_x;
			state_msg.Real_Y = req.real_y;
			state_msg.Acceleration_X = tracker.getXAcceleration();
			state_msg.Acceleration_Y = tracker.getYAcceleration();
			state_msg.Velocity_X = tracker.getXVelocity();
			state_msg.Velocity_Y = tracker.getYVelocity();
			state_pub.publish(state_msg);
			
			//real_msg.point = msg;
			//target_pub.publish(real_msg);
		}

	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub, state_pub;	
		ros::ServiceServer service; 
		geometry_msgs::Point msg;
		geometry_msgs::PointStamped real_msg;
		laser_package::state state_msg;
		Eigen::RowVector4f noise_data;
		Tracker tracker;
		//Eigen::MatrixXf measurement_matrix;
		
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






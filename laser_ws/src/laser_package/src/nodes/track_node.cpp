#include "../../include/track.h"


//Tracker tracker = Tracker();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			Eigen::MatrixXf measurement_matrix(1,2);
			Eigen::MatrixXf initial_covariance = Eigen::MatrixXf::Identity(2,2);
			Eigen::MatrixXf initial_state(2,1);	
			noise_data << 1.0,2.0,3.0,4.0;
			measurement_matrix << 1,0;
			initial_state << 0,10;
			tracker = Tracker(noise_data,0,measurement_matrix, initial_covariance, initial_state);
			ROS_INFO("Constructing SAP for tracking node...");
			target_pub = n.advertise<geometry_msgs::PointStamped>("/target_topic",1000); //publish targets to new topic
			state_pub = n.advertise<laser_package::state>("/state_topic",1000); //publish targets to new topic
			service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateTrackerCallBack,this);
			real_msg.point = msg;
			real_msg.header.frame_id = "/my_frame";
		}
		

		bool updateTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			msg.x = req.x;
			msg.y = req.y;
			state_msg.x = req.x;
			state_msg.y = req.y;
			state_msg.Acceleration_X = tracker.getXAcceleration();
			state_msg.Acceleration_Y = tracker.getYAcceleration();
			state_msg.Velocity_X = tracker.getXVelocity();
			state_msg.Velocity_Y = tracker.getYVelocity();
			if(req.y>1.25&&req.y<4.5)state_pub.publish(state_msg);
			tracker.update(req.x,req.y, req.update_time);
			real_msg.point = msg;
			target_pub.publish(real_msg);
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






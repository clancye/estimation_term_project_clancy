#include "../../include/track.h"


//Tracker tracker = Tracker();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			target_pub_1 = n.advertise<geometry_msgs::PointStamped>("/target_topic_1",1000); //publish targets to new topic
			target_pub_2 = n.advertise<geometry_msgs::PointStamped>("/target_topic_2",1000); //publish targets to new topic
			state_pub_1 = n.advertise<laser_package::state>("/state_topic_1",1000); //publish targets to new topic
			state_pub_2 = n.advertise<laser_package::state>("/state_topic_2",1000); //publish targets to new topic
			update_service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateTrackerCallBack,this);
			initialize_service = n.advertiseService("initializeTracker", &SubscribeAndPublish::initializeTrackerCallBack,this);
			
			//These are for when we want to publish to Rviz
			//real_msg_1.point = msg_1;
			//real_msg_1.header.frame_id = "/my_frame";
			//real_msg_2.point = msg_2;
			//real_msg_2.header.frame_id = "/my_frame";
		}
		
		

		bool updateTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			//fill measurement vector with new values
			z(0) = req.measured_x;
			z(1) = req.measured_y;
			
			//update the trackers
			tracker_1.update(z, req.update_time);
			tracker_2.update(z, req.update_time);
			
			//update the state messages
			updateStateMessage(tracker_1, &state_msg_1, req);
			updateStateMessage(tracker_2, &state_msg_2, req);
			
			//publish all this info to /state_topic
			state_pub_1.publish(state_msg_1);
			state_pub_2.publish(state_msg_2);
			//The below allows us to publish values so that Rviz can plot. You decide which points to plot from the target class. 
			//real_msg_1.point = msg_1;
			//target_pub_1.publish(real_msg_1);
			//real_msg_2.point = msg_2;
			//target_pub_2.publish(real_msg_2);
		}
		
		bool initializeTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			Eigen::MatrixXf initial_state(4,1);
			initial_state << req.initial_x, req.initial_x_velocity, req.initial_y, req.initial_y_velocity;
			tracker_1 = Tracker(initial_state);
			tracker_2 = Tracker(initial_state);
			updateStateMessage(tracker_1, &state_msg_1, req);
			updateStateMessage(tracker_2, &state_msg_2, req);
			state_pub_1.publish(state_msg_1);
			state_pub_2.publish(state_msg_2);
			ROS_INFO("Tracker initialized");
			
		}

	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub_1, target_pub_2, state_pub_1, state_pub_2;	
		ros::ServiceServer initialize_service, update_service; 
		geometry_msgs::Point msg_1, msg_2;
		geometry_msgs::PointStamped real_msg_1, real_msg_2;
		laser_package::state state_msg_1, state_msg_2;
		Eigen::Vector2f z;
		Tracker tracker_1, tracker_2;
		void updateStateMessage(Tracker tracker, laser_package::state *state_msg, laser_package::update_tracker::Request &req)
		{
			//real vs. measured x and y values
			state_msg->Real_X = req.real_x;
			state_msg->Real_Y = req.real_y;
			state_msg->Measured_X = req.measured_x;
			state_msg->Measured_Y = req.measured_y;
			
			//predicted state values
			state_msg->Predicted_X = tracker.getPredictedX();
			state_msg->Predicted_X_Velocity = tracker.getPredictedXVel();
			state_msg->Predicted_Y = tracker.getPredictedY();
			state_msg->Predicted_Y_Velocity = tracker.getPredictedYVel();
			
			
			//calculated velocities and accelerations
			state_msg->Acceleration_X = tracker.getXAcceleration();
			state_msg->Acceleration_Y = tracker.getYAcceleration();
			state_msg->Velocity_X = tracker.getXVelocity();
			state_msg->Velocity_Y = tracker.getYVelocity();
			
			//variances
			state_msg->Position_Variance_X = tracker.getPositionVarianceX();
			state_msg->Velocity_Variance_X = tracker.getVelocityVarianceX();
			state_msg->Position_Variance_Y = tracker.getPositionVarianceY();
			state_msg->Velocity_Variance_Y = tracker.getVelocityVarianceY();
			
			//gains
			state_msg->Position_Gain_X = tracker.getPositionGainX();
			state_msg->Velocity_Gain_X = tracker.getVelocityGainX();
			state_msg->Position_Gain_Y = tracker.getPositionGainY();
			state_msg->Velocity_Gain_Y = tracker.getVelocityGainY();
			//innovations
			state_msg->Innovation_X = tracker.getInnovationX();
			state_msg->Innovation_Y = tracker.getInnovationY();
		}
		
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






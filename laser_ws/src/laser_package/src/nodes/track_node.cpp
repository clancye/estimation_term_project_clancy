#include "../../include/track.h"


//Tracker tracker = Tracker();

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			target_pub = n.advertise<geometry_msgs::PointStamped>("/target_topic",1000); //publish targets to new topic
			state_pub = n.advertise<laser_package::state>("/state_topic",1000); //publish targets to new topic
			update_service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateTrackerCallBack,this);
			initialize_service = n.advertiseService("initializeTracker", &SubscribeAndPublish::initializeTrackerCallBack,this);
			real_msg.point = msg;
			real_msg.header.frame_id = "/my_frame";
		}
		

		bool updateTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			z(0) = req.measured_x;
			z(1) = req.measured_y;
			
			tracker.update(z, req.update_time);
			
			//real vs. measured x and y values
			state_msg.Real_X = req.real_x;
			state_msg.Real_Y = req.real_y;
			state_msg.Measured_X = req.measured_x;
			state_msg.Measured_Y = req.measured_y;
			
			//predicted state values
			state_msg.Predicted_X = tracker.getPredictedX();
			state_msg.Predicted_X_Velocity = tracker.getPredictedXVel();
			state_msg.Predicted_Y = tracker.getPredictedY();
			state_msg.Predicted_Y_Velocity = tracker.getPredictedYVel();
			
			
			//calculated velocities and accelerations
			state_msg.Acceleration_X = tracker.getXAcceleration();
			state_msg.Acceleration_Y = tracker.getYAcceleration();
			state_msg.Velocity_X = tracker.getXVelocity();
			state_msg.Velocity_Y = tracker.getYVelocity();
			
			//variances
			state_msg.Position_Variance_X = tracker.getPositionVarianceX();
			state_msg.Velocity_Variance_X = tracker.getVelocityVarianceX();
			state_msg.Position_Variance_Y = tracker.getPositionVarianceY();
			state_msg.Velocity_Variance_Y = tracker.getVelocityVarianceY();
			
			//gains
			state_msg.Position_Gain_X = tracker.getPositionGainX();
			state_msg.Velocity_Gain_X = tracker.getVelocityGainX();
			state_msg.Position_Gain_Y = tracker.getPositionGainY();
			state_msg.Velocity_Gain_Y = tracker.getVelocityGainY();
			//innovations
			state_msg.Innovation_X = tracker.getInnovationX();
			state_msg.Innovation_Y = tracker.getInnovationY();
			
			//print statements to debug
			ROS_INFO("Measured x on track node = %f\n Measured y on track node = %f\n", state_msg.Measured_X,state_msg.Measured_Y);
			ROS_INFO("Predicted measurement x = %f\n", state_msg.Predicted_X_Measurement);
			ROS_INFO("REAL x = %f \n Predicted x = %f \n", state_msg.Real_X, state_msg.Predicted_X);
			
			//publish all this info to /state_topic
			state_pub.publish(state_msg);
			
			//real_msg.point = msg;
			//target_pub.publish(real_msg);
		}
		
		bool initializeTrackerCallBack(laser_package::update_tracker::Request &req, laser_package::update_tracker::Response &res)
		{ 
			Eigen::MatrixXf initial_state(4,1);
			initial_state << req.initial_x, req.initial_x_velocity, req.initial_y, req.initial_y_velocity;
			tracker = Tracker(initial_state);
			state_msg.Predicted_X = tracker.getPredictedX();
			state_msg.Predicted_Y = tracker.getPredictedY();
			state_msg.Predicted_X_Velocity = tracker.getPredictedXVel();
			state_msg.Predicted_Y_Velocity = tracker.getPredictedYVel();
			state_pub.publish(state_msg);
			ROS_INFO("Tracker initialized");
			
		}

	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub, state_pub;	
		ros::ServiceServer initialize_service, update_service; 
		geometry_msgs::Point msg;
		geometry_msgs::PointStamped real_msg;
		laser_package::state state_msg;
		Eigen::Vector2f z;
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






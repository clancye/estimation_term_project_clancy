#include "../../include/track.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			initializeNoises();
			target_pub_2 = n.advertise<geometry_msgs::PointStamped>("/target_topic_2",1000); //publish targets to new topic
			state_pub_1 = n.advertise<laser_package::state>("/state_topic_1",1000); //publish targets to new topic
			state_pub_2 = n.advertise<laser_package::state>("/state_topic_2",1000); //publish targets to new topic
			update_service = n.advertiseService("updateTracker", &SubscribeAndPublish::updateFilterCallBack,this);
			initialize_service = n.advertiseService("initializeTracker", &SubscribeAndPublish::initializeFilterCallBack,this);
			
		
		}
		
		

		bool updateFilterCallBack(laser_package::update_filter::Request &req, laser_package::update_filter::Response &res)
		{ 
			//fill measurement vector with new values
			z(0) = req.measured_x;
			z(1) = req.measured_y;
			
			//update the trackers
			filter_1.update(z, req.update_time,UM);
			filter_2.update(z, req.update_time,CT);
			
			//update the state messages
			updateStateMessage(&tracker_1, &state_msg_1, req);
			updateStateMessage(&tracker_2, &state_msg_2, req);
			
			//publish all this info to /state_topic
			state_pub_1.publish(state_msg_1);
			state_pub_2.publish(state_msg_2);
		}
		
		bool initializeFilterCallBack(laser_package::update_filter::Request &req, laser_package::update_filter::Response &res)
		{ 
			Eigen::MatrixXd initial_state(5,1);
			
			initial_state << req.initial_x, req.initial_x_velocity, req.initial_y, req.initial_y_velocity, req.initial_turn_rate;
			filter_1 = Filter(initial_state, req.sampling_interval, noise_data[FILTER_1], UM);
			filter_2 = Filter(initial_state, req.sampling_interval, noise_data[FILTER_2], CT);
			updateStateMessage(&filter_1, &state_msg_1, req);
			updateStateMessage(&filter_2, &state_msg_2, req);
			state_pub_1.publish(state_msg_1);
			state_pub_2.publish(state_msg_2);
			ROS_INFO("Filter initialized");
			
		}

	private:
	   
		ros::NodeHandle n; 
		ros::Publisher target_pub_1, target_pub_2, state_pub_1, state_pub_2;	
		ros::ServiceServer initialize_service, update_service; 
		laser_package::state state_msg_1, state_msg_2;
		Eigen::Vector2d z;
		std::vector<Eigen::MatrixXd> noise_data;
		Eigen::MatrixXd noise_data_1, noise_data_2;
		Filter filter_1, filter_2;
		void updateStateMessage(Filter *filter, laser_package::state *state_msg, laser_package::update_filter::Request &req)
		{
			//real vs. measured x and y values
			state_msg->Real_X = req.real_x;
			state_msg->Real_Y = req.real_y;
			state_msg->Measured_X = req.measured_x;
			state_msg->Measured_Y = req.measured_y;
			
			//predicted state values
			state_msg->Predicted_X = tracker->getPredictedX();
			state_msg->Predicted_X_Velocity = tracker->getPredictedXVel();
			state_msg->Predicted_Y = tracker->getPredictedY();
			state_msg->Predicted_Y_Velocity = tracker->getPredictedYVel();
			
			
			//calculated velocities and accelerations
			state_msg->Acceleration_X = tracker->getXAcceleration();
			state_msg->Acceleration_Y = tracker->getYAcceleration();
			state_msg->Velocity_X = tracker->getXVelocity();
			state_msg->Velocity_Y = tracker->getYVelocity();
			
			//variances
			state_msg->Position_Variance_X = tracker->getPositionVarianceX();
			state_msg->Velocity_Variance_X = tracker->getVelocityVarianceX();
			state_msg->Position_Variance_Y = tracker->getPositionVarianceY();
			state_msg->Velocity_Variance_Y = tracker->getVelocityVarianceY();
			state_msg->Omega_Variance = tracker->getOmegaVariance();
			
			//gains
			state_msg->Position_Gain_X = tracker->getPositionGainX();
			state_msg->Velocity_Gain_X = tracker->getVelocityGainX();
			state_msg->Position_Gain_Y = tracker->getPositionGainY();
			state_msg->Velocity_Gain_Y = tracker->getVelocityGainY();
			//innovations
			state_msg->Innovation_X = tracker->getInnovationX();
			state_msg->Innovation_Y = tracker->getInnovationY();
		}
		
		void initializeNoises()
		{
			noise_data_1.resize(12,1);
			noise_data_2.resize(12,1);
			
			noise_data_1(MU_W,0) = 0.0;
			noise_data_1(SIGMA_W,0) = 1.0;
			noise_data_1(VAR_W,0) = noise_data_1(SIGMA_W,0)*noise_data_1(SIGMA_W,0);
			
			noise_data_1(MU_V_XI,0) = 0.0;
			noise_data_1(SIGMA_V_XI,0) = 4.0;
			noise_data_1(VAR_V_XI,0) = noise_data_1(SIGMA_V_XI,0)*noise_data_1(SIGMA_V_XI,0);
		
			noise_data_1(MU_V_ETA,0) = 0.0;
			noise_data_1(SIGMA_V_ETA,0) = 1.0;
			noise_data_1(VAR_V_ETA,0) = noise_data_1(SIGMA_V_ETA,0)*noise_data_1(SIGMA_V_ETA,0);
			
			noise_data_1(MU_V_OMEGA,0) = 0.0;
			noise_data_1(SIGMA_V_OMEGA,0) = 1.0;
			noise_data_1(VAR_V_OMEGA,0) = noise_data_1(SIGMA_V_OMEGA,0)*noise_data_1(SIGMA_V_OMEGA,0);
			
			///////////Tracker 2//////////////
			
			noise_data_2(MU_W) = 0.0;
			noise_data_2(SIGMA_W) = 1.0;
			noise_data_2(VAR_W) = noise_data_2(SIGMA_W)*noise_data_2(SIGMA_W);
			
			noise_data_2(MU_V_XI) = 0.0;
			noise_data_2(SIGMA_V_XI) = 1.0;
			noise_data_2(VAR_V_XI) = noise_data_2(SIGMA_V_XI)*noise_data_2(SIGMA_V_XI);
		
			noise_data_2(MU_V_ETA) = 0.0;
			noise_data_2(SIGMA_V_ETA) = 1.0;
			noise_data_2(VAR_V_ETA) = noise_data_2(SIGMA_V_ETA)*noise_data_2(SIGMA_V_ETA);
			
			noise_data_2(MU_V_OMEGA) = 0.0;
			noise_data_2(SIGMA_V_OMEGA) = 0.01;
			noise_data_2(VAR_V_OMEGA) = noise_data_2(SIGMA_V_OMEGA)*noise_data_2(SIGMA_V_OMEGA);
			
			noise_data.push_back(noise_data_1);
			noise_data.push_back(noise_data_2);
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






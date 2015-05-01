#include "../include/Estimator.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish(ros::NodeHandle *some_n)
		{	
			ROS_INFO("Constructing SAP for Estimator");
			n_handle = some_n;
			target_real_pub = n_handle->advertise<geometry_msgs::PointStamped>("/target_real_topic",1000); //publish targets to new topic
			target_sub = n_handle->subscribe<laser_package::state>("/filter_topic",1000,&SubscribeAndPublish::targetCallBack,this);
			kalman_pub = n_handle->advertise<laser_package::state>("/kalman_filter_topic",1000); //publish kf estimates to new topic
			extended_kalman_pub = n_handle->advertise<laser_package::state>("/extended_kalman_filter_topic",1000); //publish ekf estimates to new topic
			imm_pub = n_handle->advertise<laser_package::state>("/imm_topic",1000); //publish imm estimates to new topic
			msg_count = 0;
			setKalmanNoiseData();
			setExtendedKalmanNoiseData();
		}
		
		void targetCallBack(const laser_package::state::ConstPtr& msg)
		{
			
			rho = msg->Measured_Rho;
			theta = msg->Measured_Theta;
			z(0) = rho;
			z(1) = theta;
			if(msg_count>1)
			{
				update_time = msg->Time_Of_Measurement;
				//extended kalman filter
				ExtendedKF.updateFilter(z, update_time);
				ExtendedKF.updateCovariance();
				updateStateMessage(&ExtendedKF,msg);
				extended_kalman_pub.publish(state_msg);
				//regular kalman filter
				KF.updateFilter(z, update_time);
				KF.updateCovariance();
				updateStateMessage(&KF,msg);
				kalman_pub.publish(state_msg);
				//imm
				imm.update();
				updateStateMessage(&imm,msg);
				imm_pub.publish(state_msg);
			}
			else if(msg_count == 0)
			{
				first_xi = rho*cos(theta);
				first_eta = rho*sin(theta);
				first_time = msg->Time_Of_Measurement;
				msg_count++;
			}
			else if(msg_count==1)
			{
				second_time = msg->Time_Of_Measurement;
				initial_state(XI_INDEX) = rho*cos(theta);
				initial_state(ETA_INDEX) = rho*sin(theta);
				initial_state(XI_DOT_INDEX) = (initial_state(XI_INDEX)-first_xi)/(second_time-first_time);
				initial_state(ETA_DOT_INDEX) = (initial_state(ETA_INDEX)-first_eta)/(second_time-first_time);
				initial_state(OMEGA_INDEX) = 0.01;
				T = SAMPLING_INTERVAL;
				ExtendedKF = ExtendedKalmanFilter(initial_state,T , extended_kalman_noise_data,0.5,z);//instantiate Extended kalman filter
				updateStateMessage(&ExtendedKF,msg);
				extended_kalman_pub.publish(state_msg);
				KF = KalmanFilter(initial_state,T , kalman_noise_data, 0.5,z);//instantiate kalman filter
				updateStateMessage(&KF,msg);
				kalman_pub.publish(state_msg);
				filters.push_back(&KF);
				filters.push_back(&ExtendedKF);
				imm = IMM(filters);//instantiate IMM with vector of filters
				imm.update();
				msg_count++;
			}
			//These are for when we want to publish to Rviz
			target_point_msg.x = msg->Real_X;
			target_point_msg.y = msg->Real_Y;
			target_point_stamped_msg.point = target_point_msg;
			target_point_stamped_msg.header.frame_id = "/my_frame";
			//The below allows us to publish values so that Rviz can plot. You decide which points to plot from the target class. 
			target_real_pub.publish(target_point_stamped_msg);
		}
	private:
	ros::NodeHandle *n_handle;
	ros::Subscriber target_sub;
	ros::Publisher kalman_pub, extended_kalman_pub, imm_pub, target_real_pub;
	geometry_msgs::Point target_point_msg;
	geometry_msgs::PointStamped target_point_stamped_msg;
	laser_package::state state_msg;
	ExtendedKalmanFilter ExtendedKF;
	KalmanFilter KF;
	std::vector<Filter*> filters;
	IMM imm;
	int filterID, msg_count;
	double first_xi, second_xi, first_eta, second_eta, first_time, second_time,T, update_time, rho, theta, converted_xi, converted_eta;
	state_vector initial_state;
	initial_noise_vector kalman_noise_data, extended_kalman_noise_data;
	measurement_vector z;

	void setKalmanNoiseData()
	{
		kalman_noise_data(MU_W_XI_INDEX) = MU_W_SIMULATE_XI;
		kalman_noise_data(SIGMA_W_XI_INDEX) = SIGMA_W_SIMULATE_XI;
		kalman_noise_data(VAR_W_XI_INDEX) = kalman_noise_data(SIGMA_W_XI_INDEX)*kalman_noise_data(SIGMA_W_XI_INDEX);
		
		kalman_noise_data(MU_W_ETA_INDEX) = MU_W_SIMULATE_ETA;
		kalman_noise_data(SIGMA_W_ETA_INDEX) = SIGMA_W_SIMULATE_ETA;
		kalman_noise_data(VAR_W_ETA_INDEX) = kalman_noise_data(SIGMA_W_ETA_INDEX)*kalman_noise_data(SIGMA_W_ETA_INDEX);
			
		kalman_noise_data(MU_V_XI_INDEX) = 0.0;
		kalman_noise_data(SIGMA_V_XI_INDEX) = 1.0;
		kalman_noise_data(VAR_V_XI_INDEX) = kalman_noise_data(SIGMA_V_XI_INDEX)*kalman_noise_data(SIGMA_V_XI_INDEX);
	
		kalman_noise_data(MU_V_ETA_INDEX) = 0.0;
		kalman_noise_data(SIGMA_V_ETA_INDEX) = 1.0;
		kalman_noise_data(VAR_V_ETA_INDEX) = kalman_noise_data(SIGMA_V_ETA_INDEX)*kalman_noise_data(SIGMA_V_ETA_INDEX);
		
		kalman_noise_data(MU_V_OMEGA_INDEX) = 0.0;
		kalman_noise_data(SIGMA_V_OMEGA_INDEX) = 0.0;
		kalman_noise_data(VAR_V_OMEGA_INDEX) = kalman_noise_data(SIGMA_V_OMEGA_INDEX)*kalman_noise_data(SIGMA_V_OMEGA_INDEX);
		
		kalman_noise_data(SIGMA_W_RHO_INDEX) = SIGMA_W_SIMULATE_RHO;
		kalman_noise_data(SIGMA_W_THETA_INDEX) = SIGMA_W_SIMULATE_THETA;
	}
	
	void setExtendedKalmanNoiseData()
	{
		extended_kalman_noise_data(MU_W_XI_INDEX) = 0.0;
		extended_kalman_noise_data(SIGMA_W_XI_INDEX) = 1.0;
		extended_kalman_noise_data(VAR_W_XI_INDEX) = extended_kalman_noise_data(SIGMA_W_XI_INDEX)*extended_kalman_noise_data(SIGMA_W_XI_INDEX);
		
		extended_kalman_noise_data(MU_W_ETA_INDEX) = 0.0;
		extended_kalman_noise_data(SIGMA_W_ETA_INDEX) = 1.0;
		extended_kalman_noise_data(VAR_W_ETA_INDEX) = extended_kalman_noise_data(SIGMA_W_ETA_INDEX)*extended_kalman_noise_data(SIGMA_W_ETA_INDEX);
			
		extended_kalman_noise_data(MU_V_XI_INDEX) = 0.0;
		extended_kalman_noise_data(SIGMA_V_XI_INDEX) = 1.0;
		extended_kalman_noise_data(VAR_V_XI_INDEX) = extended_kalman_noise_data(SIGMA_V_XI_INDEX)*extended_kalman_noise_data(SIGMA_V_XI_INDEX);
	
		extended_kalman_noise_data(MU_V_ETA_INDEX) = 0.0;
		extended_kalman_noise_data(SIGMA_V_ETA_INDEX) = 1.0;
		extended_kalman_noise_data(VAR_V_ETA_INDEX) = extended_kalman_noise_data(SIGMA_V_ETA_INDEX)*extended_kalman_noise_data(SIGMA_V_ETA_INDEX);
		
		extended_kalman_noise_data(MU_V_OMEGA_INDEX) = 0.0;
		extended_kalman_noise_data(SIGMA_V_OMEGA_INDEX) = 0.01;
		extended_kalman_noise_data(VAR_V_OMEGA_INDEX) = extended_kalman_noise_data(SIGMA_V_OMEGA_INDEX)*extended_kalman_noise_data(SIGMA_V_OMEGA_INDEX);
		
		extended_kalman_noise_data(SIGMA_W_RHO_INDEX) = SIGMA_W_SIMULATE_RHO;
		extended_kalman_noise_data(SIGMA_W_THETA_INDEX) = SIGMA_W_SIMULATE_THETA;
	
	}
	void updateStateMessage(Filter *filter, const laser_package::state::ConstPtr& msg)
	{
		
		//Real and measured
		state_msg.Real_X = msg->Real_X;
		state_msg.Real_Y = msg->Real_Y;
		state_msg.Measured_X = msg->Measured_X;
		state_msg.Measured_Y = msg->Measured_Y;
		
		//Estimated state values
		state_msg.Estimated_X = filter->getEstimatedX();
		state_msg.Estimated_X_Velocity = filter->getEstimatedXVel();
		state_msg.Estimated_Y = filter->getEstimatedY();
		state_msg.Estimated_Y_Velocity = filter->getEstimatedYVel();
		
		
		//calculated velocities and accelerations
		state_msg.Acceleration_X = filter->getXAcceleration();
		state_msg.Acceleration_Y = filter->getYAcceleration();
		state_msg.Velocity_X = filter->getXVelocity();
		state_msg.Velocity_Y = filter->getYVelocity();
		
		//variances
		state_msg.Position_Variance_X = filter->getPositionVarianceX();
		state_msg.Velocity_Variance_X = filter->getVelocityVarianceX();
		state_msg.Position_Variance_Y = filter->getPositionVarianceY();
		state_msg.Velocity_Variance_Y = filter->getVelocityVarianceY();
		state_msg.Omega_Variance = filter->getOmegaVariance();
		
		//gains
		state_msg.Position_Gain_X = filter->getPositionGainX();
		state_msg.Velocity_Gain_X = filter->getVelocityGainX();
		state_msg.Position_Gain_Y = filter->getPositionGainY();
		state_msg.Velocity_Gain_Y = filter->getVelocityGainY();
		//innovations
		state_msg.Innovation_X = filter->getInnovationX();
		state_msg.Innovation_Y = filter->getInnovationY();
	}

};


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "Estimator_node");
	ros::NodeHandle n;
	ros::Rate r(floor(1/SIMULATE_RATE+0.5));
	SubscribeAndPublish SAPekf(&n);
	
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}



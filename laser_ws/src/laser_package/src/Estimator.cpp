#include "../../include/Estimator.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for EKF");
			target_sub = n.subscribe<laser_package::state>("/filter_topic",1000,&SubscribeAndPublish::targetCallBack,this);
			kalman_pub = n.advertise<laser_package::state>("/Kalman_Filter_topic",1000); //publish kf targets to new topic
			extended_kalman_pub = n.advertise<laser_package::state>("/Extended_Kalman_Filter_topic",1000); //publish ekf targets to new topic
			msg_count = 0;
			setKalmanNoiseData();
			setExtendedKalmanNoiseData();
		}
		
		void targetCallBack(const laser_package::state::ConstPtr& msg)
		{
			if(msg_count>1)
			{
				z(0) = msg->Measured_X;
				z(1) = msg->Measured_Y;
				update_time = msg->Time_Of_Measurement;
				ExtendedKF.update(z, update_time);
				updateStateMessage(&ExtendedKF,msg);
				extended_kalman_pub.publish(state_msg);
				KF.update(z, update_time);
				updateStateMessage(&KF,msg);
				kalman_pub.publish(state_msg);
			}
			else if(msg_count == 0)
			{
				first_xi = msg->Measured_X;
				first_eta = msg->Measured_Y;
				first_time = msg->Time_Of_Measurement;
				msg_count++;
			}
			else if(msg_count==1)
			{
				second_time = msg->Time_Of_Measurement;
				initial_state(XI) = msg->Measured_X;
				initial_state(ETA) = msg->Measured_Y;
				initial_state(XI_DOT) = (initial_state(XI)-first_xi)/(second_time-first_time);
				initial_state(ETA_DOT) = (initial_state(ETA)-first_eta)/(second_time-first_time);
				initial_state(OMEGA) = 0.01;
				T = second_time-first_time;
				ExtendedKF = ExtendedKalmanFilter(initial_state,T , extended_kalman_noise_data);
				updateStateMessage(&ExtendedKF,msg);
				extended_kalman_pub.publish(state_msg);
				KF = KalmanFilter(initial_state,T , kalman_noise_data);
				updateStateMessage(&KF,msg);
				kalman_pub.publish(state_msg);
				msg_count++;
			}
		}
	private:
	ros::NodeHandle n;
	ros::Subscriber target_sub;
	ros::Publisher kalman_pub, extended_kalman_pub;
	Filter ExtendedKF,KF;
	IMM imm;
	int filterID, msg_count;
	double first_xi, second_xi, first_eta, second_eta, first_time, second_time,T, update_time;
	state_vector initial_state;
	initial_noise_vector kalman_noise_data, extended_kalman_noise_data;
	measurement_vector z;

	void setKalmanNoiseData()
	{
		kalman_noise_data(MU_W) = 0.0;
		kalman_noise_data(SIGMA_W) = 1.0;
		kalman_noise_data(VAR_W) = kalman_noise_data(SIGMA_W)*kalman_noise_data(SIGMA_W);
		
		kalman_noise_data(MU_V_XI) = 0.0;
		kalman_noise_data(SIGMA_V_XI) = 1.0;
		kalman_noise_data(VAR_V_XI) = kalman_noise_data(SIGMA_V_XI)*kalman_noise_data(SIGMA_V_XI);
	
		kalman_noise_data(MU_V_ETA) = 0.0;
		kalman_noise_data(SIGMA_V_ETA) = 1.0;
		kalman_noise_data(VAR_V_ETA) = kalman_noise_data(SIGMA_V_ETA)*kalman_noise_data(SIGMA_V_ETA);
		
		kalman_noise_data(MU_V_OMEGA) = 0.0;
		kalman_noise_data(SIGMA_V_OMEGA) = 0.1;
		kalman_noise_data(VAR_V_OMEGA) = kalman_noise_data(SIGMA_V_OMEGA)*kalman_noise_data(SIGMA_V_OMEGA);
	}
	
	void setExtendedKalmanNoiseData()
	{
		extended_kalman_noise_data(MU_W) = 0.0;
		extended_kalman_noise_data(SIGMA_W) = 1.0;
		extended_kalman_noise_data(VAR_W) = extended_kalman_noise_data(SIGMA_W)*extended_kalman_noise_data(SIGMA_W);
		
		extended_kalman_noise_data(MU_V_XI) = 0.0;
		extended_kalman_noise_data(SIGMA_V_XI) = 1.0;
		extended_kalman_noise_data(VAR_V_XI) = extended_kalman_noise_data(SIGMA_V_XI)*extended_kalman_noise_data(SIGMA_V_XI);
	
		extended_kalman_noise_data(MU_V_ETA) = 0.0;
		extended_kalman_noise_data(SIGMA_V_ETA) = 1.0;
		extended_kalman_noise_data(VAR_V_ETA) = extended_kalman_noise_data(SIGMA_V_ETA)*extended_kalman_noise_data(SIGMA_V_ETA);
		
		extended_kalman_noise_data(MU_V_OMEGA) = 0.0;
		extended_kalman_noise_data(SIGMA_V_OMEGA) = 0.1;
		extended_kalman_noise_data(VAR_V_OMEGA) = extended_kalman_noise_data(SIGMA_V_OMEGA)*extended_kalman_noise_data(SIGMA_V_OMEGA);
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

	void updateIMMService(Filter *filter)
	{
		update_imm.request.xi_j = 0;
	}
};


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "ekf_node");
	SubscribeAndPublish SAPekf;
	
	ros::spin();
	
	return 0;
}



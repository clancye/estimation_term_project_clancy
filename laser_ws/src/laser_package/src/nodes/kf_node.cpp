#include "../../include/kf.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for KF");
			kf = Filter();
			target_sub = n.subscribe<laser_package::state>("/filter_topic",1000,&SubscribeAndPublish::targetCallBack,this);
			target_pub = n.advertise<laser_package::state>("/kf_topic",1000); //publish targets to new topic
			add_filter_to_IMM = n.serviceClient<laser_package::add_filter>("addFilter");
			initial_state.resize(5);
			msg_count = 0;
			setNoiseData();
			add_to_imm_srv.request.addMe = 1;
			if(add_filter_to_IMM.call(add_to_imm_srv))
			{
				filterID = add_to_imm_srv.response.filterID;
				ROS_INFO("Here is my number: %d", add_to_imm_srv.response.filterID);
			}
		}
		
		void targetCallBack(const laser_package::state::ConstPtr& msg)
		{
			if(msg_count>1)
			{
				z(0) = msg->Measured_X;
				z(1) = msg->Measured_Y;
				update_time = msg->Time_Of_Measurement;
				kf.updateKF(z, update_time);
				updateStateMessage(&kf,msg);
				target_pub.publish(state_msg);
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
				kf.initializeKF(initial_state,T , noise_data);
				updateIMMService(&kf);
				msg_count++;
			}
		}
	private:
	ros::NodeHandle n;
	ros::Subscriber target_sub;
	ros::Publisher target_pub;
	ros::ServiceClient add_filter_to_IMM, update_imm_filter;
	laser_package::state state_msg;
	Filter kf;
	int filterID, msg_count;
	double first_xi, second_xi, first_eta, second_eta, first_time, second_time,T, update_time;
	laser_package::add_filter add_to_imm_srv;
	laser_package::update_imm_filter update_imm;
	state_vector initial_state;
	initial_noise_vector noise_data;
	measurement_vector z;

	void setNoiseData()
	{
		noise_data.resize(12);
			
		noise_data(MU_W) = 0.0;
		noise_data(SIGMA_W) = 1.0;
		noise_data(VAR_W) = noise_data(SIGMA_W)*noise_data(SIGMA_W);
		
		noise_data(MU_V_XI) = 0.0;
		noise_data(SIGMA_V_XI) = 1.0;
		noise_data(VAR_V_XI) = noise_data(SIGMA_V_XI)*noise_data(SIGMA_V_XI);
	
		noise_data(MU_V_ETA) = 0.0;
		noise_data(SIGMA_V_ETA) = 1.0;
		noise_data(VAR_V_ETA) = noise_data(SIGMA_V_ETA)*noise_data(SIGMA_V_ETA);
		
		noise_data(MU_V_OMEGA) = 0.0;
		noise_data(SIGMA_V_OMEGA) = 0.1;
		noise_data(VAR_V_OMEGA) = noise_data(SIGMA_V_OMEGA)*noise_data(SIGMA_V_OMEGA);
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
	ros::init(argc, argv, "kf_node");
	SubscribeAndPublish SAPkf;
	
	ros::spin();
	
	return 0;
}



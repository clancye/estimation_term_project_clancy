#include "../../include/simulate.h"


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "simulate_node");
	ros::NodeHandle n;
	ros::Publisher filter_target_pub = n.advertise<laser_package::state>("/filter_topic",1000); //publish targets to new topic
	//ros::ServiceClient client_update = n.serviceClient<laser_package::update_tracker>("updateFilter");
	//ros::ServiceClient client_initialize = n.serviceClient<laser_package::update_tracker>("initializeFilter");
	laser_package::update_tracker srv;
	laser_package::state state_msg;
	ros::Rate r(floor(1/SAMPLING_INTERVAL+0.5));
	
	//random noise stuff
	std::default_random_engine measurement_generator;
	std::normal_distribution<double> meas_noise(MU_W_SIMULATE,VAR_W_SIMULATE);
	
	//simulator class
	Simulator simulator = Simulator();
	
	//other variables
	Eigen::VectorXd next_x(5), past_x(5);
	double w;
	past_x << 0, 10, 0, 0, OMEGA_RADS; // initial vector: xi, xi_dot, eta, eta_dot, omega
	
	
	simulator.initializeSimulators(past_x);
	/*srv.request.initial_x = past_x(XI);
	srv.request.initial_x_velocity = past_x(XI_DOT);
	srv.request.initial_y = past_x(ETA);
	srv.request.initial_y_velocity = past_x(ETA_DOT);
	srv.request.initial_turn_rate = past_x(OMEGA);
	srv.request.sampling_interval = SAMPLING_INTERVAL;
	srv.request.update_time = ros::Time::now().toSec();
	srv.request.measurement_noise_mean = MU_W;
	srv.request.measurement_noise_variance = VAR_W;*/
	//initialize tracker(s)

	
	int counter = 0;
	
	//client_initialize.call(srv);
	
	while(ros::ok())
	{
		w = meas_noise(measurement_generator);//noises
		if(counter<300)
		{
			//next_x = simulator.simulateCoordinatedTurn(OMEGA_RADS);	
			next_x = simulator.simulateUniformMotion();
		}
		else if (counter>300&counter<500)
		{
			//next_x = simulator.simulateUniformMotion();
			next_x = simulator.simulateCoordinatedTurn(OMEGA_RADS);	
		}
		else if (counter>500&&counter<1100)
		{
			next_x = simulator.simulateUniformMotion();
			//next_x = simulator.simulateCoordinatedTurn(OMEGA_RADS);	
		}
		else
		{
			next_x = simulator.simulateUniformMotion();
			//next_x = simulator.simulateCoordinatedTurn(OMEGA_RADS);	
		}
		//update values
		state_msg.Real_X = next_x(XI);
		state_msg.Measured_X = next_x(XI) + w;
		state_msg.Real_Y = next_x(ETA);
		state_msg.Measured_Y = next_x(ETA) + w;
		state_msg.Time_Of_Measurement = ros::Time::now().toSec();
		filter_target_pub.publish(state_msg);
		counter++;
		//send update
		//if(client_update.call(srv));
		
		ros::spinOnce();
		
		r.sleep();
	}
	
	
	
	

	return 0;
}



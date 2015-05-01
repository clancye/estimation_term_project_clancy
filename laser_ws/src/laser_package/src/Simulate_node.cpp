#include "../include/Simulate.h"


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "Simulate_node");
	ros::NodeHandle n;
	ros::Publisher filter_target_pub = n.advertise<laser_package::state>("/filter_topic",1000); //publishes target data for filter
	laser_package::state state_msg;
	ros::Rate r(floor(1/SIMULATE_RATE+0.5));//speed up the simulations
	
	
	//white noise engine and distributions
	std::default_random_engine measurement_generator;
	std::normal_distribution<double> measurement_noise_xi(MU_W_SIMULATE_XI,SIGMA_W_SIMULATE_XI);
	std::normal_distribution<double> measurement_noise_eta(MU_W_SIMULATE_ETA,SIGMA_W_SIMULATE_ETA);
	std::normal_distribution<double> measurement_noise_rho(MU_W_SIMULATE_RHO,SIGMA_W_SIMULATE_RHO);
	std::normal_distribution<double> measurement_noise_theta(MU_W_SIMULATE_THETA,SIGMA_W_SIMULATE_THETA);
	
	//instantiate a simulator
	Simulator simulator = Simulator();
	
	//other variables
	state_vector next_x, past_x;
	double w_xi, w_eta, w_rho, w_theta, rho, theta;
	past_x << 0, 0, 0, 250, 0; // initial vector: xi, xi_dot, eta, eta_dot, omega
	int counter = 0;
	int stop_publishing = 0;
	
	simulator.setOmega(0);
	simulator.initializeSimulators(past_x);
	
	while(ros::ok())
	{
		w_xi = measurement_noise_xi(measurement_generator);//noises
		w_eta = measurement_noise_eta(measurement_generator);//noises
		w_rho = measurement_noise_rho(measurement_generator);//noises
		w_theta = measurement_noise_theta(measurement_generator);//noises
		
		//ROS_INFO("\n w_xi = %f \n w_eta = %f \n w_rho = %f \n w_theta = %f", w_xi, w_eta, w_rho, w_theta);
		if(counter<10)
		{
			next_x = simulator.simulateUniformMotion();
			//simulator.setOmega(DEGREES);
			//next_x = simulator.simulateCoordinatedTurn();	
			}
		else if (counter>=10&counter<12)
		{
			//next_x = simulator.simulateUniformMotion();
			simulator.setOmega(2);
			next_x = simulator.simulateCoordinatedTurn();
		}
		else if (counter>=12&&counter<19)
		{
			next_x = simulator.simulateUniformMotion();
			//simulator.setOmega(DEGREES);
			//next_x = simulator.simulateCoordinatedTurn();
		}
		else if (counter>=19&&counter<24)
		{
			//next_x = simulator.simulateUniformMotion();
			simulator.setOmega(-1);
			next_x = simulator.simulateCoordinatedTurn();	
		}
		else if (counter>=24&&counter<32)
		{
			//next_x = simulator.simulateUniformMotion();
			simulator.setOmega(1);
			next_x = simulator.simulateCoordinatedTurn();	
		}
		else if (counter>=32&&counter<36)
		{
			//next_x = simulator.simulateUniformMotion();
			simulator.setOmega(-1);
			next_x = simulator.simulateCoordinatedTurn();	
		}
		else if (counter>=36&&counter<50)
		{
			next_x = simulator.simulateUniformMotion();
			//simulator.setOmega(-1);
			//next_x = simulator.simulateCoordinatedTurn();	
		}
		else
		{
				stop_publishing = true;
		}
		
		//convert to polar form
		rho = sqrt(pow((next_x(XI_INDEX)-SENSOR_XI),2) + pow((next_x(ETA_INDEX)-SENSOR_ETA),2));
		theta = atan2(next_x(ETA_INDEX)-SENSOR_ETA, next_x(XI_INDEX)-SENSOR_XI) + SENSOR_THETA;
		
		//update cartesian values and measurements
		state_msg.Real_X = next_x(XI_INDEX);
		state_msg.Measured_X = next_x(XI_INDEX) + w_xi;
		state_msg.Real_Y = next_x(ETA_INDEX);
		state_msg.Measured_Y = next_x(ETA_INDEX) + w_eta;
		//update polar values and measurements
		state_msg.Real_Rho = rho;
		state_msg.Real_Theta = theta;
		state_msg.Measured_Rho = rho + w_rho;
		state_msg.Measured_Theta = theta + w_theta;
		
		if(counter==0)state_msg.Time_Of_Measurement = ros::Time::now().toSec();
		else{state_msg.Time_Of_Measurement += SAMPLING_INTERVAL*counter;}
		if(!stop_publishing)filter_target_pub.publish(state_msg);
		counter++;
		
		ros::spinOnce();
		
		r.sleep();
	}
	return 0;
}



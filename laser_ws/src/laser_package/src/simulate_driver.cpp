#include "../../include/simulate.h"

Simulator::Simulator()
{
	ROS_INFO("Empty simulator");
}

void Simulator::initializeSimulators(Eigen::MatrixXd an_initial_state)
{
	T = SAMPLING_INTERVAL;
	x.push_back(Eigen::VectorXd::Zero(5));
	x.push_back(an_initial_state);
	Gamma.resize(5,3);
	F_CT.resize(5,5);
	F_UM.resize(5,5);
	
	
	F_UM <<
	1, T, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, T, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 0;
	
	omega = OMEGA_RADS; //multiplication is faster than division
	inverse_omega = 1/omega;
	
	F_CT <<
	1, sin(omega*T)*inverse_omega, 0, (1-cos(omega*T))*inverse_omega, 0,
	0, cos(omega*T), 0, -sin(omega*T), 0,
	0, (1-cos(omega*T))*inverse_omega, 1, sin(omega*T)*inverse_omega, 0,
	0, sin(omega*T), 0, cos(omega*T), 0,
	0, 0, 0, 0, 1;
	
	Gamma <<
	0.5*T*T, 0, 0,
	T, 0, 0,
	0, 0.5*T*T, 0,
	0, T, 0,
	0, 0, T;
}

Eigen::MatrixXd Simulator::simulateUniformMotion()
{
	std::normal_distribution<double> process_noise_xi(MU_XI, VAR_XI);
	std::normal_distribution<double> process_noise_eta(MU_ETA, VAR_ETA);
	std::normal_distribution<double> process_noise_omega(MU_OMEGA, VAR_OMEGA);
	
	v(0) = process_noise_xi(generator);
	v(1) = process_noise_eta(generator);
	v(2) = process_noise_omega(generator);
	x[0] = F_UM*x[1] + Gamma*v;
	
	x[1] = x[0];
	
	ROS_INFO("Straight");
	
	return x[0];
}

Eigen::MatrixXd Simulator::simulateCoordinatedTurn(double a_turn_rate)
{

	std::normal_distribution<double> process_noise_xi(MU_XI, VAR_XI);
	std::normal_distribution<double> process_noise_eta(MU_ETA, VAR_ETA);
	std::normal_distribution<double> process_noise_omega(MU_OMEGA, VAR_OMEGA);
	
	v(1) = process_noise_eta(generator);
	v(0) = process_noise_xi(generator);
	v(2) = process_noise_omega(generator);
	
	x[0] = F_CT*x[1] + Gamma*v;
	x[1] = x[0];
	
	ROS_INFO("Turning!");
	
	return x[0];
}

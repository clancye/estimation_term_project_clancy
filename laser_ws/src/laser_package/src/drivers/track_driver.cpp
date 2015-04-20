#include "../../include/track.h"

Tracker::Tracker(
Eigen::RowVector4f some_noise_data, 
int a_system_model, 
Eigen::MatrixXf a_measurement_matrix, 
Eigen::MatrixXf an_initial_covariance, 
Eigen::MatrixXf an_initial_state)
{
	initializeNoises(some_noise_data);
	system_model = a_system_model;// this will be used in calculations to determine the matrices
	H = a_measurement_matrix;
	P = an_initial_covariance;
	x_hat.push_back(an_initial_state);
	//ROS_INFO("mu_v = %f\n sigma_v = %f\n mu_w = %f, sigma_w = %f", mu_v, sigma_v, mu_w, sigma_w);
	//ROS_INFO("P11 = %f, P12 = %f, P21 = %f, P22 = %f", P(0,0),P(0,1),P(1,0),P(1,1));
	//ROS_INFO("X_initial = [%f; %f] \n", x_hat[0](0), x_hat[0](1));
}

void Tracker::initializeNoises(Eigen::RowVector4f noise_data)
{
	mu_v = noise_data(0);
	sigma_v = noise_data(1);
	mu_w = noise_data(2);
	sigma_w = noise_data(3);
}

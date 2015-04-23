#include "../../include/track.h"

Tracker::Tracker()
{
	ROS_INFO("Empty tracker");
}

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
	ROS_INFO("mu_v = %f\n sigma_v = %f\n mu_w = %f, sigma_w = %f", mu_v, sigma_v, mu_w, sigma_w);
	ROS_INFO("P11 = %f, P12 = %f, P21 = %f, P22 = %f", P(0,0),P(0,1),P(1,0),P(1,1));
	ROS_INFO("X_initial = [%f; %f] \n", x_hat[0](0), x_hat[0](1));
}

void Tracker::initializeNoises(Eigen::RowVector4f noise_data)
{
	mu_v = noise_data(0);
	sigma_v = noise_data(1);
	mu_w = noise_data(2);
	sigma_w = noise_data(3);
}

void Tracker::update(float x, float y, double an_update_time)
{
	updateSpeed(x,y, an_update_time);
	//ROS_INFO("Updating tracker");
}

void Tracker::updateSpeed(float some_x, float some_y, double time_of_measurement)
{
	float speed_temp_sum = 0.0;
	for(int i = MEASUREMENT_MEMORY-1;i>0;i--)
	{
		speed_temp_sum += speed_memory[i];
		speed_memory[i] = speed_memory[i-1];
	}
	float distanceBetweenMeasurements = getDistance(some_x, some_y, last_x, last_y);
	float timeBetweenMeasurements = time_of_measurement - last_time;
	float current_instantaneous_speed = distanceBetweenMeasurements/timeBetweenMeasurements;
	speed_memory[0] = current_instantaneous_speed;
	speed_temp_sum += current_instantaneous_speed;
	current_speed = speed_temp_sum/MEASUREMENT_MEMORY;
	last_x = some_x;
	last_y = some_y;
	last_time = time_of_measurement;
	ROS_INFO("current speed = %f\n TIME BETWEEN = %f \n, DISTANCE BETWEEN = %f", current_speed, timeBetweenMeasurements, distanceBetweenMeasurements);
}

float Tracker::getDistance(float x_1, float y_1, float x_2, float y_2)
{
	return sqrt(pow((x_1-x_2),2)+pow((y_1-y_2),2));
}



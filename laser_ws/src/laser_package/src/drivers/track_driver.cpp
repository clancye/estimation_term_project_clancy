#include "../../include/track.h"

Tracker::Tracker()
{
	ROS_INFO("Empty tracker");
}

Tracker::Tracker(
Eigen::RowVector4f some_noise_data, 
int a_system_model, 
Eigen::MatrixXf an_initial_state)
{
	system_model = a_system_model;
	initializeNoises(some_noise_data);
	initializeStateModel();
	// this will be used in calculations to determine the matrices
	last_time = 0.0;// need to fix for initialization
	second_last_time = 0.0;
	x_hat_vec.push_back(an_initial_state);
	//ROS_INFO("mu_v = %f\n sigma_v = %f\n mu_w = %f, sigma_w = %f", mu_v, sigma_v, mu_w, sigma_w);
	//ROS_INFO("P11 = %f, ", P(0,0));//P12 = %f, P21 = %f, P22 = %f", P(0,0),P(0,1),P(1,0),P(1,1));
	//ROS_INFO("X_initial = [%f; %f] \n", x_hat[0](0), x_hat[0](1));
}

void Tracker::initializeNoises(Eigen::RowVector4f noise_data)
{
	mu_v = noise_data(0);
	sigma_v = noise_data(1);
	mu_w = noise_data(2);
	sigma_w = noise_data(3);
}

void Tracker::initializeStateModel()
{
	if(system_model == DWNA_X_AND_Y)
	{
		float sigma_v_squared = pow(sigma_v, 2);
		float sigma_w_squared = pow(sigma_w, 2);
		F.resize(4,4);
		Gamma.resize(4,1);
		Q.resize(4,4);
		R.resize(4,4);
		F << 1, SAMPLE_TIME, 0,0,0,1,0,0,0,0,1,SAMPLE_TIME,0,0,0,1;
		Gamma << (1/2)*pow(SAMPLE_TIME,2), SAMPLE_TIME, (1/2)*pow(SAMPLE_TIME,2), SAMPLE_TIME;
		Q = Gamma*sigma_v_squared*Gamma.transpose();
		R << sigma_v_squared, 0, 0, 0, 0, sigma_v_squared, 0, 0, 0, 0, sigma_v_squared, 0, 0, 0, 0,sigma_v_squared;
	}
	else if(system_model == DWNA_X)
	{
		float sigma_v_squared = pow(sigma_v, 2);
		float sigma_w_squared = pow(sigma_w, 2);
		H.resize(1,2);
		P.resize(2,2);
		P_bar.resize(2,2);
		x_hat_bar.resize(2,1);
		S.resize(2,2);
		W.resize(2,1);
		x_hat.resize(2,1);
		F.resize(2,2);
		Gamma.resize(2,1);
		Q.resize(2,2);
		R.resize(1,1);
		R << sigma_w_squared;
		P << R, R/SAMPLE_TIME, R/SAMPLE_TIME, 2*R/(SAMPLE_TIME*SAMPLE_TIME); //initial covariance from 2-point differencing
		H << 1,0;
		F << 1, SAMPLE_TIME, 0,1;
		Gamma << (0.5*SAMPLE_TIME*SAMPLE_TIME), (SAMPLE_TIME);
		Q = Gamma*sigma_v_squared*Gamma.transpose();
		ROS_INFO("\nGamma11 = %f \n F12 = %f \n Gamma21 = %f \n Q22 = %f ", R(0,0), R(0,1),R(1,0),R(1,1)); //use to test if values are correct
	}
}

void Tracker::updateXY(float x, float y, double an_update_time)
{
	updateSpeedXY(x,y, an_update_time);
	//ROS_INFO("Updating tracker");
}

void Tracker::updateX(float x,  double an_update_time)
{
	updateSpeedX(x, an_update_time);
	//ROS_INFO("Updating tracker");
}

void Tracker::updateSpeedXY(float some_x, float some_y, double time_of_measurement)
{
	//float speed_temp_sum = 0.0;
	float velocity_x_temp_sum = 0.0;
	float velocity_y_temp_sum = 0.0;
	float accel_x_temp_sum = 0.0;
	float accel_y_temp_sum = 0.0;
	for(int i = MEASUREMENT_MEMORY-1;i>0;i--)
	{
		//speed_temp_sum += speed_memory[i];
		//speed_memory[i] = speed_memory[i-1];
		velocity_x_temp_sum += MEMORY_COEFFICIENT*velocity_x_memory[i];
		velocity_y_temp_sum += MEMORY_COEFFICIENT*velocity_y_memory[i];
		velocity_x_memory[i] = velocity_x_memory[i-1];
		velocity_y_memory[i] = velocity_y_memory[i-1];
		
		accel_x_temp_sum += MEMORY_COEFFICIENT*accel_x_memory[i];
		accel_y_temp_sum += MEMORY_COEFFICIENT*accel_y_memory[i];
		accel_x_memory[i] = accel_x_memory[i-1];
		accel_y_memory[i] = accel_y_memory[i-1];
	}
	//float distanceBetweenMeasurements = getDistance(some_x, some_y, last_x, last_y);
	float difference_of_x = some_x - last_x;
	float difference_of_y = some_y - last_y;
	float time_between_measurements = time_of_measurement - last_time;
	float time_between_velocity_measurements = time_of_measurement - second_last_time;
	float instantaneous_velocity_x = difference_of_x/time_between_measurements;
	float instantaneous_velocity_y = difference_of_y/time_between_measurements;
	//float current_instantaneous_speed = distanceBetweenMeasurements/timeBetweenMeasurements;
	//speed_memory[0] = current_instantaneous_speed;
	//speed_temp_sum += current_instantaneous_speed;
	velocity_x_memory[0] = instantaneous_velocity_x;
	velocity_y_memory[0] = instantaneous_velocity_y;
	accel_x_memory[0] = (velocity_x_memory[0]-velocity_x_memory[1])/time_between_velocity_measurements;
	accel_y_memory[0] = (velocity_y_memory[0]-velocity_y_memory[1])/time_between_velocity_measurements;
	
	
	velocity_x_temp_sum += instantaneous_velocity_x;
	velocity_y_temp_sum += instantaneous_velocity_y;
	accel_x_temp_sum += accel_x_memory[0];
	accel_y_temp_sum += accel_y_memory[0];
	
	current_x_velocity = velocity_x_temp_sum/MEASUREMENT_MEMORY;
	current_y_velocity = velocity_y_temp_sum/MEASUREMENT_MEMORY;
	current_x_accel = accel_x_temp_sum/MEASUREMENT_MEMORY;
	current_y_accel = accel_y_temp_sum/MEASUREMENT_MEMORY;
	//current_speed = speed_temp_sum/MEASUREMENT_MEMORY;
	//max_current_speed = (current_speed-max_current_speed>0)? current_speed : max_current_speed; //David current victory (5.039412 m/s)
	last_x = some_x;
	last_y = some_y;
	second_last_time = last_time;
	last_time = time_of_measurement;
	ROS_INFO("X_accel = %f\n Y_accel = %f\n time between = %f\n\n-----------------------------------------------------", current_x_accel, current_y_accel, time_between_measurements);
}

void Tracker::updateSpeedX(float some_x, double time_of_measurement)
{
	//float speed_temp_sum = 0.0;
	float velocity_x_temp_sum = 0.0;
	float accel_x_temp_sum = 0.0;
	for(int i = MEASUREMENT_MEMORY-1;i>0;i--)
	{
		velocity_x_temp_sum += MEMORY_COEFFICIENT*velocity_x_memory[i];
		velocity_x_memory[i] = velocity_x_memory[i-1];
		
		accel_x_temp_sum += MEMORY_COEFFICIENT*accel_x_memory[i];
		accel_x_memory[i] = accel_x_memory[i-1];
	}
	float difference_of_x = some_x - last_x;
	float time_between_measurements = time_of_measurement - last_time;
	float time_between_velocity_measurements = time_of_measurement - second_last_time;
	float instantaneous_velocity_x = difference_of_x/time_between_measurements;
	velocity_x_memory[0] = instantaneous_velocity_x;
	accel_x_memory[0] = (velocity_x_memory[0]-velocity_x_memory[1])/time_between_velocity_measurements;
	
	velocity_x_temp_sum += instantaneous_velocity_x;
	accel_x_temp_sum += accel_x_memory[0];
	
	current_x_velocity = velocity_x_temp_sum/MEASUREMENT_MEMORY;
	current_x_accel = accel_x_temp_sum/MEASUREMENT_MEMORY;
	last_x = some_x;
	second_last_time = last_time;
	last_time = time_of_measurement;
	ROS_INFO("X_accel = %f\n time between = %f\n\n-----------------------------------------------------", current_x_accel, time_between_measurements);
}

float Tracker::getDistance(float x_1, float y_1, float x_2, float y_2)
{
	return sqrt(pow((x_1-x_2),2)+pow((y_1-y_2),2));
}

float Tracker::getXAcceleration()
{
	return current_x_accel;
}

float Tracker::getYAcceleration()
{
	return current_y_accel;
}

float Tracker::getXVelocity()
{
	return current_x_velocity;
}

float Tracker::getYVelocity()
{
	return current_y_velocity;
}




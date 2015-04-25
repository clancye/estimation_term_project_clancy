#include "../../include/track.h"

Tracker::Tracker()
{
	ROS_INFO("Empty tracker");
}

Tracker::Tracker(Eigen::MatrixXf an_initial_state)
{
	temp_initial_state = an_initial_state;
	
	initializeStateModel();
	
	last_time = 0.0;// need to fix for initialization
	second_last_time = 0.0;
	
	//printValues();//USE TO DEBUG VALUES IN MATRICES
}

void Tracker::initializeStateModel()
{		
		resizeMatrices(); // resizes all the matrices for a 2-vector measurement
		initializeMatrices();
		updateStateCovariance();
		
		x_hat << temp_initial_state(0,0), temp_initial_state(1,0), temp_initial_state(2,0), temp_initial_state(3,0);	
		x_hat_vec.push_back(x_hat);
		x_hat_bar = F*x_hat;
		z_hat = H*x_hat_bar;
}

void Tracker::update(Eigen::Vector2f z, double an_update_time)
{
	updateDerivatives(z, an_update_time);
	ROS_INFO("HERE?");
	
	nu = z-z_hat;
	
	x_hat = x_hat_bar + W*nu;
	
	updateStateCovariance();
	
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
	//ROS_INFO("Updating tracker");
}


void Tracker::updateDerivatives(Eigen::Vector2f z, double time_of_measurement)
{
	//float speed_temp_sum = 0.0;
	float some_x = z(0);
	float some_y = z(1);
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

void Tracker::printValues()
{
	//ROS_INFO("mu_v = %f\n sigma_v = %f\n mu_w = %f, sigma_w = %f", mu_v, sigma_v, mu_w, sigma_w);
	//ROS_INFO("P11 = %f, ", P(0,0));//P12 = %f, P21 = %f, P22 = %f", P(0,0),P(0,1),P(1,0),P(1,1));
	//ROS_INFO("X_initial = [%f; %f] \n", x_hat[0](0), x_hat[0](1));
}

float Tracker::getPredictedX()
{
	return x_hat(XI);
}
float Tracker::getPredictedY()
{
	return x_hat(ETA);
}
float Tracker::getPredictedXVel()
{
	return x_hat(XI_DOT);
}
float Tracker::getPredictedYVel()
{
	return x_hat(ETA_DOT);
}

float Tracker::getPositionVarianceX()
{
	return P(0,0);
}

float Tracker::getVelocityVarianceX()
{
	return P(1,1);
}

float Tracker::getPositionGainX()
{
	return W(0,0);
}

float Tracker::getVelocityGainX()
{
	return W(1,0);
}

float Tracker::getInnovationX()
{
	return nu(0,0);
}

float Tracker::getPredictedMeasurementX()
{
	return z_hat(0);
}

float Tracker::getPositionVarianceY()
{
	return P(0,0);
}

float Tracker::getVelocityVarianceY()
{
	return P(1,1);
}

float Tracker::getPositionGainY()
{
	return W(2,1);
}

float Tracker::getVelocityGainY()
{
	return W(3,1);
}

float Tracker::getInnovationY()
{
	return nu(1);
}

float Tracker::getPredictedMeasurementY()
{
	return z_hat(2);
}

void Tracker::resizeMatrices()
{
		H_temp.resize(1,2);
		H.resize(2,4);
		x_hat.resize(4,1);
		z_hat.resize(2,1);
		P.resize(2,2);
		P_bar.resize(2,2);
		Q.resize(2,2);
		R.resize(1,1);
		Gamma.resize(2,1);
		nu.resize(2,1);
		W_temp.resize(2,1);
		W.resize(4,2);
		F_temp.resize(2,2);
		F.resize(4,4);
}

void Tracker::initializeMatrices()
{
	R << VAR_W;
	P << R, R/T, R/T, 2*R/(T*T);
	H_temp << 1, 0;
	H << H_temp, Eigen::MatrixXf::Zero(1,2),Eigen::MatrixXf::Zero(1,2),H_temp;
	F_temp << 1, T, 0,1; 
	F << F_temp, Eigen::MatrixXf::Zero(2,2), Eigen::MatrixXf::Zero(2,2), F_temp;	
	Gamma << 0.5*T*T , T;
	Q = Gamma*VAR_V*Gamma.transpose();
}
 
void Tracker::updateStateCovariance()
{
	//P_bar
	P_bar = F_temp*P*F_temp.transpose() + Q;
	//S
	S = H_temp*P_bar*H_temp.transpose() + R;
	//W
	W_temp = P_bar*H_temp.transpose()*S.inverse();
	W << W_temp, Eigen::MatrixXf::Zero(2,1), Eigen::MatrixXf::Zero(2,1), W_temp; 
	//P
	P = P_bar - W_temp*S*W_temp.transpose();
}



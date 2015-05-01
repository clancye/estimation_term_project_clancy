#include "../include/Filter.h"

Filter::Filter()
{
	//ROS_INFO("Empty Filter. \n\n Only prediction: disappointment.");
}

void Filter::initializeNoises(initial_noise_vector noise_data)
{//CHECKED---GOOD
	mu_w_xi = noise_data(MU_W_XI_INDEX);
	sigma_w_xi = noise_data(SIGMA_W_XI_INDEX);
	var_w_xi = noise_data(VAR_W_XI_INDEX);
	
	mu_w_eta = noise_data(MU_W_ETA_INDEX);
	sigma_w_eta = noise_data(SIGMA_W_ETA_INDEX);
	var_w_eta = noise_data(VAR_W_ETA_INDEX);
	
	mu_v_xi = noise_data(MU_V_XI_INDEX);
	sigma_v_xi = noise_data(SIGMA_V_XI_INDEX);
	var_v_xi = noise_data(VAR_V_XI_INDEX);
	
	mu_v_eta = noise_data(MU_V_ETA_INDEX);
	sigma_v_eta = noise_data(SIGMA_V_ETA_INDEX);
	var_v_eta = noise_data(VAR_V_ETA_INDEX);
	
	mu_v_omega = noise_data(MU_V_OMEGA_INDEX);
	sigma_v_omega = noise_data(SIGMA_V_OMEGA_INDEX);
	var_v_omega = noise_data(VAR_V_OMEGA_INDEX);
	
	sigma_w_rho = noise_data(SIGMA_W_RHO_INDEX);
	sigma_w_theta = noise_data(SIGMA_W_THETA_INDEX);
	
	validity_constant = sigma_w_theta*sigma_w_theta/sigma_w_rho;
	
	bias = exp(-sigma_w_theta*sigma_w_theta/2);
}

void Filter::initializeMatrices()
{//CHECKED---GOOD
	updateBiasing();
	last_time = 0.0;
	second_last_time = 0.0;
	Gamma <<
	0.5*T*T, 0, 0,
	T, 0, 0,
	0, 0.5*T*T, 0,
	0, T, 0,
	0, 0, T;
	
	V <<
	var_v_xi, 0, 0,
	0, var_v_eta, 0,
	0, 0, var_v_omega;
	
	Q = Gamma*V*Gamma.transpose();
	
	H << 
	1, 0, 0, 0, 0,
	0, 0, 1, 0, 0;
	
	P <<//using the two-point differencing convention from page 
	R(0,0),R(0,0)/T,0,0,0,
	R(0,0)/T, 2*R(0,0)/(T*T),0,0,0,
	0,0,R(1,1),R(1,1)/T,0,
	0,0,R(1,1)/T,2*R(1,1)/(T*T),0,
	0,0,0,0,R(0,0)/T;//this last element is (hopefully) arbitrary since we aren't measuring omega
}


void Filter::updateFilter(measurement_vector some_z, double an_update_time, state_vector some_real_state)
{//CHECKED FOR KALMAN---GOOD FOR KALMAN---NOT CHECKED FOR EXTENDED KALMAN---
	estimation_error = real_state - x_hat;
	z_polar = some_z;
	updateBiasing();
	updateDerivatives(z_cartesian, an_update_time);
	nu = z_cartesian-z_hat;
	calculateLikelihood();
	
	x_hat = x_hat_bar + W*nu;
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
}

void Filter::updateBiasing()
{//CHECKED---GOOD
	double final_validity_constant = z_polar(RHO_INDEX)*validity_constant;
	if(final_validity_constant>0.4)//unbiasing conversion to cartesian noise matrix and state vector
	{  //CHECKED---GOOD
		R(0,0) = ((1/(bias*bias))-2)*(z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*cos(z_polar(THETA_INDEX))*cos(z_polar(THETA_INDEX))) + ((z_polar(RHO_INDEX)*z_polar(RHO_INDEX)+sigma_w_rho*sigma_w_rho)*0.5*(1+bias*bias*bias*bias*cos(2*z_polar(THETA_INDEX))));
		R(1,1) = ((1/(bias*bias))-2)*(z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*sin(z_polar(THETA_INDEX))*sin(z_polar(THETA_INDEX))) + ((z_polar(RHO_INDEX)*z_polar(RHO_INDEX)+sigma_w_rho*sigma_w_rho)*0.5*(1-bias*bias*bias*bias*cos(2*z_polar(THETA_INDEX))));
		R(0,1) = (((1/(bias*bias))*z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*0.5)+(z_polar(RHO_INDEX)*z_polar(RHO_INDEX)+sigma_w_rho*sigma_w_rho)*bias*bias*bias*bias*0.5 - z_polar(RHO_INDEX)*z_polar(RHO_INDEX))*sin(2*z_polar(THETA_INDEX));
		R(1,0) = R(0,1);
		//convert to cartesian
		z_cartesian(0) = (z_polar(RHO_INDEX)*cos(z_polar(THETA_INDEX))/bias)+SENSOR_XI;
		z_cartesian(1) = (z_polar(RHO_INDEX)*sin(z_polar(THETA_INDEX))/bias)+SENSOR_ETA;
	}
	else//regular conversion to cartesian noise matrix and state vector
	{   //CHECKED---GOOD
		R(0,0) = z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*sigma_w_theta*sigma_w_theta*sin(z_polar(THETA_INDEX))*sin(z_polar(THETA_INDEX))+sigma_w_rho*sigma_w_rho*cos(z_polar(THETA_INDEX))*cos(z_polar(THETA_INDEX));
		R(1,1) = z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*sigma_w_theta*sigma_w_theta*cos(z_polar(THETA_INDEX))*cos(z_polar(THETA_INDEX))+sigma_w_rho*sigma_w_rho*sin(z_polar(THETA_INDEX))*sin(z_polar(THETA_INDEX));
		R(1,0) = (sigma_w_rho*sigma_w_rho-z_polar(RHO_INDEX)*z_polar(RHO_INDEX)*sigma_w_theta*sigma_w_theta)*sin(z_polar(THETA_INDEX))*cos(z_polar(THETA_INDEX));
		R(0,1) = R(1,0);
		//convert to cartestian
		z_cartesian(0) = (z_polar(RHO_INDEX)*cos(z_polar(THETA_INDEX)))+SENSOR_XI;
		z_cartesian(1) = (z_polar(RHO_INDEX)*sin(z_polar(THETA_INDEX)))+SENSOR_ETA;
	}	
	
	ROS_INFO("final_validity_constant = %f, bias = %f\n R(0,0) = %f\n R(0,1) = %f, R(1,0) = %f, R(1,1) = %f", final_validity_constant, bias,R(0,0), R(0,1), R(1,0), R(1,1));
}


void Filter::updateDerivatives(measurement_vector some_z, double time_of_measurement)
{
	//double speed_temp_sum = 0.0;
	double some_x = some_z(0);
	double some_y = some_z(1);
	double velocity_x_temp_sum = 0.0;
	double velocity_y_temp_sum = 0.0;
	double accel_x_temp_sum = 0.0;
	double accel_y_temp_sum = 0.0;
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
	//double distanceBetweenMeasurements = getDistance(some_x, some_y, last_x, last_y);
	double difference_of_x = some_x - last_x;
	double difference_of_y = some_y - last_y;
	double time_between_measurements = time_of_measurement - last_time;
	double time_between_velocity_measurements = time_of_measurement - second_last_time;
	double instantaneous_velocity_x = difference_of_x/time_between_measurements;
	double instantaneous_velocity_y = difference_of_y/time_between_measurements;
	//double current_instantaneous_speed = distanceBetweenMeasurements/timeBetweenMeasurements;
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
	//ROS_INFO(" time between = %f\n\n-----------------------------------------------------",time_between_measurements);
}



double Filter::getDistance(double x_1, double y_1, double x_2, double y_2)
{
	return sqrt(pow((x_1-x_2),2)+pow((y_1-y_2),2));
}

double Filter::getXAcceleration()
{
	return current_x_accel;
}

double Filter::getYAcceleration()
{
	return current_y_accel;
}

double Filter::getXVelocity()
{
	return current_x_velocity;
}

double Filter::getYVelocity()
{
	return current_y_velocity;
}

void Filter::printValues()
{
	//ROS_INFO("mu_v = %f\n sigma_v = %f\n mu_w = %f, sigma_w = %f", mu_v, sigma_v, mu_w, sigma_w);
	//ROS_INFO("P11 = %f, ", P(0,0));//P12 = %f, P21 = %f, P22 = %f", P(0,0),P(0,1),P(1,0),P(1,1));
	//ROS_INFO("X_initial = [%f; %f] \n", x_hat[0](0), x_hat[0](1));
}

double Filter::getEstimatedX()
{
	return x_hat(XI_INDEX);
}

double Filter::getEstimatedY()
{
	return x_hat(ETA_INDEX);
}

double Filter::getEstimatedXSpeed()
{
	return x_hat(XI_DOT_INDEX);
}

double Filter::getEstimatedYSpeed()
{
	return x_hat(ETA_DOT_INDEX);
}

double Filter::getEstimatedOmega()
{
	return x_hat(OMEGA_INDEX);
}

double Filter::getPositionVarianceX()
{
	return P(XI_INDEX,XI_INDEX);
}

double Filter::getPositionVarianceY()
{
	return P(ETA_INDEX,ETA_INDEX);
}

double Filter::getVelocityVarianceX()
{
	return P(XI_DOT_INDEX,XI_DOT_INDEX);
}

double Filter::getVelocityVarianceY()
{
	return P(ETA_DOT_INDEX,ETA_DOT_INDEX);
}

double Filter::getOmegaVariance()
{
	return P(OMEGA_INDEX, OMEGA_INDEX);
}

double Filter::getPositionGainX()
{
	return W(XI_INDEX,XI_INDEX);
}

double Filter::getPositionGainY()
{
	return W(2,1);
}

double Filter::getVelocityGainX()
{
	return W(1,0);
}

double Filter::getVelocityGainY()
{
	return W(3,1);
}

double Filter::getInnovationX()
{
	return nu(0);
}

double Filter::getInnovationY()
{
	return nu(1);
}

double Filter::getEstimatedMeasurementX()
{
	return z_hat(0);
}

double Filter::getEstimatedMeasurementY()
{
	return z_hat(1);
}

state_vector Filter::getStateEstimate()
{
	return x_hat;
}

covariance_matrix Filter::getCovariance()
{
	return P;
}

void Filter::calculateLikelihood()
{
	innovation_covariance_matrix temp_matrix;
	double exponent;
	temp_matrix = 2*PI*S;
	exponent = (nu.transpose())*S.inverse()*(nu);
	//ROS_INFO("exponent = %f\n S = [%f, %f; %f, %f]", exponent,S(0,0), S(0,1), S(1,0), S(1,1));
	Lambda = (1/sqrt(temp_matrix.determinant()))*exp(-0.5*(exponent));
}

double Filter::getLikelihood()
{
	//ROS_INFO("Likelihood = %f", Lambda);
	return Lambda;
}

double Filter::getNEES()
{
	double nees = estimation_error.transpose()*P.inverse()*estimation_error;
	return nees;
}

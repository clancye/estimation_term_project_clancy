#include "../../include/track.h"

Tracker::Tracker()
{
	ROS_INFO("Empty tracker");
}

Tracker::Tracker(Eigen::MatrixXf an_initial_state, float a_sampling_interval, Eigen::MatrixXf noise_data, int CT_model)
{
	T = a_sampling_interval;
	
	initializeNoises(noise_data);
	
	resizeMatrices(); 
	initializeMatrices();
	
		
	x_hat << an_initial_state(XI,0), an_initial_state(XI_DOT,0), an_initial_state(ETA,0), an_initial_state(ETA_DOT,0),an_initial_state(OMEGA,0);	
	
	
	if(!CT_model)
	{
		initializeUniformMotionSystemMatrix();
		updateUMStateCovariance();
	}
	else
	{
		initializeCoordinatedTurnSystemMatrix();
		updateCoordinatedTurnJacobian();
		updateCTStateCovariance();
	}
	
	
	
	last_time = 0.0;// need to fix for initialization
	second_last_time = 0.0;
	x_hat_vec.push_back(x_hat);
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
	//printValues();//USE TO DEBUG VALUES IN MATRICES
}

void Tracker::initializeNoises(Eigen::MatrixXf noise_data)
{
	mu_w = noise_data(MU_W,0);
	sigma_w = noise_data(SIGMA_W,0);
	var_w = noise_data(VAR_W,0);
	
	mu_v_xi = noise_data(MU_V_XI,0);
	sigma_v_xi = noise_data(SIGMA_V_XI,0);
	var_v_xi = noise_data(VAR_V_XI,0);
	
	mu_v_eta = noise_data(MU_V_ETA,0);
	sigma_v_eta = noise_data(SIGMA_V_ETA,0);
	var_v_eta = noise_data(VAR_V_ETA,0);
	
	mu_v_omega = noise_data(MU_V_OMEGA,0);
	sigma_v_omega = noise_data(SIGMA_V_OMEGA,0);
	var_v_omega = noise_data(VAR_V_OMEGA,0);
}

void Tracker::initializeMatrices()
{
	
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
	
	R << 
	var_w, 0,
	0, var_w;
	
	
	
	P <<
	var_w,var_w/T,0,0,0,
	var_w/T, 2*var_w/(T*T),0,0,0,
	0,0,var_w,var_w/T,0,
	0,0,var_w/T,2*var_w/(T*T),0,
	0,0,0,0,0;
}

void Tracker::updateCoordinatedTurnJacobian()
{
	float xi_hat = x_hat(XI);
	float xi_dot_hat = x_hat(XI_DOT);
	float eta_hat = x_hat(ETA);
	float eta_dot_hat = x_hat(ETA_DOT);
	float omega_hat = x_hat(OMEGA);
	if(abs(omega_hat>0.1))
	{
		updateOmegaPartials(xi_hat, xi_dot_hat, eta_hat, eta_dot_hat, omega_hat);
		f_x <<
		1, (sin(omega_hat*T)/omega_hat), 0, -((1-cos(omega_hat*T))/omega_hat), omega_partials(0),
		0, cos(omega_hat*T), 0, -sin(omega_hat*T), omega_partials(1),
		0, ((1-cos(omega_hat*T))/omega_hat), 1, (sin(omega_hat*T)/omega_hat), omega_partials(2),
		0, sin(omega_hat*T), 0, cos(omega_hat*T), omega_partials(3),
		0, 0, 0, 0, 1;
	}
	else
	{
		f_x <<
		1, T, 0, 0, -0.5*T*T*eta_dot_hat,
		0, 1, 0, 0, -T*eta_dot_hat,
		0, 0, 1, T, 0.5*T*T*xi_dot_hat,
		0, 0, 0, 1, T*xi_dot_hat,
		0, 0, 0, 0, 1;
	}
		
	ROS_INFO("Updating CT Jacobian");
}

void Tracker::initializeCoordinatedTurnSystemMatrix()
{
	float omega = x_hat(OMEGA);
	float inverse_omega = 1/omega; 
	F <<
	1, sin(omega*T)*inverse_omega, 0, (1-cos(omega*T))*inverse_omega, 0,
	0, cos(omega*T), 0, -sin(omega*T), 0,
	0, (1-cos(omega*T))*inverse_omega, 1, sin(omega*T)*inverse_omega, 0,
	0, sin(omega*T), 0, cos(omega*T), 0,
	0, 0, 0, 0, 1;
}

void Tracker::initializeUniformMotionSystemMatrix()
{
	F <<
	1, T, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, T, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 0;
}

void Tracker::update(Eigen::Vector2f z, double an_update_time, int CT_model)
{
	updateDerivatives(z, an_update_time);
	
	nu = z-z_hat;
	x_hat = x_hat_bar + W*nu;
	
	
	
	if(CT_model)
	{
		updateCoordinatedTurnJacobian();
		updateCTStateCovariance();
	}
	else
	{
		updateUMStateCovariance();
	}
	ROS_INFO("Velocity gain[%d] = %f\n",CT_model, getVelocityGainX());
	
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

void Tracker::updateOmegaPartials(float xi_hat, float xi_dot_hat, float eta_hat, float eta_dot_hat, float omega_hat)
{	
	omega_partials(0) = (cos(omega_hat*T)*T*xi_dot_hat/omega_hat)-(sin(omega_hat*T)*xi_dot_hat/(omega_hat*omega_hat))-(sin(omega_hat*T)*T*eta_dot_hat/omega_hat)-((-1+cos(omega_hat*T))*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(1) = -(sin(omega_hat*T)*T*xi_dot_hat)-(cos(omega_hat*T))*T*eta_dot_hat;
	omega_partials(2) = (sin(omega_hat*T)*T*xi_dot_hat/omega_hat)-((1-cos(omega_hat*T))*xi_dot_hat/(omega_hat*omega_hat))+(cos(omega_hat*T)*T*eta_dot_hat/omega_hat)-(sin(omega_hat*T)*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(3) = (cos(omega_hat*T)*T*xi_dot_hat)-(sin(omega_hat*T)*T*eta_dot_hat);
	ROS_INFO("updating omega partials");
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
	return x_hat(XI,0);
}

float Tracker::getPredictedY()
{
	return x_hat(ETA,0);
}

float Tracker::getPredictedXVel()
{
	return x_hat(XI_DOT,0);
}

float Tracker::getPredictedYVel()
{
	return x_hat(ETA_DOT,0);
}

float Tracker::getPredictedOmega()
{
	return x_hat(OMEGA,0);
}

float Tracker::getPositionVarianceX()
{
	return P(XI,XI);
}

float Tracker::getPositionVarianceY()
{
	return P(ETA,ETA);
}

float Tracker::getVelocityVarianceX()
{
	return P(XI_DOT,XI_DOT);
}

float Tracker::getVelocityVarianceY()
{
	return P(ETA_DOT,ETA_DOT);
}

float Tracker::getOmegaVariance()
{
	return P(OMEGA, OMEGA);
}

float Tracker::getPositionGainX()
{
	return W(XI,XI);
}



float Tracker::getPositionGainY()
{
	return W(2,1);
}

float Tracker::getVelocityGainX()
{
	return W(1,0);
}

float Tracker::getVelocityGainY()
{
	return W(3,1);
}


float Tracker::getInnovationX()
{
	return nu(0,0);
}

float Tracker::getInnovationY()
{
	return nu(1,0);
}

float Tracker::getPredictedMeasurementX()
{
	return z_hat(0);
}

float Tracker::getPredictedMeasurementY()
{
	return z_hat(1);
}











void Tracker::resizeMatrices()
{
	
	
		H.resize(NUM_MEASUREMENTS,NUM_STATES); //2x5
		x_hat.resize(NUM_STATES,1);//5x1
		x_hat_bar.resize(NUM_STATES,1);//5x1
		z_hat.resize(NUM_MEASUREMENTS,1);//2x1
		P.resize(NUM_STATES,NUM_STATES);//5x5
		P_bar.resize(NUM_STATES,NUM_STATES);//5x5
		Q.resize(NUM_STATES,NUM_STATES);//5x5
		R.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);//2x2
		Gamma.resize(NUM_STATES,NUM_PROCESS_NOISES);//5x3
		nu.resize(NUM_MEASUREMENTS,1);//2x1
		W.resize(NUM_STATES,NUM_MEASUREMENTS);//5x2
		F.resize(NUM_STATES,NUM_STATES);//5x5
		V.resize(NUM_PROCESS_NOISES, NUM_PROCESS_NOISES);//3x3
		f_x.resize(NUM_STATES,NUM_STATES);
}
 
void Tracker::updateUMStateCovariance()
{
	
	ROS_INFO("P = %f..%f\n%f..%f", P(0,0),P(0,1),P(1,0),P(1,1));
	P_bar = F*P*F.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
}

void Tracker::updateCTStateCovariance()
{
	P_bar = f_x*P*f_x.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
}



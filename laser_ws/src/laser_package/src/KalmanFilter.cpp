#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	ROS_INFO("Empty Kalman Filter");
}

KalmanFilter::KalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data)
{
	ROS_INFO("Initializing a KF");
	T = a_sampling_interval;
	initializeNoises(noise_data);
	initializeMatrices();
	x_hat << an_initial_state(XI), an_initial_state(XI_DOT), an_initial_state(ETA), an_initial_state(ETA_DOT),an_initial_state(OMEGA);	
	omega_initial = 0;
	
	
	initializeSystemMatrix();
	updateCovariance();
	
	x_hat_vec.push_back(x_hat);
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
}

void KalmanFilter::initializeSystemMatrix()
{
	F <<
	1, T, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, T, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 0;
}


void KalmanFilter::updateCovariance()
{
	
	ROS_INFO("P = %f..%f\n%f..%f", P(0,0),P(0,1),P(1,0),P(1,1));
	P_bar = F*P*F.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
}





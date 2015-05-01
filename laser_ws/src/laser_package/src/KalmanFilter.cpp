#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	ROS_INFO("Empty Kalman Filter");
}

KalmanFilter::KalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda)
{
	ROS_INFO("Initializing a KF");
	T = a_sampling_interval;
	Lambda = a_Lambda;
	initializeNoises(noise_data);
	initializeMatrices();
	x_hat << an_initial_state(XI_INDEX), an_initial_state(XI_DOT_INDEX), an_initial_state(ETA_INDEX), an_initial_state(ETA_DOT_INDEX),an_initial_state(OMEGA_INDEX);	
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
	P_bar = F*P*F.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
	
	//ROS_INFO("\nPosition gain KF = %f \nVelocity gain KF = %f\n", getPositionGainX(),getVelocityGainX());
}





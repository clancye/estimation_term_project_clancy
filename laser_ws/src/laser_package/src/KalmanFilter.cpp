#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	ROS_INFO("Empty Kalman Filter");
}

KalmanFilter::KalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda, measurement_vector some_z)
{//CHECKED---GOOD
	ROS_INFO("Initializing a KF");
	z_polar = some_z;
	T = a_sampling_interval;
	Lambda = a_Lambda;
	initializeNoises(noise_data);
	initializeMatrices();//updates bias automatically
	x_hat << an_initial_state(XI_INDEX), an_initial_state(XI_DOT_INDEX), an_initial_state(ETA_INDEX), an_initial_state(ETA_DOT_INDEX),0;	
	omega_initial = 0;
	
	
	initializeSystemMatrix();
	updateCovariance();
	
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
}

void KalmanFilter::initializeSystemMatrix()
{//CHECKED---GOOD
	F <<
	1, T, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, T, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 0;
}

void KalmanFilter::reinitializeFilter(state_vector some_x, covariance_matrix some_P)
{//CHECKED---GOOD
	x_hat = some_x;
	//ROS_INFO("X_hat from x_0j = [%f,%f,%f,%f,%f]", x_hat(0), x_hat(1),x_hat(2), x_hat(3),x_hat(4));
	P = some_P;
	updateCovariance();
	//ROS_INFO("P from P_0j = [%f,%f;%f,%f]", P(0,0), P(0,1), P(1,0), P(1,1));
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
}


void KalmanFilter::updateCovariance()
{//CHECKED---GOOD
	P_bar = F*P*F.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
	//ROS_INFO("\nPosition gain KF = %f \nVelocity gain KF = %f\n", getPositionGainX(),getVelocityGainX());
}





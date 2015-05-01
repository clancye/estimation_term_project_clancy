#include "../include/ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
	ROS_INFO("Empty Extended Kalman Filter");
}

ExtendedKalmanFilter::ExtendedKalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda, measurement_vector some_z)
{//CHECKED---GOOD
	ROS_INFO("Initializing an EKF");
	z_polar = some_z;
	T = a_sampling_interval;
	Lambda = a_Lambda;
	initializeNoises(noise_data);
	initializeMatrices();//updates bias automatically
	x_hat << an_initial_state(XI_INDEX), an_initial_state(XI_DOT_INDEX), an_initial_state(ETA_INDEX), an_initial_state(ETA_DOT_INDEX),an_initial_state(OMEGA_INDEX);	
	omega_initial = an_initial_state(OMEGA_INDEX);
	
	ROS_INFO("Initializing EKF: \n xi_0 =%f\n x_dot_0 = %f\n eta = %f\n eta_dot_0 = %f \n omega_0 = %f\n, SAMPLE_TIME = %f", x_hat(XI_INDEX), x_hat(XI_DOT_INDEX), x_hat(ETA_INDEX), x_hat(ETA_DOT_INDEX), x_hat(OMEGA_INDEX), T); 

	initializeSystemMatrix();
	updateCovariance();
	
	x_hat_bar = f*x_hat;
	z_hat = H*x_hat_bar;
}

void ExtendedKalmanFilter::initializeSystemMatrix()
{//CHECKED---GOOD
	double omega = x_hat(OMEGA_INDEX);
	double inverse_omega = 1/omega; 
	f <<
	1, sin(omega*T)*inverse_omega, 0, (1-cos(omega*T))*inverse_omega, 0,
	0, cos(omega*T), 0, -sin(omega*T), 0,
	0, (1-cos(omega*T))*inverse_omega, 1, sin(omega*T)*inverse_omega, 0,
	0, sin(omega*T), 0, cos(omega*T), 0,
	0, 0, 0, 0, 1;
}

void ExtendedKalmanFilter::reinitializeFilter(state_vector some_x, covariance_matrix some_P)
{//CHECKED---GOOD
	x_hat = some_x;
	
	//ROS_INFO("X_hat from x_0j = [%f,%f,%f,%f,%f]", x_hat(0), x_hat(1),x_hat(2), x_hat(3),x_hat(4));
	P = some_P;
	updateCovariance();
	//ROS_INFO("P from P_0j = [%f,%f;%f,%f]", P(0,0), P(0,1), P(1,0), P(1,1));
	x_hat_bar = f*x_hat;
	z_hat = H*x_hat_bar;
}
void ExtendedKalmanFilter::updateCovariance()
{//CHECKED---GOOD
	updateJacobian();
	P_bar = f_x*P*f_x.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
	
	//ROS_INFO("\nPosition gain EKF = %f \nVelocity gain EKF = %f\n", getPositionGainX(),getVelocityGainX());
}


void ExtendedKalmanFilter::updateJacobian()
{//CHECKED---GOOD
	double xi_dot_hat = x_hat(XI_DOT_INDEX);
	double eta_dot_hat = x_hat(ETA_DOT_INDEX);
	double omega_hat = x_hat(OMEGA_INDEX);
	if(fabs(omega_hat)>0.005)// half a degree for small angle approximation --- top of page 470 purple book
	{
		updateOmegaPartials(xi_dot_hat, eta_dot_hat, omega_hat);
		f_x<<
		1, (sin(omega_hat*T)/omega_hat), 0, -((1-cos(omega_hat*T))/omega_hat), omega_partials(0),
		0, cos(omega_hat*T), 0, -sin(omega_hat*T), omega_partials(1),
		0, ((1-cos(omega_hat*T))/omega_hat), 1, (sin(omega_hat*T)/omega_hat), omega_partials(2),
		0, sin(omega_hat*T), 0, cos(omega_hat*T), omega_partials(3),
		0, 0, 0, 0, 1;
		//ROS_INFO("VALUE ACHIEVED");
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
		
	//ROS_INFO("Updating CT Jacobian OMEGA_HAT = %f", fabs(omega_hat));
}

void ExtendedKalmanFilter::updateOmegaPartials( double xi_dot_hat,  double eta_dot_hat, double omega_hat)
{	//CHECKED---GOOD
	omega_partials(0) = (cos(omega_hat*T)*T*xi_dot_hat/omega_hat)-(sin(omega_hat*T)*xi_dot_hat/(omega_hat*omega_hat))-(sin(omega_hat*T)*T*eta_dot_hat/omega_hat)-((-1+cos(omega_hat*T))*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(1) = -(sin(omega_hat*T)*T*xi_dot_hat)-(cos(omega_hat*T))*T*eta_dot_hat;
	omega_partials(2) = (sin(omega_hat*T)*T*xi_dot_hat/omega_hat)-((1-cos(omega_hat*T))*xi_dot_hat/(omega_hat*omega_hat))+(cos(omega_hat*T)*T*eta_dot_hat/omega_hat)-(sin(omega_hat*T)*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(3) = (cos(omega_hat*T)*T*xi_dot_hat)-(sin(omega_hat*T)*T*eta_dot_hat);
	//ROS_INFO("updating omega partials");
}

double ExtendedKalmanFilter::getMode1Probability()
{
	return 0;
}

double ExtendedKalmanFilter::getMode2Probability()
{
	return 0;
}


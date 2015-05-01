#include "../include/ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
	ROS_INFO("Empty Extended Kalman Filter");
}

ExtendedKalmanFilter::ExtendedKalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda)
{
	ROS_INFO("Initializing an EKF");
	T = a_sampling_interval;
	Lambda = a_Lambda;
	initializeNoises(noise_data);
	initializeMatrices();
	x_hat << an_initial_state(XI_INDEX), an_initial_state(XI_DOT_INDEX), an_initial_state(ETA_INDEX), an_initial_state(ETA_DOT_INDEX),an_initial_state(OMEGA_INDEX);	
	omega_initial = an_initial_state(OMEGA_INDEX);
	
	ROS_INFO("Initializing EKF: \n xi_0 =%f\n x_dot_0 = %f\n eta = %f\n eta_dot_0 = %f \n omega_0 = %f\n, SAMPLE_TIME = %f", x_hat(XI_INDEX), x_hat(XI_DOT_INDEX), x_hat(ETA_INDEX), x_hat(ETA_DOT_INDEX), x_hat(OMEGA_INDEX), T); 
	
	

	initializeSystemMatrix();
	updateJacobian();
	updateCovariance();
	
	x_hat_vec.push_back(x_hat);
	x_hat_bar = F*x_hat;
	z_hat = H*x_hat_bar;
}

void ExtendedKalmanFilter::initializeSystemMatrix()
{
	double omega = x_hat(OMEGA_INDEX);
	double inverse_omega = 1/omega; 
	F <<
	1, sin(omega*T)*inverse_omega, 0, (1-cos(omega*T))*inverse_omega, 0,
	0, cos(omega*T), 0, -sin(omega*T), 0,
	0, (1-cos(omega*T))*inverse_omega, 1, sin(omega*T)*inverse_omega, 0,
	0, sin(omega*T), 0, cos(omega*T), 0,
	0, 0, 0, 0, 1;
}

void ExtendedKalmanFilter::updateCovariance()
{
	updateJacobian();
	P_bar = f_x*P*f_x.transpose() + Q;
	S = H*P_bar*H.transpose() + R;
	W = P_bar*H.transpose()*S.inverse();
	P = P_bar - W*S*W.transpose();
	
	ROS_INFO("\nPosition gain EKF = %f \nVelocity gain EKF = %f\n", getPositionGainX(),getVelocityGainX());
}


void ExtendedKalmanFilter::updateJacobian()
{
	double xi_hat = x_hat(XI_INDEX);
	double xi_dot_hat = x_hat(XI_DOT_INDEX);
	double eta_hat = x_hat(ETA_INDEX);
	double eta_dot_hat = x_hat(ETA_DOT_INDEX);
	double omega_hat = x_hat(OMEGA_INDEX);
	if(fabs(omega_hat)>0.04)
	{
		updateOmegaPartials(xi_hat, xi_dot_hat, eta_hat, eta_dot_hat, omega_hat);
		f_x <<
		1, (sin(omega_hat*T)/omega_hat), 0, -((1-cos(omega_hat*T))/omega_hat), omega_partials(0),
		0, cos(omega_hat*T), 0, -sin(omega_hat*T), omega_partials(1),
		0, ((1-cos(omega_hat*T))/omega_hat), 1, (sin(omega_hat*T)/omega_hat), omega_partials(2),
		0, sin(omega_hat*T), 0, cos(omega_hat*T), omega_partials(3),
		0, 0, 0, 0, 1;
		ROS_INFO("VALUE ACHIEVED");
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

void ExtendedKalmanFilter::updateOmegaPartials(double xi_hat, double xi_dot_hat, double eta_hat, double eta_dot_hat, double omega_hat)
{	
	omega_partials(0) = (cos(omega_hat*T)*T*xi_dot_hat/omega_hat)-(sin(omega_hat*T)*xi_dot_hat/(omega_hat*omega_hat))-(sin(omega_hat*T)*T*eta_dot_hat/omega_hat)-((-1+cos(omega_hat*T))*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(1) = -(sin(omega_hat*T)*T*xi_dot_hat)-(cos(omega_hat*T))*T*eta_dot_hat;
	omega_partials(2) = (sin(omega_hat*T)*T*xi_dot_hat/omega_hat)-((1-cos(omega_hat*T))*xi_dot_hat/(omega_hat*omega_hat))+(cos(omega_hat*T)*T*eta_dot_hat/omega_hat)-(sin(omega_hat*T)*eta_dot_hat/(omega_hat*omega_hat));
	omega_partials(3) = (cos(omega_hat*T)*T*xi_dot_hat)-(sin(omega_hat*T)*T*eta_dot_hat);
	//ROS_INFO("updating omega partials");
}




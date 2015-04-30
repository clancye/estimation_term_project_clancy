#ifndef FILTER
#define FILTER

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "laser_package/update_filter.h"
#include "laser_package/state.h"
#include "gen_defs.h"



#define MEASUREMENT_MEMORY 15
#define MEMORY_COEFFICIENT 0.96

#define UM 0
#define CT 1

#define TURN_RATE 4
#define FILTER_1 0
#define FILTER_2 1




class Filter 
{
	public:
		Filter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data);

		void updateFilter(measurement_vector z, double an_update_time);
		void reinitializeFilter(state_vector an_initial_state, covariance_matrix a_covariance);
		
		//estimates
		double getEstimatedMeasurementX();
		double getEstimatedMeasurementY();
		double getEstimatedOmega();
		double getEstimatedX();
		double getEstimatedY();
		double getEstimatedXVel();
		double getEstimatedYVel();
		
		//variances
		double getPositionVarianceX();
		double getVelocityVarianceX();
		double getPositionVarianceY();
		double getVelocityVarianceY();
		double getOmegaVariance();

		//gains
		double getPositionGainX();
		double getVelocityGainX();
		double getPositionGainY();
		double getVelocityGainY();
		
		//innovations
		double getInnovationX();
		double getInnovationY();
		
		//derivative stuff
		double getXAcceleration();
		double getYAcceleration();
		double getXVelocity();
		double getYVelocity();
		
		//return the 
		covariance_matrix getP();
		state_vector getX();
		
		void setP(covariance_matrix);
		void setX(state_vector);
	
	protected:
		double mu_w, sigma_w, var_w, mu_v_xi, sigma_v_xi,var_v_xi, mu_v_eta, sigma_v_eta,var_v_eta, mu_v_omega, sigma_v_omega,var_v_omega; 

		double last_x,last_y, current_speed, max_current_speed,speed_memory[MEASUREMENT_MEMORY],T, omega_initial;
		double velocity_x_memory[MEASUREMENT_MEMORY], velocity_y_memory[MEASUREMENT_MEMORY], current_x_velocity, current_y_velocity;
		double accel_x_memory[MEASUREMENT_MEMORY], accel_y_memory[MEASUREMENT_MEMORY], current_x_accel, current_y_accel;
		double last_time, second_last_time;
		
		std::vector<state_vector> x_hat_vec;
		std::vector<covariance_matrix> P_vec;
		measurement_matrix H;
		covariance_matrix P;
		predicted_covariance_matrix P_bar; 
		system_matrix F;
		Q_process_covariance_matrix Q;
		measurement_covariance_matrix R;
		noise_gain_matrix Gamma;
		innovation_covariance_matrix S;
		gain_matrix W;
		state_vector x_hat_bar;
		state_vector x_hat;
		measurement_vector z_hat;
		innovation_vector nu;
		V_process_covariance_matrix V;
		jacobian_matrix f_x;
		omega_partials_vector omega_partials;
		
		int system_model;
		
		
		void updateDerivatives(Eigen::Vector2d z, double time_of_measurement);
		
		void initializeNoises(Eigen::MatrixXd noise_data);
		double getDistance(double x_1, double y_1, double x_2, double y_2);
		void initializeStateModel();
		
		void printValues();
		void resizeMatrices();
		void updateCovariance();
		void initializeSystemMatrix();
		void updateSystemMatrix();
		void updateJacobian();
		void initializeMatrices();
		void updateOmegaPartials(double xi_hat, double xi_dot_hat, double eta_hat, double eta_dot_hat, double omega_hat);
		
};

#endif

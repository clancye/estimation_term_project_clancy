#ifndef FILTER_H
#define FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "laser_package/update_filter.h"
#include "laser_package/state.h"
#include "TypeDefs.h"
#include "GenDefs.h"
#include "math.h"


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
		Filter();

		void updateFilter(measurement_vector some_z, double an_update_time);
		virtual void reinitializeFilter(state_vector some_x, covariance_matrix some_P)
		{
			ROS_INFO("Reinitializing empty filter!!! BAD-----------------\n-------------------\n-------------------");
		}
		
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
		
		void calculateLikelihood();
		double getLikelihood();
		//return the 
		covariance_matrix getCovariance();
		state_vector getStateEstimate();
		
	
	protected:
		double mu_w_xi, sigma_w_xi, var_w_xi;
		double mu_w_eta, sigma_w_eta, var_w_eta;
		double mu_v_xi, sigma_v_xi,var_v_xi;
		double mu_v_eta, sigma_v_eta,var_v_eta;
		double mu_v_omega, sigma_v_omega,var_v_omega; 
		double sigma_w_rho, sigma_w_theta, validity_constant, bias;

		double last_x,last_y, current_speed, max_current_speed,speed_memory[MEASUREMENT_MEMORY],T, omega_initial;
		double velocity_x_memory[MEASUREMENT_MEMORY], velocity_y_memory[MEASUREMENT_MEMORY], current_x_velocity, current_y_velocity;
		double accel_x_memory[MEASUREMENT_MEMORY], accel_y_memory[MEASUREMENT_MEMORY], current_x_accel, current_y_accel;
		double last_time, second_last_time;
		
		double Lambda;
		
		measurement_matrix H;
		covariance_matrix P;
		predicted_covariance_matrix P_bar; 
		system_matrix F,f;//lower case is for extended kalman filter
		Q_process_covariance_matrix Q;
		measurement_covariance_matrix R;
		noise_gain_matrix Gamma;
		innovation_covariance_matrix S;
		gain_matrix W;
		state_vector x_hat_bar, x_hat;
		measurement_vector z_hat,z_polar,z_cartesian;
		innovation_vector nu;
		V_process_covariance_matrix V;
		jacobian_matrix f_x;//sticking with convention on page 383 and 384 of purple book
		omega_partials_vector omega_partials;
		
		void updateDerivatives(measurement_vector z, double time_of_measurement);
		
		void initializeNoises(initial_noise_vector noise_data);
		double getDistance(double x_1, double y_1, double x_2, double y_2);
		void printValues();
		void updateCovariance();
		void initializeSystemMatrix();
		void updateSystemMatrix();
		void initializeMatrices();
		void updateBiasing();
		
};

#endif

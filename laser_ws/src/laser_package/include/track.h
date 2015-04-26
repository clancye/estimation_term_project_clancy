#ifndef TRACK
#define TRACK

#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <rosbag/bag.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "laser_package/update_tracker.h"
#include </usr/include/eigen3/Eigen/Dense>
#include "laser_package/state.h"
#include "gen_defs.h"



#define MEASUREMENT_MEMORY 15
#define MEMORY_COEFFICIENT 0.96

#define UM 0
#define CT 1

#define TURN_RATE 4
#define TRACKER_1 0
#define TRACKER_2 1
#define NUM_STATES 5
#define NUM_MEASUREMENTS 2
#define NUM_PROCESS_NOISES 3


#define MU_W 0
#define SIGMA_W 1
#define VAR_W 2

#define MU_V_XI 3
#define SIGMA_V_XI 4
#define VAR_V_XI 5

#define MU_V_ETA 6
#define SIGMA_V_ETA 7
#define VAR_V_ETA 8

#define MU_V_OMEGA 9
#define SIGMA_V_OMEGA 10
#define VAR_V_OMEGA 11

class Tracker //We'll treat this as only a KF for now
{
	public:
	
		Tracker();
		Tracker(Eigen::MatrixXf an_initial_state, float a_sampling_interval, Eigen::MatrixXf noise_data, int CT_model);

		void predictState();
		void update(Eigen::Vector2f z, double an_update_time, int CT_model);
		float getXAcceleration();
		float getYAcceleration();
		float getXVelocity();
		float getYVelocity();
		float getPredictedX();
		float getPredictedY();
		float getPredictedXVel();
		float getPredictedYVel();
		float getPositionVarianceX();
		float getVelocityVarianceX();
		float getPositionGainX();
		float getVelocityGainX();
		float getPositionVarianceY();
		float getVelocityVarianceY();
		float getOmegaVariance();
		float getPositionGainY();
		float getVelocityGainY();
		float getInnovationX();
		float getInnovationY();
		float getPredictedMeasurementX();
		float getPredictedMeasurementY();
		float getPredictedOmega();
	
	private:
	
		float last_x,last_y, current_speed, max_current_speed,speed_memory[MEASUREMENT_MEMORY],T, omega_initial;
		float mu_w, sigma_w, var_w, mu_v_xi, sigma_v_xi,var_v_xi, mu_v_eta, sigma_v_eta,var_v_eta, mu_v_omega, sigma_v_omega,var_v_omega; 
		float velocity_x_memory[MEASUREMENT_MEMORY], velocity_y_memory[MEASUREMENT_MEMORY], current_x_velocity, current_y_velocity;
		float accel_x_memory[MEASUREMENT_MEMORY], accel_y_memory[MEASUREMENT_MEMORY], current_x_accel, current_y_accel;
		double last_time, second_last_time;
		std::vector<Eigen::MatrixXf> x_hat_vec, P_vec;
		Eigen::MatrixXf H,P,F,Q,R,Gamma, P_bar, S, W,x_hat_bar, x_hat, z_hat, nu, V,f_x;
		Eigen::Vector4f omega_partials;
		int system_model;
		void updateDerivatives(Eigen::Vector2f z, double time_of_measurement);
		void initializeNoises(Eigen::MatrixXf noise_data);
		float getDistance(float x_1, float y_1, float x_2, float y_2);
		void initializeStateModel();
		void printValues();
		void resizeMatrices();
		void updateUMStateCovariance();
		void updateCTStateCovariance();
		void initializeUniformMotionSystemMatrix();
		void initializeCoordinatedTurnSystemMatrix();
		void updateCoordinatedTurnJacobian();
		void initializeMatrices();
		void updateOmegaPartials(float xi_hat, float xi_dot_hat, float eta_hat, float eta_dot_hat, float omega_hat);
		
};

#endif

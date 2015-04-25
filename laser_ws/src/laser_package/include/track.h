#ifndef TRACK
#define TRACK

#include <ros/ros.h>
#include <math.h>
#include <rosbag/bag.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "laser_package/update_tracker.h"
#include </usr/include/eigen3/Eigen/Dense>
#include "laser_package/state.h"



#define MEASUREMENT_MEMORY 15
#define MEMORY_COEFFICIENT 0.96
#define T 1
#define XI 0
#define XI_DOT 1
#define ETA 2
#define ETA_DOT 3
#define MU_V 0.0
#define MU_W 0.0
#define SIGMA_V 3.0
#define SIGMA_W 1.0
#define VAR_V SIGMA_V*SIGMA_V
#define VAR_W SIGMA_W*SIGMA_W

class Tracker //We'll treat this as only a KF for now
{
	public:
	
		Tracker();
		Tracker(Eigen::MatrixXf an_initial_state);

		void predictState();
		void update(Eigen::Vector2f z, double an_update_time);
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
		float getPositionGainY();
		float getVelocityGainY();
		float getInnovationX();
		float getInnovationY();
		float getPredictedMeasurementX();
		float getPredictedMeasurementY();
	
	private:
	
		float last_x,last_y, current_speed, max_current_speed,speed_memory[MEASUREMENT_MEMORY];
		float velocity_x_memory[MEASUREMENT_MEMORY], velocity_y_memory[MEASUREMENT_MEMORY], current_x_velocity, current_y_velocity;
		float accel_x_memory[MEASUREMENT_MEMORY], accel_y_memory[MEASUREMENT_MEMORY], current_x_accel, current_y_accel;
		double last_time, second_last_time;
		std::vector<Eigen::MatrixXf> x_hat_vec, P_vec;
		Eigen::MatrixXf H,H_temp,P,F,F_temp,Q,R,Gamma, P_bar, S, W,W_temp, x_hat_bar, x_hat, z_hat, nu, temp_initial_state;
		int system_model;
		void updateDerivatives(Eigen::Vector2f z, double time_of_measurement);
		float getDistance(float x_1, float y_1, float x_2, float y_2);
		void initializeStateModel();
		void printValues();
		void resizeMatrices();
		void initializeMatrices();
		void updateStateCovariance();
		
};

#endif

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


#define DWNA_X 0
#define DWNA_X_AND_Y 1
#define CT 2
#define MEASUREMENT_MEMORY 15
#define MEMORY_COEFFICIENT 0.96
#define SAMPLE_TIME 0.1

class Tracker //We'll treat this as only a KF for now
{
	public:
		Tracker();
		
		Tracker(//start of Tracker constructor
		Eigen::RowVector4f some_noise_data, 
		int a_system_model, 
		Eigen::MatrixXf an_initial_state
		);//end of Tracker constructor

	
		void predictState();
		void updateXY(float x, float y, double an_update_time);
		void updateX(float x, double an_update_time);
		float getXAcceleration();
		float getYAcceleration();
		float getXVelocity();
		float getYVelocity();
	
	private:
		float sigma_v, sigma_w, mu_v, mu_w, last_x,last_y, current_speed, max_current_speed,speed_memory[MEASUREMENT_MEMORY];
		float velocity_x_memory[MEASUREMENT_MEMORY], velocity_y_memory[MEASUREMENT_MEMORY], current_x_velocity, current_y_velocity;
		float accel_x_memory[MEASUREMENT_MEMORY], accel_y_memory[MEASUREMENT_MEMORY], current_x_accel, current_y_accel;
		double last_time, second_last_time;
		std::vector<Eigen::MatrixXf> x_hat_vec, P_vec;
		void initializeNoises(Eigen::RowVector4f noise_data);
		Eigen::MatrixXf H,P,F,Q,R,Gamma, P_bar, S, W, x_hat_bar, x_hat;
		int system_model;
		void updateSpeedXY(float some_x, float some_y, double time_of_measurement);
		void updateSpeedX(float some_x, double time_of_measurement);
		float getDistance(float x_1, float y_1, float x_2, float y_2);
		void updateMeasurementHistories(float x, float y);
		void initializeStateModel();
		
};

#endif

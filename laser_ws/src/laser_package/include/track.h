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

#define DWNA = 0
#define CT = 1
#define MEASUREMENT_MEMORY 10


class Tracker //We'll treat this as only a KF for now
{
	public:
		Tracker();
		
		Tracker(//start of Tracker constructor
		Eigen::RowVector4f some_noise_data, 
		int a_system_model, 
		Eigen::MatrixXf a_measurement_matrix, 
		Eigen::MatrixXf an_initial_covariance, 
		Eigen::MatrixXf an_initial_state
		);//end of Tracker constructor

	
		void predictState();
		void update(float x, float y, double an_update_time);
	
	private:
		float sigma_v, sigma_w, mu_v, mu_w, last_x,last_y, last_time, current_speed, speed_memory[MEASUREMENT_MEMORY];
		std::vector<Eigen::MatrixXf> x_hat;
		void initializeNoises(Eigen::RowVector4f noise_data);
		Eigen::MatrixXf H,P;
		int system_model;
		void updateSpeed(float some_x, float some_y, double time_of_measurement);
		float getDistance(float x_1, float y_1, float x_2, float y_2);
		void updateMeasurementHistories(float x, float y);
		
};

#endif

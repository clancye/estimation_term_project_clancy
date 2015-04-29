#ifndef GEN_DEFS
#define GEN_DEFS

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <math.h>
#include <cmath>
#include <string.h>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

#define XI 0
#define XI_DOT 1
#define ETA 2
#define ETA_DOT 3
#define OMEGA 4

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

#define PI 3.14159265358979
#define DEG_TO_RAD_DENOM 0.00555555555


#define NUM_STATES 5
#define NUM_MEASUREMENTS 2
#define NUM_PROCESS_NOISES 3

typedef Eigen::Matrix<double, NUM_MEASUREMENTS, NUM_STATES> measurement_matrix;
typedef Eigen::Matrix<double, NUM_STATES,1> state_vector;
typedef Eigen::Matrix<double, NUM_MEASUREMENTS,1> measurement_vector;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> covariance_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> predicted_covariance_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_process_covariance_matrix;
typedef Eigen::Matrix<double, NUM_MEASUREMENTS, NUM_MEASUREMENTS> measurement_covariance_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_PROCESS_NOISES> noise_gain_matrix;
typedef Eigen::Matrix<double, NUM_MEASUREMENTS, 1> innovation_vector;
typedef Eigen::Matrix<double, NUM_MEASUREMENTS, NUM_MEASUREMENTS> innovation_covariance_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_MEASUREMENTS> gain_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> system_matrix;
typedef Eigen::Matrix<double, NUM_PROCESS_NOISES, NUM_PROCESS_NOISES> V_process_covariance_matrix;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> jacobian_matrix;
typedef Eigen::Matrix<double, NUM_STATES -1, 1> omega_partials_vector;
typedef Eigen::Matrix<double, 12,1> initial_noise_vector;
#endif

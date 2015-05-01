#ifndef TYPE_DEFS_H
#define TYPE_DEFS_H

#include </usr/include/eigen3/Eigen/Dense>

#define NUM_FILTERS 2
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
typedef Eigen::Matrix<double, 15,1> initial_noise_vector;
typedef Eigen::Matrix<double, NUM_FILTERS, NUM_FILTERS> transition_probability_matrix;
typedef Eigen::Matrix<double, NUM_FILTERS, NUM_FILTERS> mixing_probability_matrix;
typedef Eigen::Matrix<double, NUM_FILTERS, 1> mode_probability_vector;

#endif

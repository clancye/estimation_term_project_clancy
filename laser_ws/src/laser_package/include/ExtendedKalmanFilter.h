#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "laser_package/add_filter.h"
#include "laser_package/state.h"
#include "laser_package/update_imm_filter.h"
#include "Filter.h"

class ExtendedKalmanFilter : public Filter
{
	public:
		
		ExtendedKalmanFilter();
		ExtendedKalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda);
		
		void initializeSystemMatrix();
		void updateCovariance();
		void updateJacobian();
		void updateOmegaPartials(double xi_hat, double xi_dot_hat, double eta_hat, double eta_dot_hat, double omega_hat);
		
};

#endif

#ifndef KalmanFilter_H
#define KalmanFilter_H

#include "laser_package/add_filter.h"
#include "laser_package/update_imm_filter.h"
#include "laser_package/update_filter_priors.h"
#include "laser_package/state.h"
#include "Filter.h"
class KalmanFilter : public Filter
{
	public:
		
		KalmanFilter();
		KalmanFilter(state_vector an_initial_state, double a_sampling_interval, initial_noise_vector noise_data, double a_Lambda, measurement_vector some_z);
		
		void initializeSystemMatrix();
		void updateCovariance();
		
};


#endif

#ifndef IMM_H
#define IMM_H

#include "laser_package/add_filter.h"
#include "laser_package/update_imm_filter.h"
#include "laser_package/update_filter_priors.h"
#include "GenDefs.h"
#include "TypeDefs.h"
#include "Filter.h"
class IMM : public Filter//We'll treat this as only a KF for now
{
	public:
	
		IMM();
		IMM(std::vector<Filter*> some_filters);
		//addFilter(Filter a_filter);
		
		void update();
		void incrementFilterCount();
		void decrementFilterCount();
		int getFilterCount();
		void setP0J(int some_ID, covariance_matrix some_P);
		void setX0j(int some_ID, state_vector some_x);
		void setLambda(int some_ID, double some_lambda);
		covariance_matrix getP0j(int an_ID);
		state_vector getX0j(int an_ID);
		int getReinitialize();
		
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
		
		//innovations
		double getInnovationX();
		double getInnovationY();
		
		//mode probabilities (only for IMM)
		double getMode1Probability();
		double getMode2Probability();
	
	private:
	
		double c;
		transition_probability_matrix p;
		covariance_matrix P_zeros;
		state_vector x_zeros;
		mixing_probability_matrix mu_mix;
		mode_probability_vector mu_mode, Lambda,c_j_bar;
		int num_filters, num_updates;
		int reinitialize_filters;
		std::vector<covariance_matrix> P_0j;
		std::vector<covariance_matrix> P_filter;
		std::vector<state_vector> x_0j;
		std::vector<state_vector> x_hat_filter;
		std::vector<Filter*> filters; 
	
		void initializeProbabilities();
		void getFilterValues();
		void updateModeProbabilities();
		void calculateMixingProbabilities();
		void mix();
		void updateFilterPriors();
		void combineEstimatesAndCovariances();
		void calculateMixingNormalizationConstant();
		void calculateModeNormalizationConstant();
	
};

#endif

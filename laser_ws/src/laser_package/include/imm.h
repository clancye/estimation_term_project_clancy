#ifndef IMM_H
#define IMM_H

#include "gen_defs.h"
#include "laser_package/add_filter.h"
#include "laser_package/update_imm_filter.h"
class IMM //We'll treat this as only a KF for now
{
	public:
	
		IMM();
		//addFilter(Filter a_filter);
		
		void updateIMM(int filterID, Eigen::VectorXd x_hat_j, Eigen::MatrixXd P_j, double a_lambda);
		void incrementFilterCount();
		void decrementFilterCount();
		int getFilterCount();
		void setP0J(int some_ID, Eigen::MatrixXd some_P);
		void setX0j(int some_ID, Eigen::VectorXd some_x);
		void setLambda(int some_ID, double some_lambda);
		Eigen::MatrixXd getP0j(int an_ID);
		Eigen::VectorXd getX0j(int an_ID);
		int getReinitialize();
	
	private:
	
		double c;
		Eigen::MatrixXd p_ij,P,mu_ij, P_zeros, x_zeros;
		Eigen::VectorXd mu_i, Lambda, x_hat;
		int filter_count, num_updates;
		int reinitialize_filters;
		std::vector<Eigen::MatrixXd> P_0j;
		std::vector<Eigen::MatrixXd> P_filter;
		std::vector<Eigen::VectorXd> x_hat_filter;
		std::vector<Eigen::VectorXd> x_0j;
		std::vector<double> c_j_bar;
		//std::vector<Filter*> filters; 
	
		void resizeMatrices();
		void calculateMixingProbabilities();
		void mix();
		void findLikelihoods();
		void updateModeProbabilities();
		void combineEstimatesAndCovariances();
		void calculateMixingNormalizationConstant();
		void calculateModeNormalizationConstant();
		void initializeMatrices();
	
};

#endif

#ifndef SIMULATE
#define SIMULATE



#include <random>
#include "laser_package/state.h"
#include "laser_package/update_tracker.h"
#include "Estimator.h"


#define SAMPLING_INTERVAL .1
#define TURNING_RATE_DEGREES 50.0
#define OMEGA_RADS  TURNING_RATE_DEGREES*PI*DEG_TO_RAD_DENOM
#define MU_W_SIMULATE 0.0
#define SIGMA_W_SIMULATE 1.0
#define VAR_W_SIMULATE SIGMA_W_SIMULATE*SIGMA_W_SIMULATE
#define MU_XI 0.0
#define SIGMA_XI 1.0
#define VAR_XI SIGMA_XI*SIGMA_XI
#define MU_ETA 0.0
#define SIGMA_ETA 1.0
#define VAR_ETA SIGMA_ETA*SIGMA_ETA
#define MU_OMEGA 0.0
#define SIGMA_OMEGA 0.02
#define VAR_OMEGA SIGMA_OMEGA*SIGMA_OMEGA


class Simulator //We'll treat this as only a KF for now
{
	public:
	
		Simulator();
		
		void initializeSimulators(Eigen::MatrixXd an_initial_state);
		Eigen::MatrixXd simulateUniformMotion();
		Eigen::MatrixXd simulateCoordinatedTurn(double a_turn_rate);
		
	
	private:
		std::default_random_engine generator;
		std::normal_distribution<double> process_noise;
		double omega, inverse_omega, T;
		std::vector<Eigen::MatrixXd> x;
		Eigen::Vector3d v;
		Eigen::MatrixXd F_CT, F_UM, Gamma;//transition matrices for uniform motion and coordinated turns
		
};

#endif

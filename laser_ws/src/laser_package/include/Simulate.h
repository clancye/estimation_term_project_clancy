#ifndef SIMULATE_H
#define SIMULATE_H



#include <random>
#include "laser_package/state.h"
#include "laser_package/update_tracker.h"
#include "Estimator.h"

//simulation rate (NOT EQUIVALENT TO SAMPLING INTERVAL!)
#define SIMULATE_RATE 1


//xi measurement noise statistics
#define MU_W_SIMULATE_XI 0.0
#define SIGMA_W_SIMULATE_XI 1.0
#define VAR_W_SIMULATE_XI SIGMA_W_SIMULATE_XI*SIGMA_W_SIMULATE_XI
//eta measurement noise statistics
#define MU_W_SIMULATE_ETA 0.0
#define SIGMA_W_SIMULATE_ETA 1.0
#define VAR_W_SIMULATE_ETA SIGMA_W_SIMULATE_ETA*SIGMA_W_SIMULATE_ETA
//rho measurement noise statistics
#define MU_W_SIMULATE_RHO 0.0
#define SIGMA_W_SIMULATE_RHO 50.0 //meters
#define VAR_W_SIMULATE_RHO SIGMA_W_SIMULATE_RHO*SIGMA_W_SIMULATE_RHO //meters squared
//theta measurement noise statistics
#define MU_W_SIMULATE_THETA 0.0
#define SIGMA_W_SIMULATE_THETA 1*PI*DEG_TO_RAD_DENOM // one degree in radians
#define VAR_W_SIMULATE_THETA SIGMA_W_SIMULATE_THETA*SIGMA_W_SIMULATE_THETA


//xi_process_noise statistics
#define MU_XI_SIMULATE 0.0
#define SIGMA_XI_SIMULATE 0.0
#define VAR_XI_SIMULATE SIGMA_XI_SIMULATE*SIGMA_XI_SIMULATE
//eta process_noise statistics
#define MU_ETA_SIMULATE 0.0
#define SIGMA_ETA_SIMULATE 0.0
#define VAR_ETA_SIMULATE SIGMA_ETA_SIMULATE*SIGMA_ETA_SIMULATE
//omega process_noise statistics
#define MU_OMEGA_SIMULATE 0.0
#define SIGMA_OMEGA_SIMULATE 0.0
#define VAR_OMEGA_SIMULATE SIGMA_OMEGA_SIMULATE*SIGMA_OMEGA_SIMULATE


class Simulator //We'll treat this as only a KF for now
{
	public:
	
		Simulator();
		
		void initializeSimulators(state_vector an_initial_state);
		state_vector simulateUniformMotion();
		state_vector simulateCoordinatedTurn();
		void setOmega(double);
		double getOmega();
		
	
	private:
		std::default_random_engine generator;
		std::normal_distribution<double> process_noise;
		double omega, inverse_omega, T;
		std::vector<Eigen::MatrixXd> x;
		Eigen::Vector3d v;
		system_matrix F_CT, F_UM;
		noise_gain_matrix Gamma;//transition matrices for uniform motion and coordinated turns
		
};

#endif

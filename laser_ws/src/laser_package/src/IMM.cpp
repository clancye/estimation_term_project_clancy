#include "../include/IMM.h"

IMM::IMM()
{
	ROS_INFO("IMM turning on...");
}

IMM::IMM(std::vector<Filter*> some_filters)
{
	filters = some_filters;
	num_filters = filters.size();
	initializeProbabilities();
}

void IMM::initializeProbabilities()
{
	p << 0.95, 0.05, 0.05, 0.95;
	calculateMixingProbabilities();
}

void IMM::update()
{
	getFilterValues();
	updateModeProbabilities();
	calculateMixingProbabilities();
	mix();
	updateFilterPriors();
	combineEstimatesAndCovariances();
}

void getFilterValues()
{
	for(int i = 0; i<num_filters;i++)
	{
		Lambda(i) = filters[i]->getLikelihood();
		P_filter_j[i] = filters[i]->getCovariance();
		x_filter_x[i] = filters[i]->getStateEstimate();
	}
}

void IMM::updateFilterPriors()
{
	for(int i = 0; i<num_filters;i++)
	{
		filters[i]->reinitialize(x_0j[i], P_0j[i]);
	}
}

void IMM::calculateMixingProbabilities()
{
	ROS_INFO("calculating mixing probabilities");
	calculateMixingNormalizationConstant();
	for(int i = 0; i<num_filters;i++)
	{
		for(int j = 0; j<num_filters;j++)
		{
			mu_mix(i,j) = (1/c_j_bar[j])*p(i,j)*mu_mode(i);
		}
	}
}

void IMM::calculateMixingNormalizationConstant()
{
	for(int j = 0;j<num_filters;j++)
	{
		c_j_bar[j] = 0;
		for(int i = 0;i<num_filters;i++)
		{
			c_j_bar[j] += p(i,j)*mu_mode(i);
		}
	}
}
void IMM::mix()
{
	ROS_INFO("mixing");
	for(int j = 0;j<num_filters;j++)
	{
		x_0j[j] = x_zeros;
		for(int i = 0; i<num_filters;i++)
		{
			x_0j[j] += x_hat_filter[i]*mu_mix(i,j);
		}
	}
}

void IMM::updateModeProbabilities()
{
	ROS_INFO("updating mode probabilities");
	calculateModeNormalizationConstant();
	for(int i = 0;i<num_filters;i++)
	{
		mu_mode(i) = (1/c)*Lambda(i)*c_j_bar[i];
	}
}

void IMM::calculateModeNormalizationConstant()
{
	c=0;
	for(int i = 0; i<num_filters; i++)
	{
		c += Lambda(i)*c_j_bar[i];
	}
}
void IMM::combineEstimatesAndCovariances()
{
	x_hat = x_zeros;
	P = P_zeros;
	ROS_INFO("combining estimate and covariance \nZEROS x_hat = [%f, %f]\n ZEROS P = [%f,%f,%f,%f]", x_hat(0), x_hat(1), P(0,0), P(0,1), P(1,0), P(1,1));
	for(int i = 0; i<num_filters;i++)
	{
		x_hat += x_filter_j[i]*mu_mode(i);
	}
	for(int i = 0;i<num_filters;i++)
	{
		P += mu_mode*(P_filter_j[i]+(x_filter_j[i]-x_hat)*(x_filter_j[i].transpose()-x_hat.transpose()))
	}
}

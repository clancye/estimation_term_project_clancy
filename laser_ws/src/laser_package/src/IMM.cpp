#include "../include/IMM.h"

IMM::IMM()
{
	ROS_INFO("IMM turning on...");
}

IMM::IMM(std::vector<Filter*> some_filters)
{//CHECKED---GOOD
	//ROS_INFO("instantiating IMM");
	filters = some_filters;
	num_filters = some_filters.size();
	//ROS_INFO("num_filters = %d", num_filters);
	initializeProbabilities();
	P_filter.resize(num_filters);
	x_hat_filter.resize(num_filters);
	x_0j.resize(num_filters);
	P_0j.resize(num_filters);
	x_zeros<<Eigen::MatrixXd::Zero(NUM_STATES,1);
	P_zeros<<Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
}

void IMM::initializeProbabilities()
{//CHECKED---GOOD
	//ROS_INFO("initializing probabilities");
	p << 0.95, 0.05, 0.05, 0.95;
	for(int i = 0;i<num_filters;i++)
	{
		mu_mode(i) = (double)(1/(1.0*num_filters));
		//ROS_INFO("mu_mode[%d] = %f", i,mu_mode(i));
	}
	calculateMixingNormalizationConstant();
	
}

void IMM::update()
{//CHECKED---GOOD
	//ROS_INFO("updating IMM");
	getFilterValues();
	updateModeProbabilities();
	calculateMixingProbabilities();
	mix();
	updateFilterPriors();
	combineEstimatesAndCovariances();
}

void IMM::getFilterValues()
{//CHECKED---GOOD
	//ROS_INFO("getting filter values");
	for(int i = 0; i<num_filters;i++)
	{
		Lambda(i) = filters[i]->getLikelihood();
		P_filter[i] = filters[i]->getCovariance();
		x_hat_filter[i] = filters[i]->getStateEstimate();
	}
}

void IMM::updateFilterPriors()
{//CHECKED---GOOD
	//ROS_INFO("updating filter priors");
	for(int i = 0; i<num_filters;i++)
	{
		filters[i]->reinitializeFilter(x_0j[i], P_0j[i]);
	}
}

void IMM::calculateMixingProbabilities()
{//CHECKED---GOOD
	//ROS_INFO("calculating mixing probabilities");
	calculateMixingNormalizationConstant();
	for(int i = 0; i<num_filters;i++)
	{
		for(int j = 0; j<num_filters;j++)
		{
			mu_mix(i,j) = (1/c_j_bar(j))*p(i,j)*mu_mode(i);
			
		//ROS_INFO("mu_mix[%d,%d] = %f", i,j,mu_mix(i,j));
		}
	}
}

void IMM::calculateMixingNormalizationConstant()
{//CHECKED---GOOD
	//ROS_INFO("calculating mixing normalization constant");
	for(int j = 0;j<num_filters;j++)
	{
		c_j_bar(j) = 0;
		for(int i = 0;i<num_filters;i++)
		{
			c_j_bar(j) += p(i,j)*mu_mode(i);
			
		}
		//ROS_INFO("c_j_bar[%d] = %f",j, c_j_bar(j));
	}
}
void IMM::mix()
{//CHECKED---GOOD
	//ROS_INFO("mixing");
	for(int j = 0;j<num_filters;j++)
	{//CHECKED---GOOD
		x_0j[j] = x_zeros;
		for(int i = 0; i<num_filters;i++)
		{
			x_0j[j] += x_hat_filter[i]*mu_mix(i,j);
		}
	}
	
	for(int j = 0;j<num_filters;j++)
	{//CHECKED---GOOD
		P_0j[j] = P_zeros;
		for(int i = 0; i<num_filters;i++)
		{
			P_0j[j] += mu_mix(i,j)*(P_filter[i]+(x_hat_filter[i]-x_0j[j])*(x_hat_filter[i].transpose()-x_0j[j].transpose()));
		}
		//ROS_INFO("P_0j[%d] = [%f,%f;%f,%f] \n x_0j[%d] = [%f,%f]", j,P_0j[j](0,0),P_0j[j](0,1),P_0j[j](1,0),P_0j[j](1,1),j,x_0j[j](0),x_0j[j](1));
	
	}
		
}

void IMM::updateModeProbabilities()
{//CHECKED---GOOD
	//ROS_INFO("updating mode probabilities");
	calculateModeNormalizationConstant();
	for(int j = 0;j<num_filters;j++)
	{
		mu_mode(j) = (1/c)*Lambda(j)*c_j_bar(j);
		//ROS_INFO("mu_mode(%d) = %f, Lambda (%d) = %f, c_j_bar(%d) = %f", j, mu_mode(j),j,Lambda(j),j,c_j_bar(j));
	}
}

void IMM::calculateModeNormalizationConstant()
{//CHECKED---GOOD
	//ROS_INFO("calculating mode normalization constant");
	c=0;
	for(int j = 0; j<num_filters; j++)
	{
		c += Lambda(j)*c_j_bar(j);
	}
}
void IMM::combineEstimatesAndCovariances()
{//CHECKED---GOOD
	x_hat = x_zeros;
	P = P_zeros;
	//ROS_INFO("combining estimate and covariance \nZEROS x_hat = [%f, %f]\n ZEROS P = [%f,%f,%f,%f]", x_hat(0), x_hat(1), P(0,0), P(0,1), P(1,0), P(1,1));
	for(int j = 0; j<num_filters;j++)
	{
		x_hat += x_hat_filter[j]*mu_mode(j);
	}
	for(int j = 0;j<num_filters;j++)
	{
		P += mu_mode(j)*(P_filter[j]+(x_hat_filter[j]-x_hat)*(x_hat_filter[j].transpose()-x_hat.transpose()));
	}
}

double IMM::getMode1Probability()
{
	return mu_mode(0);
}

double IMM::getMode2Probability()
{
	return mu_mode(1);
}

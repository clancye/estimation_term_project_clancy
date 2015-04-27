#include "../../include/imm.h"

IMM::IMM()
{
	ROS_INFO("IMM turning on...");
}

/*
void addFilter(Filter a_filter)
{
	filters.push_back(&a_filter);
}*/


void IMM::calculateMixingProbabilities()
{
	ROS_INFO("calculating mixing probabilities");
}
void IMM::mix()
{
	ROS_INFO("mixing");
}

void IMM::findLikelihoods()
{
	ROS_INFO("finding likelihoods");
}
void IMM::updateModeProbabilities()
{
	ROS_INFO("updating mode probabilities");
}
void IMM::combineEstimatesAndCovariances()
{
	ROS_INFO("combining estimate and covariance");
}

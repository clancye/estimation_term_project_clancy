#include "../../include/detect.h"

int place_keeper,new_series, num_points_min, num_points_max,num_points;

std::vector<float> detectTargets(std::vector<float> ranges)
{
	std::vector<float> filtered_values (682), point_targets (682);
	float mean_range = 0.0;
	int mean_index;
	new_series = 1;
	for (int i = 85;i<630;i++) //iterate through the data
	{
		if(ranges[i]<4.8)
		{
			if(new_series) //if we're moving on to a new potential target, reset variables
			{
				place_keeper = i;
				num_points_min = floor(MIN_TARGET_WIDTH/(ranges[place_keeper]*THETA_DELTA));
				num_points_max = floor(MAX_TARGET_WIDTH/(ranges[place_keeper]*THETA_DELTA));
				new_series = 0;	
				num_points = 1;
			}
			if(abs(ranges[i+1]-ranges[place_keeper])<MAX_TARGET_DEPTH) //if the target is reasonably deep
			{
				num_points++; //then increment num_points
			}
			else //if we come across a point that is too far from the first point
			{
				if(num_points>num_points_min&&num_points<num_points_max) //check if num_points makes sense
				{
					
					mean_range /= num_points;
					mean_index = place_keeper + floor(num_points/2);
					point_targets[mean_index] = ranges[mean_index];
					mean_range = 0;
				}
				num_points = 0;
				new_series = 1;//start looking for more targets
			}
		}
	}
	return point_targets; //change to filtered_ranges to show extended targets, point_targets for point targets
}

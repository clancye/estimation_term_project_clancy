#include "../../include/detect.h"

int place_keeper,new_series, num_points_min, num_points_max,num_points;

std::vector<float> detectTargets(std::vector<float> ranges)
{
	std::vector<float> filtered_values (682);
	new_series = 1;
	for (int i = 0;i<682;i++) //iterate through the data
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
				for(int j = place_keeper;j<place_keeper+num_points;j++)//if it does, enter the target in a new array
				{
					filtered_values[j] = ranges[j];
				}
			}
			num_points = 0;
			new_series = 1;//start looking for more targets
		}
	}
	return filtered_values;
}

#include "../../include/detect.h"

std::vector<float> Detector::detectTargets(std::vector<float> ranges)
{
	
	float mean_range = 0.0;
	std::vector<float> tracked_targets (682);
	int mean_index;
	new_series = 1;
	for (int i = 85;i<650;i++) //iterate through the data
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
					mean_index = place_keeper + floor(num_points/2);
					mean_range = ranges[mean_index];
					{
						switch(state)
						{
							int zone;
							case INITIALIZING:	
								if(getZone(mean_range,mean_index)==-1) 
								{
									createZone(mean_range,mean_index,-1);//not tracking so we use -1
									initial_targets[mean_index] = ranges[mean_index];
									point_targets = initial_targets;
								}
								break;
							case WAITING_FOR_MOVING_TARGET:
								zone = getZone(mean_range,mean_index);
								if(zone != -1)updateVelocity(zone,mean_range, mean_index,scan_time);
								point_targets = initial_targets;
								break;
							case TRACKING:
								createZone(mean_range,mean_index,zoneBeingTracked);
								tracked_targets[mean_index] = mean_range;
								point_targets = tracked_targets;
								break;
						}
					}
					mean_range = 0;
				}
				num_points = 0;
				new_series = 1;//start looking for more targets
			}
		}
	}
	//ROS_INFO("size of point_targets = %d", point_targets.size());
	return point_targets; //change to filtered_ranges to show extended targets, point_targets for point targets
}

void Detector::setState(int some_state)
{
	state = some_state;
}

int Detector::getState()
{
	return state;
}

int Detector::getZone(float range,int index)
{
	for(int i = 0; i<zone_counter;i++)
	{
		if(range>zone_range_min[i]&&range<zone_range_max[i]&&index<zone_index_max[i]&&index>zone_index_min[i]) return i;
		
	}
	return -1;
}

void Detector::createZone(float range,int index,int zoneToTrack)
{
	if(zoneToTrack==-1)
	{
		ROS_INFO("range = %f \n index = %d\n", range, index);
		possible_target_range.push_back(range);
		possible_target_index.push_back(index);
		possible_target_x.push_back(getCartesianX(range, index));
		possible_target_y.push_back(getCartesianY(range, index));
		possible_target_time.push_back(scan_time);
		zone_index_min.push_back(index - floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_index_max.push_back(index + floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_range_min.push_back(range - MAX_MOVEMENT_PER_SCAN);
		zone_range_max.push_back(range + MAX_MOVEMENT_PER_SCAN);
		int i = zone_counter;
		ROS_INFO("zone_index_min[%d] = %d\n zone_index_max[%d] = %d\n zone_range_min[%d] = %f, zone_range_max[%d] = %f", i,zone_index_min[i],i,zone_index_max[i],i,zone_range_min[i],i,zone_range_max[i]);
		ROS_INFO("possible target range and index = [%f,%d]", possible_target_range[zone_counter],possible_target_index[zone_counter]);
		ROS_INFO("possible target [x,y] = [%f,%f]", possible_target_x[zone_counter],possible_target_y[zone_counter]);
		ROS_INFO("TIME OF AQUISITION %f \n", possible_target_time[zone_counter]);
		zone_counter++;
	}
	else
	{
		zone_index_min[zoneToTrack] = (index - floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_index_max[zoneToTrack] = (index + floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_range_min[zoneToTrack] = (range - MAX_MOVEMENT_PER_SCAN);
		zone_range_max[zoneToTrack] = (range + MAX_MOVEMENT_PER_SCAN);
	}
}

void Detector::updateVelocity(int zone,float range, int index, double time)
{
	float tempX = getCartesianX(range, index);
	float tempY = getCartesianY(range, index);
	float distance = sqrt(pow((tempX-possible_target_x[zone]),2)+pow((tempY-possible_target_y[zone]),2));
	float timeBetweenMeasurements = time - possible_target_time[zone];
	float velocity = distance/timeBetweenMeasurements;
	ROS_INFO("velocity = %f\n", velocity);
	if(velocity>VELOCITY_THRESHOLD_MIN&&velocity<VELOCITY_THRESHOLD_MAX)
	{
		trackZone(zone);
		ROS_INFO("distance = %f\n start time = %f \n now_time %f\n VELOCITY FOR ZONE %d = %f\n", distance, possible_target_time[zone],scan_time,zone, velocity);
	}
}

void Detector::setScanTime()
{
	scan_time = ros::Time::now().toSec();
	ROS_INFO("scan_time = %f \n", scan_time);
}

int Detector::getNumberOfZones()
{
	return zone_counter;
}

float Detector::getCartesianX(float range, float index)
{
	return range*cos(THETA_DELTA*(index - INDEX_SHIFT));
}

float Detector::getCartesianY(float range, float index)
{
	return range*sin(THETA_DELTA*(index - INDEX_SHIFT));
}

void Detector::trackZone(int zone)
{
	zoneBeingTracked = zone;
}

int Detector::getZoneBeingTracked()
{
	return zoneBeingTracked;
}

#include "../../include/detect.h"

std::vector<float> Detector::detectTargets(std::vector<float> ranges)
{
	
	float mean_range = 0.0;
	std::vector<float> tracked_targets (682);
	int mean_index,last_index,place_keeper;
	new_series = 1;
	for (int i = start_index;i<end_index;i++) //adapt to angle of target
	{
		if(ranges[i]<end_range&&ranges[i]>start_range)//adapt to range of target
		{
			//ROS_INFO("GOT A POINT %d", i-start_index);
			if(new_series) //if we're moving on to a new potential target, reset variables
			{
				ROS_INFO("NEW SERIES WOOT");
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
				//ROS_INFO("REACHED MAX TARGET_DEPTH numpoints= %d", num_points);
				last_index = place_keeper+num_points-1;
				if(num_points>num_points_min&&num_points<num_points_max) //check if num_points makes sense
				{
					mean_index = place_keeper + floor(num_points/2);
					mean_range = ranges[mean_index];
						switch(state)
						{
							int zone;
							case INITIALIZING:
								if(abs(ranges[place_keeper-4]-ranges[place_keeper+1])>WALL_DISTANCE_THRESHOLD&&abs(ranges[last_index+4]-ranges[last_index-1])>WALL_DISTANCE_THRESHOLD);
								{
									if(getZone(mean_range,mean_index)==-1) 
									{
										createZone(mean_range,mean_index,-1);//not tracking so we use -1
										initial_targets[mean_index] = ranges[mean_index];
										point_targets = initial_targets;
									}
								}
								break;
							case WAITING_FOR_MOVING_TARGET:
								zone = getZone(mean_range,mean_index);
								if(zone != -1)updateZone(zone,mean_range, mean_index,scan_time);
								point_targets = initial_targets;
								break;
							case TRACKING:
								zone = getZone(mean_range,mean_index);
								ROS_INFO("ZONE OF TARGET = %d", zone);
								if(zone == zoneBeingTracked)
								{	
									tracked_targets[mean_index] = mean_range;
									createZone(mean_range,mean_index,zoneBeingTracked);
									tracked_targets[start_index+1] = start_range;
									tracked_targets[end_index+1] = end_range;
									tracked_targets[end_index] = start_range;
									tracked_targets[start_index] = end_range;
									
									float x = getCartesianX(mean_range, mean_index);
									float y = getCartesianY(mean_range, mean_index);
									tracked_targets[0] = x;
									tracked_targets[1] = y;
								}
								//else{ROS_INFO("NOPE. ZONE = %d\n and zoneBeingTracked = %d",zone,getZoneBeingTracked());}
								tracked_targets[mean_index] = mean_range;
								tracked_targets[start_index-2] = mean_range;
								tracked_targets[end_index+2] = mean_range;
								point_targets = tracked_targets;
								break;
						}
					
					
				}
				//else{ROS_INFO("Target too big or too small");}
				num_points = 0;
				new_series = 1;//start looking for more targets
			}
			
		}
		//else{ROS_INFO("range is too large");}
	}
	//if(state == TRACKING)createZone(mean_range,mean_index,zoneBeingTracked);//this will be the first zone that getZone checks. 
	/*{
		start_index = zone_index_min[zoneBeingTracked];
		end_index = zone_index_max[zoneBeingTracked];
		start_range = zone_range_min[zoneBeingTracked];
		end_range = zone_range_max[zoneBeingTracked];
	}*/
	//ROS_INFO("zoneBeingTracked = %d", zoneBeingTracked);
	//ROS_INFO("start_step = %d\n end_step = %d\n start_range = %f\n end_range = %f\n",start_index, end_index,start_range, end_range);
	
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
	if(zoneBeingTracked==-1)
	{
		for(int i = 0; i<zone_counter;i++)
		{
			if(range>zone_range_min[i]&&range<zone_range_max[i]&&index<zone_index_max[i]&&index>zone_index_min[i]) return i;
			
		}
	}
	else
	{
		if(range>start_range&&range<end_range&&index<end_index&&index>start_index) return zoneBeingTracked;
	}
	return -1;
}

void Detector::createZone(float range,int index,int zoneToTrack)
{
	if(zoneToTrack==-1)
	{
		//ROS_INFO("range = %f \n index = %d\n", range, index);
		possible_target_range.push_back(range);
		possible_target_index.push_back(index);
		possible_target_x.push_back(getCartesianX(range, index));
		possible_target_y.push_back(getCartesianY(range, index));
		possible_target_time.push_back(scan_time);
		zone_index_min.push_back(index - floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_index_max.push_back(index + floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		zone_range_min.push_back(range - MAX_MOVEMENT_PER_SCAN);
		zone_range_max.push_back(range + MAX_MOVEMENT_PER_SCAN);
		previous_zone_distance.push_back(0.0);
		//int i = zone_counter;
		//ROS_INFO("zone_index_min[%d] = %d\n zone_index_max[%d] = %d\n zone_range_min[%d] = %f, zone_range_max[%d] = %f", i,zone_index_min[i],i,zone_index_max[i],i,zone_range_min[i],i,zone_range_max[i]);
		//ROS_INFO("possible target range and index = [%f,%d]", possible_target_range[zone_counter],possible_target_index[zone_counter]);
		//ROS_INFO("possible target [x,y] = [%f,%f]", possible_target_x[zone_counter],possible_target_y[zone_counter]);
		ROS_INFO("TIME OF AQUISITION %f \n", possible_target_time[zone_counter]);
		zone_counter++;
	}
	else
	{
		//zone_index_min[zoneToTrack] = (index - floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		//zone_index_max[zoneToTrack] = (index + floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		//zone_range_min[zoneToTrack] = (range - MAX_MOVEMENT_PER_SCAN);
		//zone_range_max[zoneToTrack] = (range + MAX_MOVEMENT_PER_SCAN);
		start_index = (index - floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		end_index = (index + floor(MAX_MOVEMENT_PER_SCAN/((range-MAX_MOVEMENT_PER_SCAN)*THETA_DELTA)));
		start_range = (range - MAX_MOVEMENT_PER_SCAN);
		end_range = (range + MAX_MOVEMENT_PER_SCAN);
	}
}

void Detector::updateVelocity(int zone,float range, int index, double time)
{
	float tempX = getCartesianX(range, index);
	float tempY = getCartesianY(range, index);
	float distance = sqrt(pow((tempX-possible_target_x[zone]),2)+pow((tempY-possible_target_y[zone]),2));
	float timeBetweenMeasurements = time - possible_target_time[zone];
	float velocity = distance/timeBetweenMeasurements;
	//ROS_INFO("velocity = %f\n", velocity);
	if(velocity>VELOCITY_THRESHOLD_MIN&&velocity<VELOCITY_THRESHOLD_MAX)
	{
		if(zoneBeingTracked == -1)trackZone(zone);
		//ROS_INFO("distance = %f\n start time = %f \n now_time %f\n VELOCITY FOR ZONE %d = %f\n", distance, possible_target_time[zone],scan_time,zone, velocity);
	}
}

void Detector::updateZone(int zone, float range, int index, double time)
{
	float tempX = getCartesianX(range, index);
	float tempY = getCartesianY(range, index);
	float distance = getDistance(tempX, tempY, possible_target_x[zone], possible_target_y[zone]);
	previous_zone_distance[zone] = distance - previous_zone_distance[zone];
	//previous_zone_distance[zone] = distance;
	ROS_INFO("Update zone distance = %f\n", distance);
	if(distance>ZONE_DISTANCE_MIN_THRESHOLD&&previous_zone_distance[zone]<MAX_MOVEMENT_PER_SCAN&&zoneBeingTracked==-1)trackZone(zone);
}

void Detector::setScanTime()
{
	scan_time = ros::Time::now().toSec();
	//ROS_INFO("scan_time = %f \n", scan_time);
}

int Detector::getNumberOfZones()
{
	return zone_counter;
}

float Detector::getCartesianX(float range, int index)
{
	return range*cos(THETA_DELTA*(index - INDEX_SHIFT));
}

float Detector::getCartesianY(float range, int index)
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

bool Detector::isThisAWall(float first_x, float first_y, float mean_x, float mean_y, float last_x, float last_y)
{
	float slope_1 = getSlope(first_x, first_y, mean_x, mean_y);
	float slope_2 = getSlope(mean_x, mean_y, last_x, last_y);
	ROS_INFO("first = [%f,%f] mean = [%f,%f] \n last = [%f,%f] \n slope_1 = %f \n slope_2 = %f \n",first_x, first_y, mean_x, mean_y, last_x, last_y,slope_1, slope_2);
	return ((abs(slope_1-slope_2)<WALL_SLOPE_THRESHOLD)? true : false);
}

float Detector::getSlope(float x_1, float y_1, float x_2, float y_2)
{
	return ((y_1-y_2)/(x_1-x_2));
}

float Detector::getDistance(float x_1, float y_1, float x_2, float y_2)
{
	return sqrt(pow((x_1-x_2),2)+pow((y_1-y_2),2));
}

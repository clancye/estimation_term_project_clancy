#include "../../include/track.h"
#include <random>


int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "client_detect_node");
	ros::NodeHandle n;
	ros::ServiceClient client_update = n.serviceClient<laser_package::update_tracker>("updateTracker");
	ros::ServiceClient client_initialize = n.serviceClient<laser_package::update_tracker>("initializeTracker");
	laser_package::update_tracker srv;
	
	std::default_random_engine measurement_generator, process_generator;
	std::normal_distribution<double> meas_noise(MU_W,VAR_W),process_noise(MU_V,VAR_V);
	
	float x,last_x, x_vel, last_x_vel;
	float y,last_y, y_vel, last_y_vel;
	double w,v;
	
	ros::Rate r(T);//T is the sampling interval defined in track.h
	
	last_x = 0;
	last_x_vel = 10;
	last_y = 0;
	last_y_vel = 10;
	
	srv.request.initial_x = last_x;
	srv.request.initial_x_velocity = last_x_vel;
	srv.request.initial_y = last_y;
	srv.request.initial_y_velocity = last_y_vel;
	srv.request.update_time = ros::Time::now().toSec();
	
	client_initialize.call(srv);
	
	while(ros::ok())
	{
		w = meas_noise(measurement_generator);//noise
		v = process_noise(process_generator);
		x = last_x + T*last_x_vel+pow(T,2)*v/2;//perfect model, no noise assumptions
		x_vel = last_x_vel + T*v;//system velocity evolution
		srv.request.real_x = x;
		srv.request.measured_x = x + w;//measurement value to send
		y = last_y + T*last_y_vel+pow(T,2)*v/2;//perfect model, no noise assumptions
		y_vel = last_y_vel + T*v;//system velocity evolution
		srv.request.real_y = y;
		srv.request.measured_y = y + w;//measurement value to send
		srv.request.update_time = ros::Time::now().toSec();
		if(client_update.call(srv));
		//else{ROS_INFO("X_meas = %f\n", srv.request.measured_x);}
		last_x = x;
		last_x_vel = x_vel;
		last_y = y;
		last_y_vel = y_vel;
		
		ros::spinOnce();
		
		r.sleep();
	}
	
	
	
	

	return 0;
}



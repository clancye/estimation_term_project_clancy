

#include <ros/ros.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "track_node");
	ros::NodeHandle n;

	ros::Publisher target_pub = n.advertise<sensor_msgs::LaserScan>("target_topic",1000);

	ros::Rate loop_rate(10);
	geometry_msgs::Pose2D msg;
	while ( ros::ok())
	{

		msg.x = 3.0;
		msg.y = 4.0;
		target_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();

		
	}

	return 0;

}


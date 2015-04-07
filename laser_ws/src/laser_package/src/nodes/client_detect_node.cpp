#include "../../include/detect.h"

std::vector<float> detectTargets(std::vector<float> ranges);

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
 	{
		laser_sub = n.subscribe<sensor_msgs::LaserScan>("/client_scan",1000,&SubscribeAndPublish::laserCallBack,this);
		target_pub = n.advertise<sensor_msgs::LaserScan>("/client_targets",1000); //publish targets to new topic
        ROS_INFO("Constructing SAP");
	}

	
	void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
	{ 
        sensor_msgs::LaserScan targets_msg;
        targets_msg = *msg;
		std::vector<float> filtered_ranges (682);
		filtered_ranges = detectTargets(msg->ranges);
		targets_msg.ranges = filtered_ranges;
		target_pub.publish(targets_msg);
	}
private:
   
    ros::NodeHandle n; 
    ros::Publisher target_pub;
    ros::Subscriber laser_sub;

};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "client_detect_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPDetect;

  ros::spin();

  return 0;
}



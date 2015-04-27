#include "../../include/imm.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			add_filter_to_IMM = n.advertiseService("addFilter", &SubscribeAndPublish::addFilterCallBack,this);
			filter_count = 0;
		}
		
		bool addFilterCallBack(laser_package::add_filter::Request &req, laser_package::add_filter::Response &res)
		{
			if(req.addMe)res.filterID = filter_count;
			filter_count++;
			ROS_INFO("Adding a filter");
			return true;
		}
		
	private:
	ros::NodeHandle n;
	ros::ServiceServer add_filter_to_IMM;
	int filter_count;
	
		
	};


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "imm_node");
	SubscribeAndPublish SAPimm;
	
	ros::spin();
	
	
	
	

	return 0;
}



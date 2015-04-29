#include "../../include/imm.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			add_filter_to_IMM = n.advertiseService("addFilter", &SubscribeAndPublish::addFilterCallBack,this);
			update_imm_filter = n.advertiseService("updateIMMFilter", &SubscribeAndPublish::updateIMMFilterCallBack,this);
			filter_count = 0;
		}
		
		bool addFilterCallBack(laser_package::add_filter::Request &req, laser_package::add_filter::Response &res)
		{
			if(req.addMe)res.filterID = filter_count;
			filter_count++;
			ROS_INFO("Adding a filter");
			update_imm_srv.request.xi_j = 0.1;
			updateIMMFilterCallBack(update_imm_srv.request, update_imm_srv.response);
			return true;
		}
		
		bool updateIMMFilterCallBack(laser_package::update_imm_filter::Request &req, laser_package::update_imm_filter::Response &res)
		{
			ROS_INFO("Updating a filter, x_i = %f", req.xi_j);
			return true;
		}
		
	private:
	ros::NodeHandle n;
	laser_package::update_imm_filter update_imm_srv;
	ros::ServiceServer add_filter_to_IMM, update_imm_filter;
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



#include "../../include/imm.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			add_filter_to_IMM = n.advertiseService("addFilter", &SubscribeAndPublish::addFilterCallBack,this);
			update_imm_filter = n.advertiseService("updateIMMFilter", &SubscribeAndPublish::updateIMMFilterCallBack,this);
			initialize_an_imm_filter = n.advertiseService("initializeIMMFilter", &SubscribeAndPublish::initializeIMMFilterCallBack,this);
			update_filter = n.serviceClient<laser_package::update_filter_priors>("updateFilterPriors");
			filter_count = 0;
		}
		
		bool addFilterCallBack(laser_package::add_filter::Request &req, laser_package::add_filter::Response &res)
		{
			if(req.addMe)res.filterID = filter_count;
			filter_count++;
			ROS_INFO("Adding a filter");
			update_imm_srv.request.xi_j = 0.1;
			if(updateIMMFilterCallBack(update_imm_srv.request, update_imm_srv.response))
			{
				ROS_INFO("update imm filter call back returned true");
				return true;
			}
		}
		
		bool updateIMMFilterCallBack(laser_package::update_imm_filter::Request &req, laser_package::update_imm_filter::Response &res)
		{
			ROS_INFO("Updating a filter, x_i = %f", req.xi_j);
			priors_update.request.xi_0j = 0.5555;
			return update_filter.call(priors_update);
		}
		
		bool initializeIMMFilterCallBack(laser_package::update_imm_filter::Request &req, laser_package::update_imm_filter::Response &res)
		{
			ROS_INFO("initializing a filter, should be 0.23232323 x_i = %f", req.xi_j);
			priors_update.request.xi_0j = 0.5555;
			update_filter.call(priors_update);
			return true;
		}
		
		
	private:
	ros::NodeHandle n;
	laser_package::update_imm_filter update_imm_srv;
	ros::ServiceServer add_filter_to_IMM, update_imm_filter, initialize_an_imm_filter;
	ros::ServiceClient update_filter;
	laser_package::update_filter_priors priors_update;
	int filter_count;
	
		
	};


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "imm_node");
	SubscribeAndPublish SAPimm;
	ros::Rate r(100);
	ros::AsyncSpinner spinner(2);
	while(ros::ok())
	{
		spinner.start();
		//ros::spinOnce();
		//r.sleep();
	}
	
	spinner.stop();
	
	
	

	return 0;
}



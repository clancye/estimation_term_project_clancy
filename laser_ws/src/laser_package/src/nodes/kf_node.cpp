#include "../../include/kf.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{	
			ROS_INFO("Constructing SAP for tracking node...");
			kf = Filter();
			target_sub = n.subscribe<laser_package::state>("/filter_topic",1000,&SubscribeAndPublish::targetCallBack,this);
			target_pub = n.advertise<laser_package::state>("/kf_topic",1000); //publish targets to new topic
			add_filter_to_IMM = n.serviceClient<laser_package::add_filter>("addFilter");
			srv.request.addMe = 1;
			if(add_filter_to_IMM.call(srv))
			{
				filterID = srv.response.filterID;
				ROS_INFO("Here is my number: %d", srv.response.filterID);
			}
		}
		
		void targetCallBack(const laser_package::state::ConstPtr& msg)
		{
			state_msg.Measured_X = msg->Measured_X;
			target_pub.publish(state_msg);
			ROS_INFO("xi = %f", msg->Measured_X);
		}
	private:
	ros::NodeHandle n;
	ros::Subscriber target_sub;
	ros::Publisher target_pub;
	ros::ServiceClient add_filter_to_IMM;
	laser_package::state state_msg;
	Filter kf;
	int filterID;
	laser_package::add_filter srv;

	
		
	};


int main(int argc, char **argv)
{
	//ROS stuff
	ros::init(argc, argv, "kf_node");
	SubscribeAndPublish SAPekf;
	
	ros::spin();
	
	
	
	

	return 0;
}



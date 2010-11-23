// Client = Sender

#include "ros/ros.h"					// includes all the headers necessary to use all the public pieces of the ROS system
#include "rviz_example/SetMarker.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_get");		// initialize ROS
	ros::NodeHandle n;				// create a handle to this process node
        ros::ServiceClient client = n.serviceClient<rviz_example::SetMarker>("set_marker");
	
	int32_t xpos=4;
	//int32_t xpos=0;
	//int32_t xpos=0;
	
        rviz_example::SetMarker srv;
	
// By default roscpp will install a SIGINT handler which provides Ctrl-C handling. ros::ok() will return false if this happens, or if we have been kicked off the network by another node with the same name, or if ros::shutdown() has been called by another part of the application. Once ros::ok() returns false, all ROS calls will fail.
	while (ros::ok()) {
		xpos = ((xpos+1)%31);	
		srv.request.x = xpos-15;
		srv.request.y = xpos-15;
		srv.request.z = xpos-15;

		if (client.call(srv))
		{
		  ROS_INFO("Calling Successful %i", xpos);
		}
		else
		{
			ROS_ERROR("Failed to call service SetMarker");
			return 1;
		}
	ros::Duration(0.05).sleep();
	}

	return 0;
}

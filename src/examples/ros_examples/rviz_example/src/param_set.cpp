#include <ros/ros.h>				// includes all the headers necessary to use all the public pieces of the ROS system

int main( int argc, char** argv )
{
  ros::init(argc, argv, "param_set");		// initialize ROS
  ros::NodeHandle n;				// create a handle to this process node
  int32_t xpos = 0;
  int32_t signx = 1;

// By default roscpp will install a SIGINT handler which provides Ctrl-C handling. ros::ok() will return false if this happens, or if we have been kicked off the network by another node with the same name, or if ros::shutdown() has been called by another part of the application. Once ros::ok() returns false, all ROS calls will fail.
  while (ros::ok())
  {

    if (xpos>4){
      signx = -1;
    }
   
    if (xpos<-4) {
      signx =1;
    }

    xpos = xpos + signx*1;
    n.setParam("position_x", xpos);
    ros::Duration(1.0).sleep();
  }
}

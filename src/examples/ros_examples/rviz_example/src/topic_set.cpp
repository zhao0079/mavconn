#include <ros/ros.h>				// includes all the headers necessary to use all the public pieces of the ROS system
#include <std_msgs/Int32.h>
//#include <sstream>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "topic_set");		// initialize ROS
  ros::NodeHandle n;				// create a handle to this process node
  ros::Publisher position_pub =n.advertise<std_msgs::Int32>("position",100);
  ros::Rate loop_rate(24);



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
    
    std_msgs::Int32 msg;
    msg.data = xpos;
    position_pub.publish(msg);
    // ROS_INFO("I published [%s]", ss.str().c_str());
    // ros::spinOnce();
    loop_rate.sleep();




  }
}

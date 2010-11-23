#include <ros/ros.h>				// includes all the headers necessary to use all the public pieces of the ROS system
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "param_get");		// initialize ROS
  ros::NodeHandle n;				// create a handle to this process node
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  int32_t xpos=0;

// By default roscpp will install a SIGINT handler which provides Ctrl-C handling. ros::ok() will return false if this happens, or if we have been kicked off the network by another node with the same name, or if ros::shutdown() has been called by another part of the application. Once ros::ok() returns false, all ROS calls will fail.
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "moving_marker";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = xpos;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

 
    n.getParam("/position_x", xpos);


    ros::Duration(1.0).sleep();
  }
}

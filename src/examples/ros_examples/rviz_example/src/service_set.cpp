#include <ros/ros.h>				// includes all the headers necessary to use all the public pieces of the ROS system
#include <visualization_msgs/Marker.h>
#include "rviz_example/SetMarker.h"
//#include <std_msgs/Int32.h>
//#include <stdio.h>
//#include <iostream.h>

int32_t xpos=0;
int32_t ypos=0;
int32_t zpos=0; 

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

bool setm(rviz_example::SetMarker::Request &req,
          rviz_example::SetMarker::Response &res)
{
  xpos = req.x;
  ypos = req.y;
  zpos = req.z;
  res.done = true;
 
  marker.pose.position.x = xpos;
  marker.pose.position.y = ypos;
  marker.pose.position.z = zpos;
  marker_pub.publish(marker);
     
  return true;
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "service_set");		// initialize ROS

  ros::NodeHandle m;				// create a handle to this process node
  ros::ServiceServer service = m.advertiseService("set_marker", setm);

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  ROS_INFO("Ready to draw.");
     marker.header.frame_id = "/my_frame";
     marker.header.stamp = ros::Time();
     marker.ns = "moving_marker";
     marker.id = 0;
     marker.type = shape;
     marker.action = visualization_msgs::Marker::ADD;

     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     marker.scale.x = 1.0;
     marker.scale.y = 1.0;
     marker.scale.z = 1.0;
     marker.color.r = 1.0f;
     marker.color.g = 0.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     marker.lifetime = ros::Duration();

  ros::spin();
}

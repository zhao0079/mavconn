/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

(c) 2009, 2010 MAVCONN PROJECT

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/


/**
* @file
*   @brief ROS Visualization of the competition environment
*
*   @author Bastian Buecheler <mavteam@student.ethz.ch>
*   @author Christian Schluchter <schluchc@ee.ethz.ch>
*
*/

// Standard includes
#include <stdio.h>
#include <boost/program_options.hpp>


// MAVCONN includes
#include "mavconn.h"
#include <lcm/lcm.h>
#include "comm/lcm/mavlink_message_t.h"

// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

namespace config = boost::program_options;



bool debug; 					///< Enable debug functions and output
bool verbose; 					///< Enable verbose output
bool help;						///< Enable help output (legend)

float const pi = 3.14159;




int main (int argc, char ** argv) {
	// SET UP PROGRAM OPTIONS --------------------------------------------------------------------
	config::options_description desc("Allowed options");
	desc.add_options()
								("help", config::bool_switch(&help)->default_value(false), "produce help message (including legend of the visualization)")
								("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
								("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
								;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);
	//--------------------------------------------------------------------------------------------
	// ROS node initialization
	ros::init(argc, argv, "px_house");		// initialize ROS
	ros::NodeHandle n;							// create a handle to this process node





	/*--------------------------------------------------------------------------------------------
	 * Dimensions and Positions
	 */

	// Origin of global world is starting point
	//Wall
	float wall_pos_x = 3;
	float wall_pos_y = 2;
	float wall_pos_z = 0;
	float wall_dir_yaw = 0;
	float wall_dim_x = 0.1;
	float wall_dim_y = 10;
	float wall_dim_z = 2.5;

	//House center
	float house_pos_x = 10;
	float house_pos_y = 3;
	float house_pos_z = 0;
	float house_dir_yaw = 0;

	float house_dim_x = 4;
	float house_dim_y = 3;
	float house_dim_z = 3;
	float house_door_offset_y = 0.75;
	float house_door_dim_y = 1.5;
	float house_door_dim_z = 2;
	float house_win_offset_x = 0.75;
	float house_win_offset_z = 0.75;
	float house_win_dim_x = 1.5;
	float house_win_dim_z = 1.5;
	float house_wall_thick = 0.01;
	float house_chim_dim_x = 1;
	float house_chim_dim_z = 1;
	float house_chim_offset_x = 3;
	float house_chim_offset_y = 2;

	//Landing point
	float land_pos_x = 0;
	float land_pos_y = 10;
	float land_pos_z = 0;
	float land_dir_yaw = 0;
	float land_dim_x = 1;
	float land_dim_y = 1;
	float land_dim_z = house_wall_thick;
	//--------------------------------------------------------------------------------------------


	/*
	 * Transformations
	 */
	static tf::TransformBroadcaster br;
	// Frame-Transformation from world frame (my_frame, x forward, z up, cog 0) to navigation frame (x forward, z down, cog 0)


	tf::Transform transform_world_nav;
	transform_world_nav.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion world_nav_rot = tf::createQuaternionFromRPY(pi, 0, 0);
	transform_world_nav.setRotation(world_nav_rot);
	br.sendTransform(tf::StampedTransform(transform_world_nav, ros::Time::now(), "my_frame", "navi_frame"));		// my_frame = world


	tf::Transform transform_nav_wall;
	transform_nav_wall.setOrigin(tf::Vector3(wall_pos_x,wall_pos_y,wall_pos_z));
	tf::Quaternion nav_wall_rot = tf::createQuaternionFromRPY(0, 0, wall_dir_yaw);
	transform_nav_wall.setRotation(nav_wall_rot);
	br.sendTransform(tf::StampedTransform(transform_nav_wall, ros::Time::now(), "navi_frame", "wall_frame"));

	tf::Transform transform_nav_house;
	transform_nav_house.setOrigin(tf::Vector3(house_pos_x,house_pos_y,house_pos_z));
	tf::Quaternion nav_house_rot = tf::createQuaternionFromRPY(0, 0, house_dir_yaw);
	transform_nav_house.setRotation(nav_house_rot);
	br.sendTransform(tf::StampedTransform(transform_nav_house, ros::Time::now(), "navi_frame", "house_frame"));

	tf::Transform transform_nav_land;
	transform_nav_land.setOrigin(tf::Vector3(land_pos_x,land_pos_y,land_pos_z));
	tf::Quaternion nav_land_rot = tf::createQuaternionFromRPY(0, 0, land_dir_yaw);
	transform_nav_land.setRotation(nav_land_rot);
	br.sendTransform(tf::StampedTransform(transform_nav_land, ros::Time::now(), "navi_frame", "land_frame"));

	usleep(1000000);

	// Setting markers---------------------------------------------------------------------------------------------
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("house_markers", 0);
	visualization_msgs::Marker marker;
	uint32_t cyl = visualization_msgs::Marker::CYLINDER;
	uint32_t cube = visualization_msgs::Marker::CUBE;


	// properties of the default marker
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.type = cube;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1.0;

	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	// ------------------------------------------------------------------------------------------------------------

	visualization_msgs::Marker wall_marker = marker;
	visualization_msgs::Marker land_marker = marker;
	visualization_msgs::Marker house_marker1 = marker;
	house_marker1.header.frame_id = "/house_frame";
	house_marker1.ns = "house";
	house_marker1.id = 1;
	house_marker1.pose.position.z = -house_dim_z/2;
	visualization_msgs::Marker house_marker2 = house_marker1;
	house_marker1.id = 2;
	visualization_msgs::Marker house_marker3 = house_marker1;
	house_marker1.id = 3;
	visualization_msgs::Marker house_marker4 = house_marker1;
	house_marker1.id = 4;
	visualization_msgs::Marker house_marker5 = house_marker1;
	house_marker1.id = 5;
	visualization_msgs::Marker house_marker6 = house_marker1;
	house_marker1.id = 6;
	visualization_msgs::Marker house_marker7 = house_marker1;
	house_marker1.id = 7;
	visualization_msgs::Marker house_marker8 = house_marker1;
	house_marker1.id = 8;
	visualization_msgs::Marker house_marker9 = house_marker1;
	house_marker1.id = 9;
	visualization_msgs::Marker house_marker10 = house_marker1;
	house_marker1.id = 10;

	wall_marker.header.frame_id = "/wall_frame";
	wall_marker.ns = "wall";
	wall_marker.pose.position.z = -wall_dim_z/2;
	wall_marker.scale.x = wall_dim_x;
	wall_marker.scale.y = wall_dim_y;
	wall_marker.scale.z = wall_dim_z;

	land_marker.header.frame_id = "/land_frame";
	land_marker.ns = "land";
	land_marker.scale.x = land_dim_x;
	land_marker.scale.y = land_dim_y;
	land_marker.scale.z = land_dim_z;

	visualization_msgs::Marker start_marker = land_marker;
	start_marker.header.frame_id = "/navi_frame";
	start_marker.ns = "start";

	house_marker1.scale.x = house_wall_thick;
	house_marker1.scale.y = house_door_offset_y;
	house_marker1.scale.z = house_dim_z;
	house_marker1.pose.position.x = -house_dim_x/2;
	house_marker1.pose.position.y = -house_dim_y/2+house_marker1.scale.y/2;
	house_marker1.pose.position.z = -house_dim_z/2;

	house_marker2.scale.x = house_wall_thick;
	house_marker2.scale.y = house_door_dim_y;
	house_marker2.scale.z = house_dim_z-house_door_dim_z;
	house_marker2.pose.position.x = -house_dim_x/2;
	house_marker2.pose.position.y = -house_dim_y/2+house_door_offset_y+house_door_dim_y/2;
	house_marker2.pose.position.z = -house_door_dim_z-house_marker2.scale.z/2;

	house_marker3.scale.x = house_wall_thick;
	house_marker3.scale.y = house_dim_y-house_door_offset_y-house_door_dim_y;
	house_marker3.scale.z = house_dim_z;
	house_marker3.pose.position.x = -house_dim_x/2;
	house_marker3.pose.position.y = house_dim_y/2-house_marker3.scale.y/2;
	house_marker3.pose.position.z = -house_dim_z/2;

	house_marker4.scale.x = house_win_offset_x;
	house_marker4.scale.y = house_wall_thick;
	house_marker4.scale.z = house_dim_z;
	house_marker4.pose.position.x = -house_dim_x/2+house_win_offset_x/2;
	house_marker4.pose.position.y = -house_dim_y/2;
	house_marker4.pose.position.z = -house_dim_z/2;

	house_marker5.scale.x = house_win_dim_x;
	house_marker5.scale.y = house_wall_thick;
	house_marker5.scale.z = house_dim_z-house_win_dim_z-house_win_offset_z;
	house_marker5.pose.position.x = -house_dim_x/2+house_win_offset_x+house_win_dim_x/2;
	house_marker5.pose.position.y = -house_dim_y/2;
	house_marker5.pose.position.z = -house_win_offset_z-house_win_dim_z-house_marker5.scale.z/2;

	house_marker6.scale.x = house_win_dim_x;
	house_marker6.scale.y = house_wall_thick;
	house_marker6.scale.z = house_win_offset_z;
	house_marker6.pose.position.x = -house_dim_x/2+house_win_offset_x+house_win_dim_x/2;
	house_marker6.pose.position.y = -house_dim_y/2;
	house_marker6.pose.position.z = -house_win_offset_z/2;

	house_marker7.scale.x = house_dim_x-house_win_offset_x-house_win_dim_x;
	house_marker7.scale.y = house_wall_thick;
	house_marker7.scale.z = house_dim_z;
	house_marker7.pose.position.x = -house_dim_x/2+house_win_offset_x+house_win_dim_x+house_marker7.scale.x/2;
	house_marker7.pose.position.y = -house_dim_y/2;
	house_marker7.pose.position.z = -house_dim_z/2;

	house_marker8.scale.x = house_wall_thick;
	house_marker8.scale.y = house_dim_y;
	house_marker8.scale.z = house_dim_z;
	house_marker8.pose.position.x = house_dim_x/2;
	house_marker8.pose.position.y = 0;
	house_marker8.pose.position.z = -house_dim_z/2;

	house_marker9.scale.x = house_dim_x;
	house_marker9.scale.y = house_wall_thick;
	house_marker9.scale.z = house_dim_z;
	house_marker9.pose.position.x = 0;
	house_marker9.pose.position.y = house_dim_y/2;
	house_marker9.pose.position.z = -house_dim_z/2;


	house_marker10.type = cyl;
	house_marker10.scale.x = house_chim_dim_x;
	house_marker10.scale.y = house_chim_dim_x;
	house_marker10.scale.z = house_chim_dim_z;
	house_marker10.pose.position.x = -house_dim_x/2+house_chim_offset_x;
	house_marker10.pose.position.y = -house_dim_y/2+house_chim_offset_y;
	house_marker10.pose.position.z = -house_dim_z-house_chim_dim_z/2;


	ros::Rate loop_rate(1000);
	// By default roscpp will install a SIGINT handler which provides Ctrl-C handling. ros::ok() will return false if this happens, or if we have been kicked off the network by another node with the same name, or if ros::shutdown() has been called by another part of the application. Once ros::ok() returns false, all ROS calls will fail.
	while (ros::ok())
	{

		transform_world_nav.setOrigin(tf::Vector3(0,0,0));
		transform_world_nav.setRotation(world_nav_rot);
		br.sendTransform(tf::StampedTransform(transform_world_nav, ros::Time::now(), "my_frame", "navi_frame"));		// my_frame = world

		transform_nav_wall.setOrigin(tf::Vector3(wall_pos_x,wall_pos_y,wall_pos_z));
		transform_nav_wall.setRotation(nav_wall_rot);
		br.sendTransform(tf::StampedTransform(transform_nav_wall, ros::Time::now(), "navi_frame", "wall_frame"));

		transform_nav_house.setOrigin(tf::Vector3(house_pos_x,house_pos_y,house_pos_z));
		transform_nav_house.setRotation(nav_house_rot);
		br.sendTransform(tf::StampedTransform(transform_nav_house, ros::Time::now(), "navi_frame", "house_frame"));

		transform_nav_land.setOrigin(tf::Vector3(land_pos_x,land_pos_y,land_pos_z));
		transform_nav_land.setRotation(nav_land_rot);
		br.sendTransform(tf::StampedTransform(transform_nav_land, ros::Time::now(), "navi_frame", "land_frame"));

		marker_pub.publish(wall_marker);
		marker_pub.publish(start_marker);
		marker_pub.publish(land_marker);
		marker_pub.publish(house_marker1);
		marker_pub.publish(house_marker2);
		marker_pub.publish(house_marker3);
		marker_pub.publish(house_marker4);
		marker_pub.publish(house_marker5);
		marker_pub.publish(house_marker6);
		marker_pub.publish(house_marker7);
		marker_pub.publish(house_marker8);
		marker_pub.publish(house_marker9);
		marker_pub.publish(house_marker10);
		loop_rate.sleep();

	}
	n.shutdown();
	ros::shutdown();
	return 0;
}

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
*   @brief ROS Visualization of MAVCONN and Waypoints
*
*   @author Bastian Buecheler <mavteam@student.ethz.ch>
*   @author Christian Schluchter <mavteam@student.ethz.ch>
*
*/


#include <stdio.h>
#include "mavconn.h"

#include <lcm/lcm.h>
#include "comm/lcm/mavlink_message_t.h"

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

bool debug = true; 					///< Enable debug functions and output
lcm_t* lcm;
int sysid = 99;
int compid = 25;


int main (int argc, char ** argv) {
	lcm = lcm_create (NULL);
	if (!lcm)
	{
		printf("\nCouldn't start LCM link, aborting\n");
		return 1;
	}



	mavlink_message_t msg;
	// send position a few times
	for (int i = 0; i<= 5; i++)
	{
		mavlink_msg_local_position_pack(sysid, compid, &msg, 0, 1, 0, -1, 0, 0, 0);
		mavlink_message_t_publish (lcm, "MAVLINK", &msg);
		printf("Sent position\n");
		usleep(100000);
	}
/*
	float yaw = 0;
	bool autocontinue = true;
	bool current = true;
	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,0, 1, 1, 1, current, 2, 0, -1, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 0);
	usleep(100000);

	current = false;
	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,1, 1, 1, 1, current, 2, 0, -3, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 1);
	usleep(100000);

	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,2, 1, 1, 1, current, 4, 0, -3, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 2);
	usleep(100000);

	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,3, 1, 1, 1, current, 5, 1, -1, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 3);
	usleep(100000);

	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,4, 1, 1, 1, current, 7, 3, -1, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 4);
	usleep(100000);

	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,5, 1, 1, 1, current, 10, 3, -1, yaw, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 5);
	usleep(100000);

	autocontinue = false;
	mavlink_msg_waypoint_pack(sysid, compid, &msg, 1, 1,6, 1, 1, 1, current, 10, 3, 0, -180, autocontinue);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent waypoint %i\n", 6);
	usleep(100000);

	mavlink_msg_local_position_setpoint_pack(sysid, compid, &msg, 1.5, 0, -1, yaw);
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);
	printf("Sent setpoint \n");
	usleep(10000);
*/

}

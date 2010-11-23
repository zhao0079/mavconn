/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

(c) 2009, 2010 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/


/**
 * @file
 *   @brief ROS Visualization of PixHawk, Waypoints, etc.
 *
 *   @author Bastian Buecheler <mavteam@student.ethz.ch>
 *   @author Christian Schluchter <schluchc@ee.ethz.ch>
 *
 */



// Standard includes
#include <stdio.h>

//boost
#include <boost/program_options.hpp>

//threads
#include "glib.h"


// Pixhawk includes
#include "mavconn.h"
#include <lcm/lcm.h>
#include "comm/lcm/mavlink_message_t.h"

// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

namespace config = boost::program_options;



bool debug; 					///< Enable debug functions and output
bool verbose; 					///< Enable verbose output
bool help;						///< Enable help output (legend)

float const pi = 3.14159;

// dimensions of quadrocopter
float const base_height = 0.1;
float const base_width = 0.1;
float const cam_height = 0.05;
float const cam_width = 0.1;
float const cam_length = 0.2;
float const prop_height = 0.02;
float const prop_diam = 0.2;
float const prop_radius = prop_diam/2;

//marker dimensions
float const markersize = 0.2;

// Initialization of the markers
ros::Publisher marker_pub;

visualization_msgs::Marker marker;

visualization_msgs::Marker px_marker_c;		// middle part
visualization_msgs::Marker px_marker_f;		// front part
visualization_msgs::Marker px_marker_b;		// back part
visualization_msgs::Marker px_marker_l;		// left part
visualization_msgs::Marker px_marker_r;		// right part
visualization_msgs::Marker px_marker_cam;	// camera part

visualization_msgs::Marker px_est_marker_c;		// middle part (estimated)
visualization_msgs::Marker px_est_marker_f;		// front part (estimated)
visualization_msgs::Marker px_est_marker_b;		// back part (estimated)
visualization_msgs::Marker px_est_marker_l;		// left part (estimated)
visualization_msgs::Marker px_est_marker_r;		// right part (estimated)
visualization_msgs::Marker px_est_marker_cam;	// camera part (estimated)

visualization_msgs::Marker px_mult_marker_c;		// middle part (multitracker)
visualization_msgs::Marker px_mult_marker_f;		// front part (multitracker)
visualization_msgs::Marker px_mult_marker_b;		// back part (multitracker)
visualization_msgs::Marker px_mult_marker_l;		// left part (multitracker)
visualization_msgs::Marker px_mult_marker_r;		// right part (multitracker)
visualization_msgs::Marker px_mult_marker_cam;	// camera part (multitracker)

visualization_msgs::Marker wp_marker_c;		// waypoint center
visualization_msgs::Marker wp_marker_dir;	// waypoint direction

visualization_msgs::Marker setpoint_marker_c;	// control setpoint marker center
visualization_msgs::Marker setpoint_marker_dir;	// control setpoint marker direction

visualization_msgs::Marker marker_marker;	// art marker

std::vector<mavlink_waypoint_t> waypoints;   ///< vector that holds the waypoints (needed to set a previously drawn waypoint active)


void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		if (debug) printf("Waiting for LCM data\n");
		lcm_handle (lcm);
	}
	return NULL;
}



static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user) {

	if (verbose) {
		printf("Received message on channel \"%s\":\n", channel);
	}

	/*
	 * TF Frame Transformations
	 */
	static tf::TransformBroadcaster br;

	// Frame-Transformation from world frame (my_frame, x forward, z up, cog 0) to navigation frame (x forward, z down, cog 0)
	tf::Transform transform_world_nav;
	transform_world_nav.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion world_nav_rot = tf::createQuaternionFromRPY(pi, 0, 0);
	transform_world_nav.setRotation(world_nav_rot);
	br.sendTransform(tf::StampedTransform(transform_world_nav, ros::Time::now(), "my_frame", "navi_frame"));		// my_frame = world

	// Frame-Transformation from navigation frame to body frame (x forward, z down, origin cog)
	tf::Transform transform_nav_body;
	tf::Quaternion world_nav_body_rot;

	// Frame-Transformation from navigation frame to body_est frame (x forward, z down, origin cog)
	tf::Transform transform_nav_body_est;
	tf::Quaternion world_nav_body_est_rot;

	// Frame-Transformation from navigation frame to body_multi frame (x forward, z down, origin cog)
	tf::Transform transform_nav_body_mult;
	tf::Quaternion world_nav_body_mult_rot;




	/*
	 * Mavlink Message Handling
	 */
	switch(msg->msgid) {
	/*case MAVLINK_MSG_ID_ATTITUDE:
		// FIXME: 	component ID of IMU
		// decode message only if sent by the right component
//		if (msg->compid == PX_COMP_ID_IMU) {			// Filter Component-ID imu, ALTERNATIVE : PX_COM_ID_ASCTEC
			mavlink_attitude_t att;
			mavlink_msg_attitude_decode(msg, &att);
			if (verbose) {
				printf("Received attitude message: \n");
				printf("roll:% f\n", att.roll);
				printf("pitch:% f\n", att.pitch);
				printf("yaw:% f\n", att.yaw);
			}
			world_nav_body_rot = tf::createQuaternionFromRPY(att.roll, att.pitch, att.yaw);
			transform_nav_body.setRotation(world_nav_body_rot);
			br.sendTransform(tf::StampedTransform(transform_nav_body, ros::Time::now(), "navi_frame", "pixhawk"));

			px_marker_c.header.stamp = ros::Time::now();
			px_marker_f.header.stamp = ros::Time::now();
			px_marker_b.header.stamp = ros::Time::now();
			px_marker_l.header.stamp = ros::Time::now();
			px_marker_r.header.stamp = ros::Time::now();
			px_marker_cam.header.stamp = ros::Time::now();

			px_marker_c.header.frame_id = "/pixhawk";
			px_marker_f.header.frame_id = "/pixhawk";
			px_marker_b.header.frame_id = "/pixhawk";
			px_marker_l.header.frame_id = "/pixhawk";
			px_marker_r.header.frame_id = "/pixhawk";
			px_marker_cam.header.frame_id = "/pixhawk";

			marker_pub.publish(px_marker_c);
			marker_pub.publish(px_marker_f);
			marker_pub.publish(px_marker_b);
			marker_pub.publish(px_marker_l);
			marker_pub.publish(px_marker_r);
			marker_pub.publish(px_marker_cam);
//		}
		break;*/
	case MAVLINK_MSG_ID_LOCAL_POSITION:
			mavlink_local_position_t pos;
			mavlink_msg_local_position_decode(msg, &pos);
			if (verbose) {
				printf("Received local position message: \n");
				printf("x-pos:% f\n", pos.x);
				printf("y-pos:% f\n", pos.y);
				printf("z-pos:% f\n", pos.z);
				// printf("roll:% f\n", pos.roll);
				// printf("pitch:% f\n", pos.pitch);
				// printf("yaw:% f\n", pos.yaw);
			}
			transform_nav_body.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
			// world_nav_body_rot = tf::createQuaternionFromRPY(pos.roll, pos.pitch, pos.yaw);
			transform_nav_body.setRotation(world_nav_body_rot);
			br.sendTransform(tf::StampedTransform(transform_nav_body, ros::Time::now(), "navi_frame", "pixhawk"));

			px_marker_c.header.stamp = ros::Time::now();
			px_marker_f.header.stamp = ros::Time::now();
			px_marker_b.header.stamp = ros::Time::now();
			px_marker_l.header.stamp = ros::Time::now();
			px_marker_r.header.stamp = ros::Time::now();
			px_marker_cam.header.stamp = ros::Time::now();

			px_marker_c.header.frame_id = "/pixhawk";
			px_marker_f.header.frame_id = "/pixhawk";
			px_marker_b.header.frame_id = "/pixhawk";
			px_marker_l.header.frame_id = "/pixhawk";
			px_marker_r.header.frame_id = "/pixhawk";
			px_marker_cam.header.frame_id = "/pixhawk";

			marker_pub.publish(px_marker_c);
			marker_pub.publish(px_marker_f);
			marker_pub.publish(px_marker_b);
			marker_pub.publish(px_marker_l);
			marker_pub.publish(px_marker_r);
			marker_pub.publish(px_marker_cam);

		break;
	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		if (msg->compid == PX_COMP_ID_MULTITRACKER) {
			mavlink_vision_position_estimate_t pos_mult;
			mavlink_msg_vision_position_estimate_decode(msg, &pos_mult);
			transform_nav_body_mult.setOrigin(tf::Vector3(pos_mult.x, pos_mult.y, pos_mult.z));
			world_nav_body_mult_rot = tf::createQuaternionFromRPY(pos_mult.roll, pos_mult.pitch, pos_mult.yaw);
			transform_nav_body_mult.setRotation(world_nav_body_mult_rot);
			br.sendTransform(tf::StampedTransform(transform_nav_body_mult, ros::Time::now(), "navi_frame", "pixhawk_mult"));

			if (verbose) {
				printf("Received multitracker vision estimate message: \n");
				printf("x-pos:% f\n", pos_mult.x);
				printf("y-pos:% f\n", pos_mult.y);
				printf("z-pos:% f\n", pos_mult.z);
				printf("yaw:% f\n", pos_mult.yaw);
				printf("pitch:% f\n", pos_mult.pitch);
				printf("roll:% f\n", pos_mult.roll);
			}

			px_mult_marker_c.header.stamp = ros::Time::now();
			px_mult_marker_f.header.stamp = ros::Time::now();
			px_mult_marker_b.header.stamp = ros::Time::now();
			px_mult_marker_l.header.stamp = ros::Time::now();
			px_mult_marker_r.header.stamp = ros::Time::now();
			px_mult_marker_cam.header.stamp = ros::Time::now();

			px_mult_marker_c.header.frame_id = "/pixhawk_mult";
			px_mult_marker_f.header.frame_id = "/pixhawk_mult";
			px_mult_marker_b.header.frame_id = "/pixhawk_mult";
			px_mult_marker_l.header.frame_id = "/pixhawk_mult";
			px_mult_marker_r.header.frame_id = "/pixhawk_mult";
			px_mult_marker_cam.header.frame_id = "/pixhawk_mult";

			marker_pub.publish(px_mult_marker_c);
			marker_pub.publish(px_mult_marker_f);
			marker_pub.publish(px_mult_marker_b);
			marker_pub.publish(px_mult_marker_l);
			marker_pub.publish(px_mult_marker_r);
			marker_pub.publish(px_mult_marker_cam);
		}
		else {
			mavlink_vision_position_estimate_t pos_est;
			mavlink_msg_vision_position_estimate_decode(msg, &pos_est);
			transform_nav_body_est.setOrigin(tf::Vector3(pos_est.x, pos_est.y, pos_est.z));
			world_nav_body_est_rot = tf::createQuaternionFromRPY(pos_est.roll, pos_est.pitch, pos_est.yaw);
			transform_nav_body_est.setRotation(world_nav_body_est_rot);
			br.sendTransform(tf::StampedTransform(transform_nav_body_est, ros::Time::now(), "navi_frame", "pixhawk_est"));

			if (verbose) {
				printf("Received tracker vision estimate message: \n");
				printf("x-pos:% f\n", pos_est.x);
				printf("y-pos:% f\n", pos_est.y);
				printf("z-pos:% f\n", pos_est.z);
				printf("yaw:% f\n", pos_est.yaw);
				printf("pitch:% f\n", pos_est.pitch);
				printf("roll:% f\n", pos_est.roll);
			}

			px_est_marker_c.header.stamp = ros::Time::now();
			px_est_marker_f.header.stamp = ros::Time::now();
			px_est_marker_b.header.stamp = ros::Time::now();
			px_est_marker_l.header.stamp = ros::Time::now();
			px_est_marker_r.header.stamp = ros::Time::now();
			px_est_marker_cam.header.stamp = ros::Time::now();

			px_est_marker_c.header.frame_id = "/pixhawk_est";
			px_est_marker_f.header.frame_id = "/pixhawk_est";
			px_est_marker_b.header.frame_id = "/pixhawk_est";
			px_est_marker_l.header.frame_id = "/pixhawk_est";
			px_est_marker_r.header.frame_id = "/pixhawk_est";
			px_est_marker_cam.header.frame_id = "/pixhawk_est";

			marker_pub.publish(px_est_marker_c);
			marker_pub.publish(px_est_marker_f);
			marker_pub.publish(px_est_marker_b);
			marker_pub.publish(px_est_marker_l);
			marker_pub.publish(px_est_marker_r);
			marker_pub.publish(px_est_marker_cam);
		}
		break;
	case MAVLINK_MSG_ID_WAYPOINT:
		mavlink_waypoint_t wayp;
		mavlink_msg_waypoint_decode(msg, &wayp);
		waypoints.insert(waypoints.begin()+wayp.seq, wayp);
		if (verbose) {
			printf("Received waypoint message: \n");
			printf("x-pos:% f\n", wayp.x);
			printf("y-pos:% f\n", wayp.y);
			printf("z-pos:% f\n", wayp.z);
			printf("yaw:% f\n", wayp.yaw);
			printf("autocontinue:% i\n", wayp.autocontinue);
			printf("curent:% i\n", wayp.current);
		}
		wp_marker_c.header.stamp = ros::Time::now();
		wp_marker_dir.header.stamp = ros::Time::now();

		wp_marker_c.header.frame_id = "/navi_frame";
		wp_marker_dir.header.frame_id = "/navi_frame";

		wp_marker_c.id = wayp.seq*2+1+50;
		wp_marker_c.pose.position.x = wayp.x;
		wp_marker_c.pose.position.y = wayp.y;
		wp_marker_c.pose.position.z = wayp.z;
		wp_marker_dir.id = wayp.seq*2+2+50;
		wp_marker_dir.pose.position.x = wayp.x;
		wp_marker_dir.pose.position.y = wayp.y;
		wp_marker_dir.pose.position.z = wayp.z;
		wp_marker_dir.pose.orientation.x = 0;
		wp_marker_dir.pose.orientation.y = 0;
		wp_marker_dir.pose.orientation.z = wayp.yaw;

		if (wayp.autocontinue){
			wp_marker_c.color.r = 1.0f;
			wp_marker_c.color.g = 1.0f;
			wp_marker_c.color.b = 0.0f;
			wp_marker_dir.color.r = 1.0f;
			wp_marker_dir.color.g = 1.0f;
			wp_marker_dir.color.b = 0.0f;
		}
		else{
			wp_marker_c.color.r = 1.0f;
			wp_marker_c.color.g = 0.0f;
			wp_marker_c.color.b = 0.0f;
			wp_marker_dir.color.r = 1.0f;
			wp_marker_dir.color.g = 0.0f;
			wp_marker_dir.color.b = 0.0f;
		}

		if (wayp.current){
			wp_marker_c.color.r = 0.0f;
			wp_marker_c.color.g = 1.0f;
			wp_marker_c.color.b = 0.0f;
			wp_marker_dir.color.r = 0.0f;
			wp_marker_dir.color.g = 1.0f;
			wp_marker_dir.color.b = 0.0f;
		}

		marker_pub.publish(wp_marker_c);
		marker_pub.publish(wp_marker_dir);
		usleep(100);
		marker_pub.publish(wp_marker_c);
		marker_pub.publish(wp_marker_dir);
		break;
	/*case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
	{
		mavlink_waypoint_set_current_t sa;
		mavlink_msg_waypoint_set_current_decode(msg, &sa);
		if (verbose) {
			printf("Received waypoint set curent message: \n");
			printf("id: %i\n", sa.seq);
		}
		mavlink_waypoint_t wp = waypoints[sa.seq];
		wp_marker_c.header.stamp = ros::Time::now();
		wp_marker_dir.header.stamp = ros::Time::now();

		wp_marker_c.header.frame_id = "/navi_frame";
		wp_marker_dir.header.frame_id = "/navi_frame";

		wp_marker_c.pose.position.x = wp.x;
		wp_marker_c.pose.position.y = wp.y;
		wp_marker_c.pose.position.z = wp.z;
		wp_marker_dir.pose.position.x = wp.x;
		wp_marker_dir.pose.position.y = wp.y;
		wp_marker_dir.pose.position.z = wp.z;
		wp_marker_dir.pose.orientation.x = 0;
		wp_marker_dir.pose.orientation.y = 0;
		wp_marker_dir.pose.orientation.z = wp.yaw;

		wp_marker_c.id = sa.seq*2+1+50;
		wp_marker_dir.id = sa.seq*2+2+50;
		wp_marker_c.color.r = 0.0f;
		wp_marker_c.color.g = 1.0f;
		wp_marker_c.color.b = 0.0f;
		wp_marker_dir.color.r = 0.0f;
		wp_marker_dir.color.g = 1.0f;
		wp_marker_dir.color.b = 0.0f;

		marker_pub.publish(wp_marker_c);
		marker_pub.publish(wp_marker_c);
		usleep(100);
		marker_pub.publish(wp_marker_dir);
		marker_pub.publish(wp_marker_dir);
		break;
	}*/
	case MAVLINK_MSG_ID_WAYPOINT_REACHED:
		{
			mavlink_waypoint_reached_t rwp;
			mavlink_msg_waypoint_reached_decode(msg, &rwp);
			if (verbose) {
				printf("Received waypoint reached message: \n");
				printf("id: %i\n", rwp.seq);
			}
			mavlink_waypoint_t wp = waypoints[rwp.seq];
			//wp_marker_c.header.stamp = ros::Time::now();
			//wp_marker_dir.header.stamp = ros::Time::now();

			wp_marker_c.header.frame_id = "/navi_frame";
			wp_marker_dir.header.frame_id = "/navi_frame";

			wp_marker_c.pose.position.x = wp.x;
			wp_marker_c.pose.position.y = wp.y;
			wp_marker_c.pose.position.z = wp.z;
			wp_marker_dir.pose.position.x = wp.x;
			wp_marker_dir.pose.position.y = wp.y;
			wp_marker_dir.pose.position.z = wp.z;
			wp_marker_dir.pose.orientation.x = 0;
			wp_marker_dir.pose.orientation.y = 0;
			wp_marker_dir.pose.orientation.z = wp.yaw;

			wp_marker_c.id = rwp.seq*2+1+50;
			wp_marker_dir.id = rwp.seq*2+2+50;
			wp_marker_c.color.r = 1.0f;
			wp_marker_c.color.g = 1.0f;
			wp_marker_c.color.b = 1.0f;
			wp_marker_dir.color.r = 1.0f;
			wp_marker_dir.color.g = 1.0f;
			wp_marker_dir.color.b = 1.0f;

			marker_pub.publish(wp_marker_c);
			marker_pub.publish(wp_marker_dir);
			usleep(1000);
			marker_pub.publish(wp_marker_c);
			marker_pub.publish(wp_marker_dir);
			break;
		}
	/*case MAVLINK_MSG_ID_MARKER:
		mavlink_marker_t art_marker;
		mavlink_msg_marker_decode(msg, &art_marker);
		if (verbose) {
			printf("Received marker message: \n");
			printf("id: %i\n", art_marker.id);
			printf("x-pos:% f\n", art_marker.x);
			printf("y-pos:% f\n", art_marker.y);
			printf("z-pos:% f\n", art_marker.z);
			printf("roll:% f\n", art_marker.roll);
			printf("pitch:% f\n", art_marker.pitch);
			printf("yaw:% f\n", art_marker.yaw);

		}
		marker_marker.header.stamp = ros::Time::now();
		marker_marker.header.frame_id = "/navi_frame";
		marker_marker.id = art_marker.id+100;
		marker_marker.pose.position.x = art_marker.x;
		marker_marker.pose.position.y = art_marker.y;
		marker_marker.pose.position.z = art_marker.z;
		marker_marker.pose.orientation.x = art_marker.roll;
		marker_marker.pose.orientation.y = art_marker.pitch;
		marker_marker.pose.orientation.z = art_marker.yaw;

		marker_pub.publish(marker_marker);
		break;*/
	case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT:
		mavlink_local_position_setpoint_t setpoint;
		mavlink_msg_local_position_setpoint_decode(msg, &setpoint);
		if (verbose) {
			printf("Received setpoint message: \n");
			printf("x-pos:% f\n", setpoint.x);
			printf("y-pos:% f\n", setpoint.y);
			printf("z-pos:% f\n", setpoint.z);
			printf("yaw:% f\n", setpoint.yaw);
		}
		setpoint_marker_c.header.stamp = ros::Time::now();
		setpoint_marker_c.header.frame_id = "/navi_frame";
		setpoint_marker_c.id = 2+200;
		setpoint_marker_c.pose.position.x = setpoint.x;
		setpoint_marker_c.pose.position.y = setpoint.y;
		setpoint_marker_c.pose.position.z = setpoint.z;
		setpoint_marker_c.pose.orientation.z = setpoint.yaw;
		setpoint_marker_c.header.stamp = ros::Time::now();
		setpoint_marker_dir.header.frame_id = "/navi_frame";
		setpoint_marker_dir.id = 2+200+1;
		setpoint_marker_dir.pose.position.x = setpoint.x;
		setpoint_marker_dir.pose.position.y = setpoint.y;
		setpoint_marker_dir.pose.position.z = setpoint.z;
		setpoint_marker_dir.pose.orientation.z = setpoint.yaw;

		marker_pub.publish(setpoint_marker_c);
		marker_pub.publish(setpoint_marker_dir);
		usleep(100);
		marker_pub.publish(setpoint_marker_c);
		marker_pub.publish(setpoint_marker_dir);
		break;
	case MAVLINK_MSG_ID_HEARTBEAT:
		if (verbose) {
			printf("Received heartbeat");
		}
		break;
	default:
		if (verbose) {
			printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		}
		break;
	}
}



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


	printf("bla");

	printf("setpoint message id %i",MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT);
	printf("vision message id %i",MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE);

	// print legend
	if (help)
	{
		printf("Legend of Waypoints:\n");
		printf("green: active waypoint\n");
		printf("yellow: waypoints, autocontinue true\n");
		printf("red: waypoints, autocontinue false\n");
		printf("blue: control setpoints\n");
	}


	// ROS node initialization
	ros::init(argc, argv, "px_rviz");		// initialize ROS
	ros::NodeHandle n;							// create a handle to this process node
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_markers", 0);



	// Setting markers---------------------------------------------------------------------------------------------
	uint32_t cyl = visualization_msgs::Marker::CYLINDER;
	uint32_t cube = visualization_msgs::Marker::CUBE;
	uint32_t arrow = visualization_msgs::Marker::ARROW;
	uint32_t sphere = visualization_msgs::Marker::SPHERE;

	// properties of the default marker
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "px";
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
	marker.scale.x = base_width;
	marker.scale.y = base_width;
	marker.scale.z = base_height;
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	// define other markers
	px_marker_c = marker;
	px_marker_f = marker;
	px_marker_b = marker;
	px_marker_l = marker;
	px_marker_r = marker;
	px_marker_cam = marker;

	wp_marker_c = marker;
	wp_marker_dir = marker;

	marker_marker = marker;


	px_marker_c.id = 1;
	px_marker_f.id = 2;
	px_marker_b.id = 3;
	px_marker_l.id = 4;
	px_marker_r.id = 5;
	px_marker_cam.id = 6;

	px_marker_f.type = cyl;
	px_marker_b.type = cyl;
	px_marker_l.type = cyl;
	px_marker_r.type = cyl;
	wp_marker_c.type = sphere;
	wp_marker_dir.type = arrow;
	marker_marker.type = cube;


	px_marker_f.color.r = 1.0f;
	px_marker_f.color.g = 0.0f;
	px_marker_f.color.b = 0.0f;

	marker_marker.color.r = 1.0f;
	marker_marker.color.g = 1.0f;
	marker_marker.color.b = 0.0f;


	px_marker_f.scale.x = prop_diam;
	px_marker_f.scale.y = prop_diam;
	px_marker_f.scale.z = prop_height;
	px_marker_b.scale.x = prop_diam;
	px_marker_b.scale.y = prop_diam;
	px_marker_b.scale.z = prop_height;
	px_marker_l.scale.x = prop_diam;
	px_marker_l.scale.y = prop_diam;
	px_marker_l.scale.z = prop_height;
	px_marker_r.scale.x = prop_diam;
	px_marker_r.scale.y = prop_diam;
	px_marker_r.scale.z = prop_height;
	px_marker_cam.scale.x = cam_length;
	px_marker_cam.scale.y = cam_width;
	px_marker_cam.scale.z = cam_height;

	wp_marker_c.scale.x = prop_diam;
	wp_marker_c.scale.y = wp_marker_c.scale.x;
	wp_marker_c.scale.z = wp_marker_c.scale.x;
	wp_marker_dir.scale.x = prop_diam;
	wp_marker_dir.scale.y = 5*wp_marker_dir.scale.x;
	wp_marker_dir.scale.z = 5*wp_marker_dir.scale.x;

	marker_marker.scale.x = markersize;
	marker_marker.scale.y = markersize;
	marker_marker.scale.z = markersize/10;


	setpoint_marker_c = wp_marker_c;
	setpoint_marker_dir = wp_marker_dir;
	setpoint_marker_c.color.r = 0.0f;
	setpoint_marker_c.color.g = 0.0f;
	setpoint_marker_c.color.b = 1.0f;
	setpoint_marker_dir.color.r = 0.0f;
	setpoint_marker_dir.color.g = 0.0f;
	setpoint_marker_dir.color.b = 1.0f;


	px_marker_f.pose.position.x = marker.pose.position.x + prop_radius + base_width/2;
	px_marker_f.pose.position.z = marker.pose.position.z + base_height/4;
	px_marker_b.pose.position.x = marker.pose.position.x - prop_radius - base_width/2;
	px_marker_b.pose.position.z = marker.pose.position.z + base_height/4;
	px_marker_l.pose.position.y = marker.pose.position.y + prop_radius + base_width/2;
	px_marker_l.pose.position.z = marker.pose.position.z + base_height/4;
	px_marker_r.pose.position.y = marker.pose.position.y - prop_radius - base_width/2;
	px_marker_r.pose.position.z = marker.pose.position.z + base_height/4;
	px_marker_cam.pose.position.z = marker.pose.position.z + base_height/2 + cam_height/2;
	px_marker_cam.pose.position.x = marker.pose.position.x + base_width/2;


	px_est_marker_c = px_marker_c;
	px_est_marker_f = px_marker_f;
	px_est_marker_b = px_marker_b;
	px_est_marker_l = px_marker_l;
	px_est_marker_r = px_marker_r;
	px_est_marker_cam = px_marker_cam;

	px_mult_marker_c = px_marker_c;
	px_mult_marker_f = px_marker_f;
	px_mult_marker_b = px_marker_b;
	px_mult_marker_l = px_marker_l;
	px_mult_marker_r = px_marker_r;
	px_mult_marker_cam = px_marker_cam;

	px_est_marker_c.id = 11;
	px_est_marker_f.id = 12;
	px_est_marker_b.id = 13;
	px_est_marker_l.id = 14;
	px_est_marker_r.id = 15;
	px_est_marker_cam.id = 16;

	px_mult_marker_c.id = 17;
	px_mult_marker_f.id = 18;
	px_mult_marker_b.id = 19;
	px_mult_marker_l.id = 20;
	px_mult_marker_r.id = 21;
	px_mult_marker_cam.id = 22;

	px_mult_marker_c.color.r = 0.0f;
	px_mult_marker_c.color.g = 1.0f;
	px_mult_marker_c.color.b = 1.0f;
	px_mult_marker_b.color.r = 0.0f;
	px_mult_marker_b.color.g = 1.0f;
	px_mult_marker_b.color.b = 1.0f;
	px_mult_marker_l.color.r = 0.0f;
	px_mult_marker_l.color.g = 1.0f;
	px_mult_marker_l.color.b = 1.0f;
	px_mult_marker_r.color.r = 0.0f;
	px_mult_marker_r.color.g = 1.0f;
	px_mult_marker_r.color.b = 1.0f;
	px_mult_marker_cam.color.r = 0.0f;
	px_mult_marker_cam.color.g = 1.0f;
	px_mult_marker_cam.color.b = 1.0f;

	px_est_marker_c.color.r = 1.0f;
	px_est_marker_c.color.g = 1.0f;
	px_est_marker_c.color.b = 0.0f;
	px_est_marker_b.color.r = 1.0f;
	px_est_marker_b.color.g = 1.0f;
	px_est_marker_b.color.b = 0.0f;
	px_est_marker_l.color.r = 1.0f;
	px_est_marker_l.color.g = 1.0f;
	px_est_marker_l.color.b = 0.0f;
	px_est_marker_r.color.r = 1.0f;
	px_est_marker_r.color.g = 1.0f;
	px_est_marker_r.color.b = 0.0f;
	px_est_marker_cam.color.r = 1.0f;
	px_est_marker_cam.color.g = 1.0f;
	px_est_marker_cam.color.b = 0.0f;
	// ------------------------------------------------------------------------------------------------------------

	lcm_t * lcm;
	lcm = lcm_create ("udpm://");
	if (!lcm) return 1;

	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
		{
			g_thread_init(NULL);
			// Only initialize g thread if not already done
		}

	mavlink_message_t_subscription_t * commSub = mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, NULL);

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Failed to create LCM handling thread: %s!!\n", err->message );
		g_error_free(err);
		exit(EXIT_FAILURE);
	}





	while(ros::ok()){
		usleep(10000);
	}

	mavlink_message_t_unsubscribe (lcm, commSub);
	lcm_destroy (lcm);
	n.shutdown();
	ros::shutdown();
	return 0;
}

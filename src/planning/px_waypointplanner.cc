/*======================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  Benjamin Knecht (bknecht@student.ethz.ch)
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

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

========================================================================*/

/**
*   @file
*   @brief a program to manage waypoints and exchange them with the ground station
*
*   @author Benjamin Knecht <bknecht@student.ethz.ch>
*   @author Christian Schluchter <schluchc@ee.ethz.ch>
*/

#include <boost/program_options.hpp>
#include <iostream>
#include <vector>

#include <PxVector3.h>

#include "mavconn.h"
#include <glib.h>

namespace config = boost::program_options;

lcm_t* lcm;

bool debug;             	///< boolean for debug output or behavior
bool verbose;           	///< boolean for verbose output
std::string configFile;		///< Configuration file for parameters

//==== variables for the planner ====
bool idle = false;      				///< indicates if the system is following the waypoints or is waiting
uint16_t current_active_wp_id = -1;		///< id of current waypoint
bool yawReached;						///< boolean for yaw attitude reached
bool posReached;						///< boolean for position reached
uint64_t timestamp_lastoutside_orbit = 0;///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
uint64_t timestamp_firstinside_orbit = 0;///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value

std::vector<mavlink_waypoint_t*> waypoints1;	///< vector1 that holds the waypoints
std::vector<mavlink_waypoint_t*> waypoints2;	///< vector2 that holds the waypoints

std::vector<mavlink_waypoint_t*>* waypoints = &waypoints1;					///< pointer to the currently active waypoint vector
std::vector<mavlink_waypoint_t*>* waypoints_receive_buffer = &waypoints2;	///< pointer to the receive buffer waypoint vector

//==== variables needed for communication protocol ====
uint8_t systemid = getSystemID();          		///< indicates the ID of the system
uint8_t compid = MAV_COMP_ID_WAYPOINTPLANNER;	///< indicates the component ID of the waypointplanner

PxParamClient* paramClient;

enum PX_WAYPOINTPLANNER_STATES
{
	PX_WPP_IDLE = 0,
	PX_WPP_SENDLIST,
	PX_WPP_SENDLIST_SENDWPS,
	PX_WPP_GETLIST,
	PX_WPP_GETLIST_GETWPS,
	PX_WPP_GETLIST_GOTALL
};

PX_WAYPOINTPLANNER_STATES current_state = PX_WPP_IDLE;
uint16_t protocol_current_wp_id = 0;
uint16_t protocol_current_count = 0;
uint8_t protocol_current_partner_systemid = 0;
uint8_t protocol_current_partner_compid = 0;
uint64_t protocol_timestamp_lastaction = 0;

uint64_t timestamp_last_send_setpoint = 0;


/*
*  @brief Sends an waypoint ack message
*/
void send_waypoint_ack(uint8_t target_systemid, uint8_t target_compid, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_waypoint_ack_t wpa;

	wpa.target_system = target_systemid;
	wpa.target_component = target_compid;
	wpa.type = type;

	mavlink_msg_waypoint_ack_encode(systemid, compid, &msg, &wpa);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	if (verbose) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
}

/*
*  @brief Broadcasts the new target waypoint and directs the MAV to fly there
*
*  This function broadcasts its new active waypoint sequence number and
*  sends a message to the controller, advising it to fly to the coordinates
*  of the waypoint with a given orientation
*
*  @param seq The waypoint sequence number the MAV should fly to.
*/
void send_waypoint_current(uint16_t seq)
{
	if(seq < waypoints->size())
	{
		mavlink_waypoint_t *cur = waypoints->at(seq);

		mavlink_message_t msg;
		mavlink_waypoint_current_t wpc;

		wpc.seq = cur->seq;

		mavlink_msg_waypoint_current_encode(systemid, compid, &msg, &wpc);
		mavlink_message_t_publish(lcm, "MAVLINK", &msg);

		usleep(paramClient->getParamValue("PROTOCOLDELAY"));

		if (verbose) printf("Broadcasted new current waypoint %u\n", wpc.seq);
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
}

/*
*  @brief Directs the MAV to fly to a position
*
*  Sends a message to the controller, advising it to fly to the coordinates
*  of the waypoint with a given orientation
*
*  @param seq The waypoint sequence number the MAV should fly to.
*/
void send_setpoint(uint16_t seq)
{
	if(seq < waypoints->size())
	{
		mavlink_waypoint_t *cur = waypoints->at(seq);

		mavlink_message_t msg;
		mavlink_local_position_setpoint_set_t PControlSetPoint;

		// send new set point to local IMU
		if (cur->frame == 1)
		{
			PControlSetPoint.target_system = systemid;
			PControlSetPoint.target_component = MAV_COMP_ID_IMU;
			PControlSetPoint.x = cur->x;
			PControlSetPoint.y = cur->y;
			PControlSetPoint.z = cur->z;
			PControlSetPoint.yaw = cur->yaw;

			mavlink_msg_local_position_setpoint_set_encode(systemid, compid, &msg, &PControlSetPoint);
			mavlink_message_t_publish(lcm, "MAVLINK", &msg);

			usleep(paramClient->getParamValue("PROTOCOLDELAY"));
		}
		else
		{
			if (verbose) printf("No new set point sent to IMU because the new waypoint %u had no local coordinates\n", cur->seq);
		}

		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		timestamp_last_send_setpoint = now;
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
}

void send_waypoint_count(uint8_t target_systemid, uint8_t target_compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_waypoint_count_t wpc;

	wpc.target_system = target_systemid;
	wpc.target_component = target_compid;
	wpc.count = count;

	mavlink_msg_waypoint_count_encode(systemid, compid, &msg, &wpc);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	if (verbose) printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);

	usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

void send_waypoint(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
	if (seq < waypoints->size())
	{
		mavlink_message_t msg;
		mavlink_waypoint_t *wp = waypoints->at(seq);
		wp->target_system = target_systemid;
		wp->target_component = target_compid;
		mavlink_msg_waypoint_encode(systemid, compid, &msg, wp);
		mavlink_message_t_publish(lcm, "MAVLINK", &msg);
		if (verbose) printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);

		usleep(paramClient->getParamValue("PROTOCOLDELAY"));
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
}

void send_waypoint_request(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
	if (seq < waypoints->size())
	{
		mavlink_message_t msg;
		mavlink_waypoint_request_t wpr;
		wpr.target_system = target_systemid;
		wpr.target_component = target_compid;
		wpr.seq = seq;
		mavlink_msg_waypoint_request_encode(systemid, compid, &msg, &wpr);
		mavlink_message_t_publish(lcm, "MAVLINK", &msg);
		if (verbose) printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);

		usleep(paramClient->getParamValue("PROTOCOLDELAY"));
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
}

/*
*  @brief emits a message that a waypoint reached
*
*  This function broadcasts a message that a waypoint is reached.
*
*  @param seq The waypoint sequence number the MAV has reached.
*/
void send_waypoint_reached(uint16_t seq)
{
	mavlink_message_t msg;
	mavlink_waypoint_reached_t wp_reached;

	wp_reached.seq = seq;

	mavlink_msg_waypoint_reached_encode(systemid, compid, &msg, &wp_reached);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	if (verbose) printf("Sent waypoint %u reached message\n", wp_reached.seq);

	usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

float distanceToSegment(uint16_t seq, float x, float y, float z)
{
	if (seq < waypoints->size())
	{
		mavlink_waypoint_t *cur = waypoints->at(seq);

		const PxVector3 A(cur->x, cur->y, cur->z);
		const PxVector3 C(x, y, z);

		// seq not the second last waypoint
		if ((uint16_t)(seq+1) < waypoints->size())
		{
			mavlink_waypoint_t *next = waypoints->at(seq+1);
			const PxVector3 B(next->x, next->y, next->z);
			const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
			if (r >= 0 && r <= 1)
			{
				const PxVector3 P(A + r*(B-A));
				return (P-C).length();
			}
			else if (r < 0.f)
			{
				return (C-A).length();
			}
			else
			{
				return (C-B).length();
			}
		}
		else
		{
			return (C-A).length();
		}
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
	return -1.f;
}

float distanceToPoint(uint16_t seq, float x, float y, float z)
{
	if (seq < waypoints->size())
	{
		mavlink_waypoint_t *cur = waypoints->at(seq);

		const PxVector3 A(cur->x, cur->y, cur->z);
		const PxVector3 C(x, y, z);

		return (C-A).length();
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds\n");
	}
	return -1.f;
}

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
	// Handle param messages
	paramClient->handleMAVLinkPacket(msg);

	//check for timed-out operations
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	if (now-protocol_timestamp_lastaction > paramClient->getParamValue("PROTOCOLTIMEOUT") && current_state != PX_WPP_IDLE)
	{
		if (verbose) printf("Last operation (state=%u) timed out, changing state to PX_WPP_IDLE\n", current_state);
		current_state = PX_WPP_IDLE;
		protocol_current_count = 0;
		protocol_current_partner_systemid = 0;
		protocol_current_partner_compid = 0;
		protocol_current_wp_id = -1;

		if(waypoints->size() == 0)
		{
			current_active_wp_id = -1;
		}
	}

	if(now-timestamp_last_send_setpoint > paramClient->getParamValue("SETPOINTDELAY") && current_active_wp_id < waypoints->size())
	{
		send_setpoint(current_active_wp_id);
	}

	switch(msg->msgid)
	{
		case MAVLINK_MSG_ID_ATTITUDE:
		{
			if(msg->sysid == systemid && current_active_wp_id < waypoints->size())
			{
				mavlink_waypoint_t *wp = waypoints->at(current_active_wp_id);
				if(wp->frame == 1)
				{
					mavlink_attitude_t att;
					mavlink_msg_attitude_decode(msg, &att);
					float yaw_tolerance = paramClient->getParamValue("YAWTOLERANCE");
					//compare current yaw
					if (att.yaw - yaw_tolerance >= 0.0f && att.yaw + yaw_tolerance < 2.f*M_PI)
					{
						if (att.yaw - yaw_tolerance <= wp->yaw && att.yaw + yaw_tolerance >= wp->yaw)
							yawReached = true;
					}
					else if(att.yaw - yaw_tolerance < 0.0f)
					{
						float lowerBound = 360.0f + att.yaw - yaw_tolerance;
						if (lowerBound < wp->yaw || wp->yaw < att.yaw + yaw_tolerance)
							yawReached = true;
					}
					else
					{
						float upperBound = att.yaw + yaw_tolerance - 2.f*M_PI;
						if (att.yaw - yaw_tolerance < wp->yaw || wp->yaw < upperBound)
							yawReached = true;
					}
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_LOCAL_POSITION:
		{
			if(msg->sysid == systemid && current_active_wp_id < waypoints->size())
			{
				mavlink_waypoint_t *wp = waypoints->at(current_active_wp_id);

				if(wp->frame == 1)
				{
					mavlink_local_position_t pos;
					mavlink_msg_local_position_decode(msg, &pos);
					if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);

					posReached = false;

					// compare current position (given in message) with current waypoint
					float orbit = wp->param1;

					float dist;
					if (wp->param2 == 0)
					{
						dist = distanceToSegment(current_active_wp_id, pos.x, pos.y, pos.z);
					}
					else
					{
						dist = distanceToPoint(current_active_wp_id, pos.x, pos.y, pos.z);
					}

					if (dist >= 0.f && dist <= orbit && yawReached)
					{
						posReached = true;
					}
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_ACTION: // special action from ground station
		{
			mavlink_action_t action;
			mavlink_msg_action_decode(msg, &action);
			if(action.target == systemid)
			{
				if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
				switch (action.action)
				{
//				case MAV_ACTION_LAUNCH:
//					if (verbose) std::cerr << "Launch received" << std::endl;
//					current_active_wp_id = 0;
//					if (waypoints->size()>0)
//					{
//						setActive(waypoints[current_active_wp_id]);
//					}
//					else
//						if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
//					break;

//				case MAV_ACTION_CONTINUE:
//					if (verbose) std::c
//					err << "Continue received" << std::endl;
//					idle = false;
//					setActive(waypoints[current_active_wp_id]);
//					break;

//				case MAV_ACTION_HALT:
//					if (verbose) std::cerr << "Halt received" << std::endl;
//					idle = true;
//					break;

//				default:
//					if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
//					break;
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_ACK:
		{
			mavlink_waypoint_ack_t wpa;
			mavlink_msg_waypoint_ack_decode(msg, &wpa);

			if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wpa.target_system == systemid && wpa.target_component == compid))
			{
				protocol_timestamp_lastaction = now;

				if (current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)
				{
					if (protocol_current_wp_id == waypoints->size()-1)
					{
						if (verbose) printf("Received Ack after having sent last waypoint, going to state PX_WPP_IDLE\n");
						current_state = PX_WPP_IDLE;
						protocol_current_wp_id = 0;
					}
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
		{
			mavlink_waypoint_set_current_t wpc;
			mavlink_msg_waypoint_set_current_decode(msg, &wpc);

			if(wpc.target_system == systemid && wpc.target_component == compid)
			{
				protocol_timestamp_lastaction = now;

				if (current_state == PX_WPP_IDLE)
				{
					if (wpc.seq < waypoints->size())
					{
						if (verbose) printf("Received MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT\n");
						current_active_wp_id = wpc.seq;
						uint32_t i;
						for(i = 0; i < waypoints->size(); i++)
						{
							if (i == current_active_wp_id)
							{
								waypoints->at(i)->current = true;
							}
							else
							{
								waypoints->at(i)->current = false;
							}
						}
						if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
						yawReached = false;
						posReached = false;
						send_waypoint_current(current_active_wp_id);
						send_setpoint(current_active_wp_id);
						timestamp_firstinside_orbit = 0;
					}
					else
					{
						if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: Index out of bounds\n");
					}
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
		{
			mavlink_waypoint_request_list_t wprl;
			mavlink_msg_waypoint_request_list_decode(msg, &wprl);
			if(wprl.target_system == systemid && wprl.target_component == compid)
			{
				protocol_timestamp_lastaction = now;

				if (current_state == PX_WPP_IDLE || current_state == PX_WPP_SENDLIST)
				{
					if (waypoints->size() > 0)
					{
						if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u changing state to PX_WPP_SENDLIST\n", msg->sysid);
						if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST again from %u staying in state PX_WPP_SENDLIST\n", msg->sysid);
						current_state = PX_WPP_SENDLIST;
						protocol_current_wp_id = 0;
						protocol_current_partner_systemid = msg->sysid;
						protocol_current_partner_compid = msg->compid;
					}
					else
					{
						if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
					}
					protocol_current_count = waypoints->size();
					send_waypoint_count(msg->sysid,msg->compid, protocol_current_count);
				}
				else
				{
					if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST because i'm doing something else already (state=%i).\n", current_state);
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
		{
			mavlink_waypoint_request_t wpr;
			mavlink_msg_waypoint_request_decode(msg, &wpr);
			if(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid && wpr.target_system == systemid && wpr.target_component == compid)
			{
				protocol_timestamp_lastaction = now;

				//ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				if ((current_state == PX_WPP_SENDLIST && wpr.seq == 0) || (current_state == PX_WPP_SENDLIST_SENDWPS && (wpr.seq == protocol_current_wp_id || wpr.seq == protocol_current_wp_id + 1) && wpr.seq < waypoints->size()))
				{
					if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u changing state to PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
					if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id + 1) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
					if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u (again) from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

					current_state = PX_WPP_SENDLIST_SENDWPS;
					protocol_current_wp_id = wpr.seq;
					send_waypoint(protocol_current_partner_systemid, protocol_current_partner_compid, wpr.seq);
				}
				else
				{
					if (verbose)
					{
						if (!(current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because i'm doing something else already (state=%i).\n", current_state); break; }
						else if (current_state == PX_WPP_SENDLIST)
						{
							if (wpr.seq != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
						}
						else if (current_state == PX_WPP_SENDLIST_SENDWPS)
						{
							if (wpr.seq != protocol_current_wp_id && wpr.seq != protocol_current_wp_id + 1) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, protocol_current_wp_id, protocol_current_wp_id+1);
							else if (wpr.seq >= waypoints->size()) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
						}
						else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST - FIXME: missed error description\n");
					}
				}
			}
			else
			{
				//we we're target but already communicating with someone else
				if((wpr.target_system == systemid && wpr.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid))
				{
					if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, protocol_current_partner_systemid);
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_COUNT:
		{
			mavlink_waypoint_count_t wpc;
			mavlink_msg_waypoint_count_decode(msg, &wpc);
			if(wpc.target_system == systemid && wpc.target_component == compid)
			{
				protocol_timestamp_lastaction = now;

				if (current_state == PX_WPP_IDLE || (current_state == PX_WPP_GETLIST && protocol_current_wp_id == 0))
				{
					if (wpc.count > 0)
					{
						if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) from %u changing state to PX_WPP_GETLIST\n", wpc.count, msg->sysid);
						if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) again from %u\n", wpc.count, msg->sysid);

						current_state = PX_WPP_GETLIST;
						protocol_current_wp_id = 0;
						protocol_current_partner_systemid = msg->sysid;
						protocol_current_partner_compid = msg->compid;
						protocol_current_count = wpc.count;

						printf("clearing receive buffer and readying for receiving waypoints\n");
						while(waypoints_receive_buffer->size() > 0)
						{
							delete waypoints_receive_buffer->back();
							waypoints_receive_buffer->pop_back();
						}

						send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
					}
					else
					{
						if (verbose) printf("Ignoring MAVLINK_MSG_ID_WAYPOINT_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
					}
				}
				else
				{
					if (verbose && !(current_state == PX_WPP_IDLE || current_state == PX_WPP_GETLIST)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm doing something else already (state=%i).\n", current_state);
					else if (verbose && current_state == PX_WPP_GETLIST && protocol_current_wp_id != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm already receiving waypoint %u.\n", protocol_current_wp_id);
					else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT - FIXME: missed error description\n");
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT:
		{
			mavlink_waypoint_t wp;
			mavlink_msg_waypoint_decode(msg, &wp);

			if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wp.target_system == systemid && wp.target_component == compid))
			{
				protocol_timestamp_lastaction = now;

				//ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
				if ((current_state == PX_WPP_GETLIST && wp.seq == 0) || (current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id && wp.seq < protocol_current_count))
				{
					if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u changing state to PX_WPP_GETLIST_GETWPS\n", wp.seq, msg->sysid);
					if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u\n", wp.seq, msg->sysid);
					if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq-1 == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u (again) from %u\n", wp.seq, msg->sysid);

					current_state = PX_WPP_GETLIST_GETWPS;
					protocol_current_wp_id = wp.seq + 1;
					mavlink_waypoint_t* newwp = new mavlink_waypoint_t;
					memcpy(newwp, &wp, sizeof(mavlink_waypoint_t));
					waypoints_receive_buffer->push_back(newwp);

					if(protocol_current_wp_id == protocol_current_count && current_state == PX_WPP_GETLIST_GETWPS)
					{
						if (verbose) printf("Got all %u waypoints, changing state to PX_WPP_IDLE\n", protocol_current_count);

						send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);

						if (current_active_wp_id > waypoints_receive_buffer->size()-1)
						{
							current_active_wp_id = waypoints_receive_buffer->size() - 1;
						}

						// switch the waypoints list
						std::vector<mavlink_waypoint_t*>* waypoints_temp = waypoints;
						waypoints = waypoints_receive_buffer;
						waypoints_receive_buffer = waypoints_temp;

						//get the new current waypoint
						uint32_t i;
						for(i = 0; i < waypoints->size(); i++)
						{
							if (waypoints->at(i)->current == 1)
							{
								current_active_wp_id = i;
								//if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
								yawReached = false;
								posReached = false;
								send_waypoint_current(current_active_wp_id);
								send_setpoint(current_active_wp_id);
								timestamp_firstinside_orbit = 0;
								break;
							}
						}

						if (i == waypoints->size())
						{
							current_active_wp_id = -1;
							yawReached = false;
							posReached = false;
							timestamp_firstinside_orbit = 0;
						}

						current_state = PX_WPP_IDLE;
					}
					else
					{
						send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
					}
				}
				else
				{
					if (current_state == PX_WPP_IDLE)
					{
						//we're done receiving waypoints, answer with ack.
						send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);
						printf("Received MAVLINK_MSG_ID_WAYPOINT while state=PX_WPP_IDLE, answered with WAYPOINT_ACK.\n");
					}
					if (verbose)
					{
						if (!(current_state == PX_WPP_GETLIST || current_state == PX_WPP_GETLIST_GETWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u because i'm doing something else already (state=%i).\n", wp.seq, current_state); break; }
						else if (current_state == PX_WPP_GETLIST)
						{
							if(!(wp.seq == 0)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the first waypoint ID (%u) was not 0.\n", wp.seq);
							else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
						}
						else if (current_state == PX_WPP_GETLIST_GETWPS)
						{
							if (!(wp.seq == protocol_current_wp_id)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was not the expected %u.\n", wp.seq, protocol_current_wp_id);
							else if (!(wp.seq < protocol_current_count)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was out of bounds.\n", wp.seq);
							else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
						}
						else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
					}
				}
			}
			else
			{
				//we we're target but already communicating with someone else
				if((wp.target_system == systemid && wp.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && current_state != PX_WPP_IDLE)
				{
					if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, protocol_current_partner_systemid);
				}
				else if(wp.target_system == systemid && wp.target_component == compid)
				{
					if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
		{
			mavlink_waypoint_clear_all_t wpca;
			mavlink_msg_waypoint_clear_all_decode(msg, &wpca);

			if(wpca.target_system == systemid && wpca.target_component == compid && current_state == PX_WPP_IDLE)
			{
				protocol_timestamp_lastaction = now;

				if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
				while(waypoints->size() > 0)
				{
					delete waypoints->back();
					waypoints->pop_back();
				}
				current_active_wp_id = -1;
			}
			else if (wpca.target_system == systemid && wpca.target_component == compid && current_state != PX_WPP_IDLE)
			{
				if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, current_state);
			}
			break;
		}

		default:
		{
			if (debug) std::cerr << "Waypoint: received message of unknown type" << std::endl;
			break;
		}
	}

	//check if the current waypoint was reached
	if ((posReached && /*yawReached &&*/ !idle))
	{
		if (current_active_wp_id < waypoints->size())
		{
			mavlink_waypoint_t *cur_wp = waypoints->at(current_active_wp_id);

			if (timestamp_firstinside_orbit == 0)
			{
				// Announce that last waypoint was reached
				if (verbose) printf("*** Reached waypoint %u ***\n", cur_wp->seq);
				send_waypoint_reached(cur_wp->seq);
				timestamp_firstinside_orbit = now;
			}

			// check if the MAV was long enough inside the waypoint orbit
			//if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
			if(now-timestamp_firstinside_orbit >= cur_wp->param2*1000)
			{
				if (cur_wp->autocontinue)
				{
					cur_wp->current = 0;
					if (current_active_wp_id == waypoints->size() - 1 && waypoints->size() > 1)
					{
						//the last waypoint was reached, if auto continue is
						//activated restart the waypoint list from the beginning
						current_active_wp_id = 1;
					}
					else
					{
						if (current_active_wp_id + 1 < waypoints->size())
							current_active_wp_id++;
					}

					// Fly to next waypoint
					timestamp_firstinside_orbit = 0;
					send_waypoint_current(current_active_wp_id);
					send_setpoint(current_active_wp_id);
					waypoints->at(current_active_wp_id)->current = true;
					posReached = false;
					//yawReached = false;
					if (verbose) printf("Set new waypoint (%u)\n", current_active_wp_id);
				}
			}
		}
	}
	else
	{
		timestamp_lastoutside_orbit = now;
	}
}

/**
*  @brief main function of process (start here)
*
*  The function parses for program options, sets up some example waypoints and connects to IPC
*/
int main(int argc, char* argv[])
{
	std::string waypointfile;
	config::options_description desc("Allowed options");
	desc.add_options()
					("help", "produce help message")
					("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
					("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
					("config", config::value<std::string>(&configFile)->default_value("config/parameters_waypointplanner.cfg"), "Config file for system parameters")
					("waypointfile", config::value<std::string>(&waypointfile)->default_value(""), "Config file for waypoint")
					;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	mavlink_message_t_subscription_t * comm_sub = mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, NULL);

	paramClient = new PxParamClient(systemid, compid, lcm, configFile, verbose);
	paramClient->setParamValue("POSFILTER", 1.f);
	paramClient->setParamValue("SETPOINTDELAY", 1000000);
	paramClient->setParamValue("PROTOCOLDELAY", 40);
	paramClient->setParamValue("PROTOCOLTIMEOUT", 2000000);
	paramClient->setParamValue("YAWTOLERANCE", 0.1745f);
	paramClient->readParamsFromFile(configFile);


	if (waypointfile.length())
	{
		std::ifstream wpfile;
		wpfile.open(waypointfile);
		if (!wpfile) {
			printf("Unable to open waypoint file\n");
			exit(1); // terminate with error
		}

        while (!wpfile.eof())
        {
        	mavlink_waypoint_t *wp = new mavlink_waypoint_t();

        	int temp = 0;

        	wpfile >> wp->seq;
        	wpfile >> wp->x;
			wpfile >> wp->y;
			wpfile >> wp->z;
			wpfile >> wp->yaw;
			wpfile >> wp->autocontinue;
			wpfile >> temp;
			wp->current = temp;
			wp->orbit = 0.f;
			wpfile >> wp->param1;
			wpfile >> wp->param2;

			char c = (char)wpfile.peek();
			if(c != '\n')
			{
				delete wp;
				break;
			}

        	wp->frame = 1;
			waypoints->push_back(wp);
        }
		wpfile.close();

		//get the new current waypoint
		uint32_t i;
		for(i = 0; i < waypoints->size(); i++)
		{
			if (waypoints->at(i)->current == 1)
			{
				current_active_wp_id = i;
				if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
				//yawReached = false;
				posReached = false;
				//send_waypoint_set_current(current_active_wp_id);
				send_setpoint(current_active_wp_id);
				timestamp_firstinside_orbit = 0;
				break;
			}
		}

		if (i == waypoints->size())
		{
			current_active_wp_id = -1;
			posReached = false;
			timestamp_firstinside_orbit = 0;
		}

	}


	printf("WAYPOINTPLANNER INITIALIZATION DONE, RUNNING..\n");

	while (1)
	{
		lcm_handle (lcm);
	}

	mavlink_message_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
}

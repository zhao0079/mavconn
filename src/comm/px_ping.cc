/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010, 2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
*   @brief Ping utility to measure system communication latency
*
*   @author Lorenz Meier, <mavteam@student.ethz.ch>
*
*/



// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */

// MAVLINK includes
#include <mavlink.h>

#include <vector>

#include "glib.h"
#include "mavconn.h"

using std::string;

struct timeval tv;		  ///< System time
// Settings
int sysid;             ///< The unique system id of this MAV, 0-254. Has to be consistent across the system
int compid = PX_COMP_ID_PING;
int nPing;				  ///< Number of ping packets to send
int pingInterval;         ///< Interval between ping packets
bool silent;              ///< Wether console output should be enabled
bool verbose;             ///< Enable verbose output
bool emitHeartbeat;       ///< Generate a heartbeat with this process
bool debug;               ///< Enable debug functions and output
bool test;                ///< Enable test mode

std::vector<uint64_t> emitTimes; ///< List containing the timestamps when each ping message was sent

/**
* @brief Handle a MAVLINK message received from LCM
*
* The message is forwarded to the serial port.
*
* @param rbuf LCM receive buffer
* @param channel LCM channel
* @param msg MAVLINK message
* @param user LCM user
*/
static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
		const mavlink_message_t* msg, void * user)
{
	//printf("RECEIVED: %d %d %d\n", msg->sysid, msg->compid, msg->msgid);

	// Do not accept messages originating from this component
	if (msg->sysid != sysid || msg->compid != compid)
	{
		//lcm_t* lcm = dynamic_cast<lcm_t*>(user);
		if ((lcm_t*)user == NULL)
		{
			fprintf(stderr, "ERROR: LCM connection broke\n");
		}
		else
		{
			// Everything is alright, decode MAVLink message
			switch (msg->msgid)
			{
			case MAVLINK_MSG_ID_PING:
			{
				mavlink_ping_t ping;
				mavlink_msg_ping_decode(msg, &ping);
				uint64_t r_timestamp = getSystemTimeUsecs();
				if (ping.target_system == 0 && ping.target_component == 0)
				{
					mavlink_message_t r_msg;
					mavlink_msg_ping_pack(sysid, compid, &r_msg, ping.seq, msg->sysid, msg->compid, r_timestamp);
					mavlink_message_t_publish((lcm_t*)user, MAVLINK_MAIN, &r_msg);
				}
				else
				{
					if (ping.target_system == sysid && ping.target_component == compid)
						{
						    // This is a response to a ping request
							uint64_t sendTime = emitTimes.at(ping.seq);
							uint64_t roundTrip = r_timestamp - sendTime;
							printf("Response: SYS: %d\t COMP: %d\t roundtrip time: %lld\n", msg->sysid, msg->compid, roundTrip);
						}
				}
			}
			break;
			default:
				break;
			}
		}
	}
}

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

/**
* @brief Main function to start serial link process
*/
int main(int argc, char* argv[])
{
	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "N" },
			{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "ID" },
			{ "ping-requests", 'n', 0, G_OPTION_ARG_INT, &nPing, "Number of ping requests", "10" },
			{ "ping-interval", 'i', 0, G_OPTION_ARG_INT, &pingInterval, "Interval between ping attempts, in microseconds", "I" },
			{ "silent", 's', 0, G_OPTION_ARG_NONE, &silent, "Be silent", NULL },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- ping other processes and systems");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	// Start process

	if (!silent) printf("\nPING CLIENT STARTED\n");

	// SETUP LCM
	lcm_t* lcm;
	lcm = lcm_create (NULL);
	if (!lcm)
	{
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("Started LCM client..\n");
	}

	// Start thread to handle incoming LCM messages
	// Thread
	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	mavlink_message_t_subscription_t * comm_sub =
			mavlink_message_t_subscribe (lcm, MAVLINK_MAIN, &mavlink_handler, (void*)lcm);
	if (!silent) printf("Subscribed to %s LCM channel.\n", MAVLINK_MAIN);

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Failed to create LCM handling thread: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	// Start to ping system nPing times

	int seq = 0;
	uint64_t time = 0;

	printf("Sending %d ping requests with interval %d, waiting for responses..\n", nPing, pingInterval);

	for (int pingCount = 0; pingCount < nPing; pingCount++)
	{
		mavlink_message_t msg;
		time = getSystemTimeUsecs();
		emitTimes.push_back(time);
		// Send out request for ping responses
		mavlink_msg_ping_pack(sysid, compid, &msg, seq, 0, 0, time);
		seq++;
		mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
		usleep(pingInterval);
		if (debug) printf("Sent MAVLink PING seq: %d\n", seq);
	}

	// Wait for responses
	while(1)
	{
		// Wait for CTRL-C
		// FIXME Implement timeout, loss counter
	}

	// Disconnect from LCM
	mavlink_message_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);

	g_thread_join(lcm_thread);
	exit(EXIT_SUCCESS);
}


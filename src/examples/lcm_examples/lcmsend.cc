/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

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

======================================================================*/

/**
 * @file
 *   @brief LCM example
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <stdio.h>

// MAVLINK message format includes
#include "protocol.h"
#include "mavlink.h"

// LCM transport includes
#include <lcm/lcm.h>
#include "comm/lcm/mavlink_message_t.h"

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// Timer for benchmarking
struct timeval tv;

static void
send_mavlink_message(lcm_t * lcm)
{
	// The ID of this system. The valid range is 1-127
	uint8_t systemID = 100;

	// Instatiate a message and set all fields to zero (else values could be also different from zero)
	mavlink_message_t msg;
	// Pack the liftoff action message into this space, ready for sending
	mavlink_action_t action;
	action.action = MAV_ACTION_LAUNCH;
	action.target = 1; // Send action to MAV 001
	mavlink_msg_action_encode(systemID, 0, &msg, &action);

	// Publish the message on the LCM IPC bus
	mavlink_message_t_publish (lcm, "MAVLINK", &msg);

	///////
	// Benchmarking the latency

	// Store time of sending, assuming sending does not take longer than one second
	gettimeofday(&tv, NULL);
	uint64_t sendTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	// Pack the attitude message
	mavlink_msg_attitude_pack(systemID, 0, &msg, sendTime, 0.0f, 0.1f, 0.2f, 0, 0, 0);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

}

int
main (int argc, char ** argv)
{

	// LCM reference
	lcm_t * lcm;

	lcm = lcm_create (NULL);
	if (!lcm)
	{
		return 1;
	}
	else
	{
		// Take time
		gettimeofday(&tv, NULL);
		uint64_t lastTime = tv.tv_sec * 1000000 + tv.tv_usec;
		// Send content
		send_mavlink_message(lcm);

		// Take time difference
		gettimeofday(&tv, NULL);
		// Difference in milliseconds
		uint64_t diffTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec - lastTime;

		printf("Sent ACTION message over LCM bus, took %f ms\n", diffTime);

		lcm_destroy(lcm);
		return 0;
	}
}


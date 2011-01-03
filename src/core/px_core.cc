/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

(c) 2009 MAVCONN PROJECT

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

// BOOST includes
#include <boost/program_options.hpp>
#include <cstdio>
#include <iostream>
#include <glib.h>
#include <stdio.h>
#include <glibtop.h>
#include <glibtop/cpu.h>
#include <glibtop/mem.h>
#include <glibtop/proclist.h>

// MAVLINK message format includes
#include "mavconn.h"

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// Timer for benchmarking
struct timeval tv;

namespace config = boost::program_options;
using std::string;
using namespace std;

#define COMM_READY_TIMEOUT 200
#define COMM_GCS_TIMEOUT 2000

enum COMM_STATE
{
	COMM_STATE_UNINIT=0,
			COMM_STATE_BOOT,
			COMM_STATE_READY,
			COMM_STATE_GCS_ESTABLISHED
};

enum COMPONENT_STATE
{
	CMP_STATE_UNINIT=0,
			CMP_STATE_OK
};

typedef struct
{
	uint8_t core;
	uint8_t comm;
	uint8_t comm_errors;
	uint8_t subsys_errors;
} system_state_t;

system_state_t static inline mk_system_state_t()
{
	system_state_t s;
	s.core = MAV_STATE_UNINIT;
	s.comm = COMM_STATE_UNINIT;
	s.comm_errors = 0;
	s.subsys_errors = 0;
	return s;
}


// Settings
int systemid;					///< The unique system id of this MAV, 0-255. Has to be consistent across the system
int compid = PX_COMP_ID_CORE;	///< The unique component id of this process.
int systemType = MAV_QUADROTOR;	///< The system type
bool silent;					///< Wether console output should be enabled
bool verbose;					///< Enable verbose output
bool emitHeartbeat;				///< Generate a heartbeat with this process
bool debug;						///< Enable debug functions and output
std::string configFile;			///< Configuration file for parameters

uint64_t currTime;
uint64_t lastTime;

uint64_t lastGCSTime;

PxParamClient* paramClient;

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,const mavlink_message_t* msg, void * user)
{
	if (debug) printf("Received message on channel \"%s\":\n", channel);

	// Handle param messages
	paramClient->handleMAVLinkPacket(msg);

	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_ACTION:
	{
		switch (mavlink_msg_action_get_action(msg))
		{
		case MAV_ACTION_SHUTDOWN:
		{
			if (verbose) std::cerr << "Shutdown received, shutting down system" << std::endl;
			mavlink_message_t response;
			mavlink_sys_status_t status;
			status.status = MAV_STATE_POWEROFF;
			status.mode = MAV_MODE_LOCKED;
			mavlink_msg_sys_status_encode(systemid, compid, &response, &status);
			mavlink_message_t_publish ((lcm_t*)user, "MAVLINK", &response);
			if (system ("halt"))
				if (verbose) std::cerr << "Shutdown failed." << std::endl;

		}
		break;
		case MAV_ACTION_REBOOT:
		{
			if (verbose) std::cerr << "Reboot received, rebooting system" << std::endl;
			mavlink_message_t response;
			mavlink_sys_status_t status;
			status.status = MAV_STATE_POWEROFF;
			status.mode = MAV_MODE_LOCKED;
			mavlink_msg_sys_status_encode(systemid, compid, &response, &status);
			mavlink_message_t_publish ((lcm_t*)user, "MAVLINK", &response);
			if(system ("reboot"))
				if (verbose) std::cerr << "Reboot failed." << std::endl;
		}
		break;
		}
	}
	break;
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		switch(mavlink_msg_heartbeat_get_type(msg))
		{
		case OCU:
			gettimeofday(&tv, NULL);
			uint64_t currTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
			// Groundstation present
			lastGCSTime = currTime;
			if (verbose) std::cout << "Heartbeat received from GCS/OCU " << msg->sysid;
			break;
		}
	}
	break;
	case MAVLINK_MSG_ID_PING:
	{
		mavlink_ping_t ping;
		mavlink_msg_ping_decode(msg, &ping);
		uint64_t r_timestamp = getSystemTimeUsecs();
		if (ping.target_system == 0 && ping.target_component == 0)
		{
			mavlink_message_t r_msg;
			mavlink_msg_ping_pack(systemid, compid, &r_msg, ping.seq, msg->sysid, msg->compid, r_timestamp);
			mavlink_message_t_publish((lcm_t*)user, MAVLINK_MAIN, &r_msg);
		}
	}
	break;
	default:
		if (debug) printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
}

void* lcm_wait(void* lcm_ptr)
		{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle (lcm);
	}
	return NULL;
		}

int main (int argc, char ** argv)
{
	try
	{
		// Handling Program options

		config::options_description desc("Allowed options");
		desc.add_options()
								("help", "produce help message")
								("acid,a", config::value<int>(&systemid)->default_value(42), "ID of this system, 1-127")
								("heartbeat,h", config::bool_switch(&emitHeartbeat)->default_value(true), "send heartbeat signals")
								("silent,s", config::bool_switch(&silent)->default_value(false), "surpress outputs")
								("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
								("debug,d", config::bool_switch(&debug)->default_value(false), "Output debug messages to console")
								("config,c", config::value<std::string>(&configFile)->default_value("config/parameters_core.cfg"), "Config file for system parameters")
								;
		config::variables_map vm;
		config::store(config::parse_command_line(argc, argv, desc), vm);
		config::notify(vm);

		if (vm.count("help"))
		{
			std::cout << desc << std::endl;
			return 1;
		}

		lcm_t * lcm;

		lcm = lcm_create ("udpm://");
		if (!lcm)
			return 1;

		// Initialize parameter client before subscribing (and receiving) MAVLINK messages
		paramClient = new PxParamClient(systemid, compid, lcm, configFile, verbose);
		paramClient->setParamValue("SYS_ID", systemid);
		paramClient->readParamsFromFile(configFile);

		mavlink_message_t_subscription_t * commSub =
				mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, lcm);

		// Thread
		GThread* lcm_thread;
		GError* err;

		if( !g_thread_supported() )
		{
			g_thread_init(NULL);
			// Only initialize g thread if not already done
		}

		if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
		{
			printf("Thread create failed: %s!!\n", err->message );
			g_error_free ( err ) ;
		}

		// Initialize system information library
		glibtop_init();

		glibtop_cpu cpu;
		glibtop_mem memory;
		glibtop_proclist proclist;

		printf("\nPX CORE STARTED\n");

		while (1)
		{

			// Send heartbeat if enabled
			if (emitHeartbeat)
			{
				gettimeofday(&tv, NULL);
				uint64_t currTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

				if (currTime - lastTime > 1000000)
				{
					// SEND OUT TIME MESSAGE
					// send message as close to time aquisition as possible
					mavlink_message_t msg;

					mavlink_msg_system_time_pack(systemid, compid, &msg, currTime);
					mavlink_message_t_publish (lcm, "MAVLINK", &msg);

					lastTime = currTime;
					if (verbose) std::cout << "Emitting heartbeat" << std::endl;

					// SEND HEARTBEAT

					// Pack message and get size of encoded byte string
					mavlink_msg_heartbeat_pack(systemid, compid, &msg, systemType, MAV_AUTOPILOT_PIXHAWK);
					mavlink_message_t_publish (lcm, "MAVLINK", &msg);

					// GET SYSTEM INFORMATION
					glibtop_get_cpu (&cpu);
					glibtop_get_mem(&memory);

					if (verbose)
					{
						printf("CPU TYPE INFORMATIONS \n\n"
						"Cpu Total : %ld \n"
						"Cpu User : %ld \n"
						"Cpu Nice : %ld \n"
						"Cpu Sys : %ld \n"
						"Cpu Idle : %ld \n"
						"Cpu Frequences : %ld \n",
						(unsigned long)cpu.total,
						(unsigned long)cpu.user,
						(unsigned long)cpu.nice,
						(unsigned long)cpu.sys,
						(unsigned long)cpu.idle,
						(unsigned long)cpu.frequency);

						float load = ((float)(unsigned long)cpu.total-(float)(unsigned long)cpu.idle) / (float)(unsigned long)cpu.total;
						printf("\nLOAD: %f %%\n\n", load*100.0f);

						printf("\nMEMORY USING\n\n"
						"Memory Total : %ld MB\n"
						"Memory Used : %ld MB\n"
						"Memory Free : %ld MB\n"
						"Memory Shared: %ld MB\n"
						"Memory Buffered : %ld MB\n"
						"Memory Cached : %ld MB\n"
						"Memory user : %ld MB\n"
						"Memory Locked : %ld MB\n",
						(unsigned long)memory.total/(1024*1024),
						(unsigned long)memory.used/(1024*1024),
						(unsigned long)memory.free/(1024*1024),
						(unsigned long)memory.shared/(1024*1024),
						(unsigned long)memory.buffer/(1024*1024),
						(unsigned long)memory.cached/(1024*1024),
						(unsigned long)memory.user/(1024*1024),
						(unsigned long)memory.locked/(1024*1024));

						int which = 0, arg = 0;
						glibtop_get_proclist(&proclist,which,arg);
						printf("%ld\n%ld\n%ld\n",
						(unsigned long)proclist.number,
						(unsigned long)proclist.total,
						(unsigned long)proclist.size);
					}
				}
			}
			usleep(5000);
		}

		mavlink_message_t_unsubscribe (lcm, commSub);
		lcm_destroy (lcm);
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}


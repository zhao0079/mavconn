/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
*   @brief Interface to a NMEA 0183 standard GPS receiver
*
*   This process connects any external GPS device to the system's MAVLink/LCM bus.
*   @see http://en.wikipedia.org/wiki/NMEA_0183
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
// BOOST includes
#include <boost/program_options.hpp>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <nmea/nmea.h> /* NMEA GPS data parsing */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

// MAVLINK includes
#include <mavlink.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>
#include "mavconn.h"

#include <gps.h>

namespace config = boost::program_options;
using std::string;
using namespace std;

struct timeval tv;		  ///< System time
// Settings
int systemid;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = MAV_COMP_ID_GPS;
string port;              ///< The serial port name, e.g. /dev/ttyUSB0
std::string host;		  ///< The GPSD host
bool silent;              ///< Wether console output should be enabled
bool verbose;             ///< Enable verbose output
bool emitHeartbeat;       ///< Generate a heartbeat with this process
bool debug;               ///< Enable debug functions and output
bool test;                ///< Enable test mode

lcm_t* lcm;               ///< Reference to LCM bus

///**
//* @brief Handle a MAVLINK message received from LCM
//*
//* The message is forwarded to the serial port.
//*
//* @param rbuf LCM receive buffer
//* @param channel LCM channel
//* @param msg MAVLINK message
//* @param user LCM user
//*/
//static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
//		const mavlink_message_t* msg, void * user)
//{
//	int fd = *(static_cast<int*>(user));
//	if (fd == -1)
//	{
//		std::cerr << "Unable to send message over serial port. Port " << port << " not ready!" << std::endl;
//	}
//	else
//	{
//		// Do nothing, this could later be used to send commands to the GPS receiver
//	}
//}
//
//void* lcm_wait(void* lcm_ptr)
//		{
//	lcm_t* lcm = (lcm_t*) lcm_ptr;
//	// Blocking wait for new data
//	while (1)
//	{
//		if (debug) printf("Waiting for LCM data\n");
//		lcm_handle (lcm);
//	}
//	return NULL;
//		}

void px_gps_fetch(struct gps_data_t *gps_data, char *buf, size_t size, int num)
{
   printf("In fetchjob\n");
}

/**
* @brief Main function to start serial link process
*/
int main(int argc, char* argv[])
{

	// Handling Program options

	config::options_description desc("Allowed options");
	desc.add_options()
					("help", "produce help message")
					("sysid,a", config::value<int>(&systemid)->default_value(42), "ID of this system, 1-127")
					("host,h", config::value<string>(&host)->default_value("127.0.0.1"), "Host running GPSD, IP or DNS address")
					("port,p", config::value<string>(&port)->default_value("2947"), "GPSD port")
					("silent,s", config::bool_switch(&silent)->default_value(false), "surpress outputs")
					("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
					("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
					;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	// SETUP SERIAL PORT

	if (!silent) printf("GPSD INTERFACE STARTED\n");

	// Exit if opening port failed
	// Open the serial port.

	// SETUP LCM
	lcm = lcm_create (NULL);
	if (!lcm)
	{
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("Started LCM client..\n");
	}

//	// Start thread to handle incoming LCM messages
//	// Thread
//	GThread* lcm_thread;
//	GError* err;
//
//	if( !g_thread_supported() )
//	{
//		g_thread_init(NULL);
//		// Only initialize g thread if not already done
//	}

//	mavlink_message_t_subscription_t * comm_sub =
//			mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, (void*)fd_ptr);
//	if (!silent) printf("Subscribed to %s LCM channel.\n", "MAVLINK");

	// Run indefinitely while the LCM and serial threads handle the data
	if (!silent) printf("\nREADY, waiting for GPSD data.\n");

//	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
//	{
//		printf("Failed to create LCM handling thread: %s!!\n", err->message );
//		g_error_free ( err ) ;
//	}



	// Start GPSD interface and forward data to MAVLink/LCM
	// for more details, see: http://gpsd.berlios.de/client-howto.html

	struct gps_data_t* gpsdata = 0;
	gpsdata = gps_open(host.c_str(), port.c_str());

	if (!gpsdata)
	{
		// Exit with error
		perror("ERROR: CANNOT CONNECT TO GPSD SERVICE");
		mavlink_statustext_t statustext;
		mavlink_message_t msg;
		sprintf((char*)&statustext.text, "ERROR: Could not connect to GPSD");
		mavlink_msg_statustext_encode(systemid, compid, &msg, &statustext);
		mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
		usleep(50000);
		exit(EXIT_FAILURE);
	}

	// Request GPS data
#if (GPSD_API_MAJOR_VERSION > 3)
	gps_stream(gpsdata, WATCH_ENABLE, NULL);
#else
	gps_query(gpsdata, "w+x\n");
#endif

	double lastUpdateTime = 0;
	uint64_t sysTime = getSystemTimeUsecs();
	float maxUpdateRate = 10; // 10 Hz max. update rate
	unsigned int waitInterval = 1000000.0f/maxUpdateRate;
	int maxIgnoreCount = 3; // If the max ignore count is exceeded, the process stops
	int ignoreCount = 0;

	if (gpsdata != NULL)
	{
		while(1)
		{
			// Blocking wait
			gps_poll(gpsdata);
			if (gpsdata != NULL)
			{
				if (ignoreCount > maxIgnoreCount)
				{
					perror("ERROR: GPS data stays constantly invalid");
					mavlink_message_t msg;
					mavlink_statustext_t statustext;
					sprintf((char*)&statustext.text, "ERROR, GPS DIED: Data %d times invalid!", ignoreCount);
					mavlink_msg_statustext_encode(systemid, compid, &msg, &statustext);
					mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
					exit(EXIT_FAILURE);
				}
				
				if (gpsdata->fix.time != gpsdata->fix.time) // NaN check
				{
					// Check if fix time is valid
					// if fix time is invalid, receiver is not ready
					// but data might keep coming in, so slow down to
					// 10 Hz update rate until receiver is ready
					if (getSystemTimeUsecs() - sysTime < waitInterval)
					{
						usleep(waitInterval);
						// Continue to next attempt
						continue;
					}
				}
				if (debug) printf("%f Got GPS data..\n", gpsdata->fix.time);

				if (gpsdata->fix.time == 0)
				{
					ignoreCount++;
					if (debug)
					{
						
						printf(" data is invalid: ignored, continuing.\n");
						continue;
						
					}
				}

				// FIXME Required 3D fix
				if ((gpsdata->fix.time == lastUpdateTime))// lastFixMode))
				{
					ignoreCount++;
					usleep(waitInterval);
					
					if (debug) printf(" data is duplicate: ignored, continuing.\n");
					// Continue with next GPS fix, this one has already been handled previously
					continue;
				}
				else
				{
					lastUpdateTime = gpsdata->fix.time;
				}
				
				sysTime = getSystemTimeUsecs();

				// GPS data assumed to be valid
				// resetting ignore count
				ignoreCount = 0;

				// Forward satellite info
				// FIXME LIMIT TO 1 Hz
				mavlink_message_t msg;
				mavlink_gps_status_t status;
#if (GPSD_API_MAJOR_VERSION > 3)
				for (int i = 0; i < gpsdata->satellites_visible; i++)
				{
					status.satellites_visible = gpsdata->satellites_visible;
					#else
				for (int i = 0; i < gpsdata->satellites; i++)
				{
					status.satellites_visible = gpsdata->satellites;
#endif
					status.satellite_prn[i] = gpsdata->PRN[i];
					status.satellite_used[i] = gpsdata->used[i];
					status.satellite_elevation[i] = gpsdata->elevation[i];
					status.satellite_azimuth[i] = ((gpsdata->azimuth[i]/360.0f)*255.0f); // Scale 0-360 deg. to 0-255
					status.satellite_snr[i] = gpsdata->ss[i];
				}

				// Send message
				mavlink_msg_gps_status_encode(systemid, compid, &msg, &status);
				mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);

				// FIXME Currently required a 3D fix
				if (gpsdata->fix.mode > 2)
				{
					// New data arrived, forward to LCM

					mavlink_message_t msg;
					mavlink_gps_raw_t gps;
					gps.usec = gpsdata->fix.time * 1000000llU;
					gps.fix_type = gpsdata->fix.mode;
					gps.lat = gpsdata->fix.latitude;
					gps.lon = gpsdata->fix.longitude;
					if (gpsdata->fix.mode == 3)
					{
						// Altitude is valid
						gps.alt = gpsdata->fix.altitude;
					}
					else
					{
						// Altitude is invalid
						gps.alt = 0;
					}
					gps.usec = gpsdata->fix.time;
					gps.v = gpsdata->fix.speed;
					mavlink_msg_gps_raw_encode(systemid, compid, &msg, &gps);
					mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
					if (debug)
					{
#if (GPSD_API_MAJOR_VERSION > 3)
						printf(" GPS FIX: lat: %f lon: %f alt: %f (%d/%d satellites used)\n", gps.lat, gps.lon, gps.alt, gpsdata->satellites_used, gpsdata->satellites_visible);
#else
						printf(" GPS FIX: lat: %f lon: %f alt: %f (%d/%d satellites used)\n", gps.lat, gps.lon, gps.alt, gpsdata->satellites_used, gpsdata->satellites);
#endif

					}
				}
				else
				{
#if (GPSD_API_MAJOR_VERSION > 3)
					if (debug) printf(" NO GPS FIX (%d/%d satellites used)\n", gpsdata->satellites_used, gpsdata->satellites_visible);
#else
					if (debug) printf(" NO GPS FIX (%d/%d satellites used)\n", gpsdata->satellites_used, gpsdata->satellites);
#endif


					// Zurich: lat: 47.387707 lon: 8.499325
				}

				if (debug)
				{
#if (GPSD_API_MAJOR_VERSION > 3)
					for (int i = 0; i < gpsdata->satellites_visible; i++)
					{
						printf("    %2.2d: %2.2d %3.3d %3.3f %c\n", gpsdata->PRN[i], gpsdata->elevation[i], gpsdata->azimuth[i], gpsdata->ss[i], gpsdata->used[i]? 'Y' : 'N');
					}
#else
					for (int i = 0; i < gpsdata->satellites; i++)
					{
						printf("    %2.2d: %2.2d %3.3d %d %c\n", gpsdata->PRN[i], gpsdata->elevation[i], gpsdata->azimuth[i], gpsdata->ss[i], gpsdata->used[i]? 'Y' : 'N');
					}
#endif
				}

			}
			else
			{
				if (!silent) fprintf(stderr, "ERROR: Connection to GPS broke. Exiting.\n");
				exit(EXIT_FAILURE);
			}

		}

		// Disconnect from LCM
		//mavlink_message_t_unsubscribe (lcm, comm_sub);
		lcm_destroy (lcm);

		//g_thread_join(lcm_thread);
		exit(EXIT_SUCCESS);
	}
	else
	{
		mavlink_statustext_t statustext;
		mavlink_message_t msg;
		sprintf((char*)&statustext.text, "ERROR: Could not connect to GPSD");
		mavlink_msg_statustext_encode(systemid, compid, &msg, &statustext);
		mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
		usleep(50000);
		lcm_destroy (lcm);
		gps_close(gpsdata);
		//g_thread_join(lcm_thread);
		if (!silent) fprintf(stderr, "ERROR: Could not connect to GPSD on host:%s port:%s. Exiting.\n", host.c_str(), port.c_str());
		exit(EXIT_FAILURE);
	}
}


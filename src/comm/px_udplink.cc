/*=====================================================================

 MAVCONN Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

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
 *   @brief UDPLink
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Bryan Godbolt <godbolt@ualberta.ca>
 *
 */

/* POSIX Headers */
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdlib>
#include <fcntl.h>
#include <ctime>
#include <errno.h>
#include <netdb.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#endif
#include <boost/program_options.hpp>
#include <glib.h>
#include "mavconn.h"

// External namespaces
namespace config = boost::program_options;
// Settings
int systemid;
int componentid;
uint8_t mode;
std::string host = "localhost"; ///< host name for UDP server
std::string port = "14550"; ///< port for UDP server to open connection
bool silent; ///< silent run mode
bool verbose; ///< verbose run mode
bool emitHeartbeat; ///< tells the program to emit heart beats regularly
bool dataOnly; ///< send only data, without video stream
bool debug; ///< debug mode

int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in gcAddr;
struct sockaddr_in locAddr;
struct sockaddr_in fromAddr;
int messageBurstSize = 20; ///< Number of image messages to send in a row - controls how much bandwidth the image consumes

ssize_t recsize;
socklen_t fromlen;

struct timeval tv;

lcm_t* lcm;

/**
 * @brief Handle a MAVLINK message over LCM
 *
 * @param rbuf LCM receive buffer
 * @param channel LCM channel
 * @param msg MAVLINK message
 * @param user LCM user
 */
static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,
		const mavlink_message_t* msg, void * user)
{
	// Send message over UDP
	int link = *(static_cast<int*>(user));
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int messageLength = mavlink_msg_to_send_buffer(buf, msg);

	if (verbose)
	{
		printf("(SYS: %d/COMP: %d/LCM->UDP) Received message from LCM with %i payload bytes and %i total length\n",
				msg->sysid, msg->compid, msg->len, messageLength);
		for (int i = 0; i < messageLength; i++)
		{
			fprintf(stderr, "%02x ", buf[i]);
		}
		fprintf(stderr, "\n");
	}

	// Send over UDP
	int bytes_sent = sendto(link, buf, messageLength, 0, (struct sockaddr*) &gcAddr,
			sizeof(struct sockaddr_in));
	//	extern int errno;


	if (bytes_sent < 1)
	{
		// Error handling
		perror("Could not send over UDP socket");
		fprintf(stderr, "Target address and host: %s:%s\n", host.c_str(), port.c_str());
	}
	else
	{
		if (debug) fprintf(stderr, "SENT %d BYTES OVER UDP TO %s:%s", bytes_sent, host.c_str(), port.c_str());
	}
}

void* lcm_wait(void* lcm_ptr)
		{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle(lcm);
	}
	return NULL;
		}

void* udp_wait(void* lcm_ptr)
		{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	// READ PENDING BYTES ON UDP LINK
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	while (1)
	{
		int recsize = recvfrom(sock, (void *) buf, MAVLINK_MAX_PACKET_LEN, 0,
				(struct sockaddr *) &gcAddr, &fromlen);
		if (recsize < 1)
		{
			// An error occured
		}

		// Something received - print out all bytes and parse packet
		mavlink_message_t msg;
		mavlink_status_t status;

		for (int i = 0; i < recsize; ++i)
		{
			uchar tmpchar = buf[i];
			if (debug) printf("%02x ", tmpchar);
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
			{
				// Packet received
				printf("\n(SYS: %d/COMP: %d/UDP) Received message from UDP with %i payload bytes and %i total length\n",
						msg.sysid, msg.compid, msg.len, recsize);
				mavlink_message_t_publish (lcm, "MAVLINK", &msg);
			}
		}

	}
	return NULL;
		}

int main(int argc, char* argv[])
{
	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "sysid", 'a', 0, G_OPTION_ARG_INT, &systemid, "ID of this system", NULL },
			{ "compid", 'c', 0, G_OPTION_ARG_INT, &componentid, "ID of this component", NULL },
			{ "host", 'r', 0, G_OPTION_ARG_STRING, &host, "Remote host", "hostname" },
			{ "port", 'p', 0, G_OPTION_ARG_STRING, &port, "Remote port", "port, 14550" },
			{ "silent", 's', 0, G_OPTION_ARG_NONE, &silent, "Be silent", NULL },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate between LCM broadcast bus and ground control link");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	// Handling program options done

	// Print the basic configuration
	printf("Connecting to host %s:%s\n", host.c_str(), port.c_str());

	//		sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//		struct sockaddr_in gcAddr;
	//		struct sockaddr_in locAddr;
	//		struct sockaddr_in fromAddr;
	//		ssize_t recsize;
	//		socklen_t fromlen;

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(0);//htons(14551);

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
	if (-1 == bind(sock, (struct sockaddr *) &locAddr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
	}

	/* Attempt to make it non blocking */
	// if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	//    {
	//	fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
	//	close(sock);
	//	exit(EXIT_FAILURE);
	//    }

	// Get internet address for hostname
	struct hostent *hp = gethostbyname(host.c_str());

	if (hp == NULL)
	{
		fprintf(stderr,"Unknown remote host address: %s, please check spelling.\n", host.c_str());
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("Using IP address: %s for host %s\n", inet_ntoa( *( struct in_addr*)( hp -> h_addr)), hp->h_name);

		// If more than one address exists
		if (hp->h_addr_list[1] != NULL)
		{
			printf("Additional (unused) addresses of this host (%s) are:\n", hp->h_name);
			int i=1;
			while (hp->h_addr_list[i] != NULL)
			{
				printf( "\t%s \n", inet_ntoa( *( struct in_addr*)( hp -> h_addr_list[i])));
				i++;
			}
		}
	}


	// Set host and port
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = hp->h_addrtype;
	gcAddr.sin_addr.s_addr = ((struct in_addr *)(hp->h_addr))->s_addr;//inet_addr("127.0.0.1");//inet_addr(host.c_str());
	gcAddr.sin_port = htons(atoi(port.c_str()));

	lcm = lcm_create ("udpm://");
	if (!lcm)
	{
		return 1;
	}

	mavlink_message_t_subscription_t * comm_sub =
			mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, &sock);

	// Initialize LCM receiver thread
	GThread* lcm_thread;
	GThread* udp_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread creation failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	if( (udp_thread = g_thread_create((GThreadFunc)udp_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread creation failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	printf("MAVLINK_BRIDGE_UDP INITIALIZATION DONE, RUNNING..\n");

	while (1)
	{
		sleep(1); // Sleep one second
	}
	mavlink_message_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	close(sock);

	g_thread_join(lcm_thread);
	g_thread_join(udp_thread);
	exit(0);
}


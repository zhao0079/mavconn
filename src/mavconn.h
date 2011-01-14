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
 *   @brief MAVCONN Lightweight Robotics Middleware main header
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup comm */
/*@{*/


#ifndef _MAVCONN_H_
#define _MAVCONN_H_

#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

// MAVLINK message format includes
#include <mavlink.h>

// OpenCV types
//#include <cxtypes.h>

// LCM transport includes
#include <lcm/lcm.h>
#include "comm/lcm/mavlink_message_t.h"

// Time
#include <sys/time.h>
#include <time.h>

// Parameter client
#include "core/PxParamClient.h"

// ROS support
#define PX_ROS_ENABLED	0


enum MAVCONN_COMPONENT_IDS
{
	PX_COMP_ID_CORE = 100,
	PX_COMP_ID_PING = 101,
	PX_COMP_ID_ASCTEC = 102,
	PX_COMP_ID_MESSENGER = 103,
	PX_COMP_ID_CAMERA = 110,
	PX_COMP_ID_TRACKER = 120,
	PX_COMP_ID_MULTITRACKER = 121,
	PX_COMP_ID_POSITIONHOLD = 122,
	PX_COMP_ID_MAVLINK_BRIDGE_SERIAL = 130,
	PX_COMP_ID_MAVLINK_BRIDGE_UDP = 131,
	PX_COMP_ID_MAVLINK_BRIDGE_VICON = 132
};

#define MAVLINK_MAIN "MAVLINK"
#define MAVLINK_IMAGES "IMAGES"

static inline uint64_t getSystemTimeUsecs()
{
	struct timeval tv;		  //System time
	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
}

//static inline void sendSystemMessage(int compd, std::string message)
//{
//	mavlink_message_t msg;
//	mavlink_statustext_t statustext;
//	sprintf((char*)&statustext.text, "ERROR, GPS DIED: Data %d times invalid!", ignoreCount);
//	mavlink_msg_statustext_encode(systemid, compid, &msg, &statustext);
//	mavlink_message_t_publish(lcm, MAVLINK_MAIN, &msg);
//}

// FIXME
static inline int getSystemID(void)
{
	static bool isCached = false;
	// Return 42 on error or no config file present
	static int systemId = 42;

	if (!isCached)
	{
		// Read config file
		std::string line;
		std::ifstream configFile("/etc/mavconn/mavconn.conf");
		if (configFile.is_open())
		{
			while (configFile.good())
			{
				std::string key;
				int value;
				configFile >> key;
				configFile >> value;

				if (value > 0 && value < 256)
				{
					systemId = value;
				}
			}
		}

	}

	return systemId;
}

// FIXME: Camera struct is a little large currently
struct Camera_t
{
	uint64_t id; 	  ///< Unique ID of the camera, e.g. the Point Grey product id
	float caldata[9]; ///< Camera intrinsics
	float toBody[16]; ///< Transformation matrix from camera to body coordinate frame
};

/**struct CamImage_t
{
	uint64_t utime; ///< Milliseconds since unix epoch (01/01/1970, 00:00)
	uint32_t seq;   ///< Image sequence, e.g. to detect at which rate images are processed
	Camera_t cam;   ///< ID of the capturing camera
	IplImage img;   ///< The image container, as IPLImage
};
*/

#ifndef isnan
# define isnan(x) \
		(sizeof (x) == sizeof (long double) ? isnan_ld (x) \
				: sizeof (x) == sizeof (double) ? isnan_d (x) \
						: isnan_f (x))
static inline int isnan_f  (float       x) { return x != x; }
static inline int isnan_d  (double      x) { return x != x; }
static inline int isnan_ld (long double x) { return x != x; }
#endif

#ifndef isinf
# define isinf(x) \
		(sizeof (x) == sizeof (long double) ? isinf_ld (x) \
				: sizeof (x) == sizeof (double) ? isinf_d (x) \
						: isinf_f (x))
static inline int isinf_f  (float       x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_d  (double      x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_ld (long double x)
{ return !isnan (x) && isnan (x - x); }
#endif

#ifndef isnumber
# define isnumber(x) \
		(!isnan(x) && !isinf(x))
#endif

#endif //_PX_H_

/*@}*/

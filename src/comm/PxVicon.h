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
 *   @brief PxVicon
 *
 *   @author Lionel Heng <hengli@student.ethz.ch>
 *
 */

#ifndef PXVICON_H_
#define PXVICON_H_

#include "ViconClient.h"

using namespace ViconDataStreamSDK::CPP;

typedef struct
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
	double timestamp;
} PxViconPose;

class PxVicon
{
public:
	PxVicon();
	bool connect(const std::string& viconHostName, bool transmitMulticast);
	bool disconnect(void);

	bool waitForFrame(void);

	PxViconPose getPose(void);

	static double getTime(void);

private:
	uint32_t getFrameNumber(void) const;
	double getLatency(void) const;

	std::string toString(const bool value) const;
	std::string toString(const Direction::Enum direction) const;
	std::string toString(const DeviceType::Enum deviceType) const;
	std::string toString(const Unit::Enum unit) const;

	Client viconClient;
	bool transmitMulticast;

	uint32_t lastFrameNumber;

	double timeout;
};

#endif

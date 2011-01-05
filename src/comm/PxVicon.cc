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

#include "PxVicon.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <sys/time.h>

#include "PxTransform.h"

PxVicon::PxVicon()
 : transmitMulticast(false)
 , lastFrameNumber(0)
 , timeout(0.5)
{

}

bool
PxVicon::connect(const std::string& viconHostName, bool transmitMulticast)
{
	this->transmitMulticast = transmitMulticast;

	// Connect to a server
	std::cerr << "[VICON] Connecting to VICON at " << viconHostName << " ";
	while (!viconClient.IsConnected().Connected)
	{
		// Direct connection
		viconClient.Connect(viconHostName);

		// Multicast connection
		// viconClient.ConnectToMulticast(viconHostName, "224.0.0.0");

		std::cerr << "." << std::flush;
		usleep(500000);
	}
	std::cerr << std::endl;

	viconClient.EnableSegmentData();

	std::cerr << "[VICON] Segment Data Enabled: "
			  << toString(viconClient.IsSegmentDataEnabled().Enabled)
			  << std::endl;

	// Set the streaming mode
	// viconClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
	viconClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
	// viconClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

	// Set the global up axis
	viconClient.SetAxisMapping(Direction::Left,
							   Direction::Forward,
							   Direction::Up); // Z-up
	// viconClient.SetGlobalUpAxis(Direction::Forward,
	//                           Direction::Up,
	//                           Direction::Right); // Y-up

	Output_GetAxisMapping axisMapping = viconClient.GetAxisMapping();
	std::cerr << "[VICON] Axis Mapping:"
			  << " X-" << toString(axisMapping.XAxis)
			  << " Y-" << toString(axisMapping.YAxis)
			  << " Z-" << toString(axisMapping.ZAxis) << std::endl;

	// Discover the version number
	Output_GetVersion version = viconClient.GetVersion();
	std::cerr << "[VICON] Version: " << version.Major << "."
									 << version.Minor << "."
									 << version.Point << std::endl;

	if (transmitMulticast)
	{
		viconClient.StartTransmittingMulticast("localhost", "224.0.0.0");
	}

	return true;
}

bool
PxVicon::disconnect(void)
{
	if (transmitMulticast)
	{
		viconClient.StopTransmittingMulticast();
	}

	// Disconnect and dispose
	viconClient.Disconnect();

	return true;
}

bool
PxVicon::waitForFrame(void)
{
	bool newFrame = false;
	double startTime = getTime();

	// Wait for a frame
	while (!newFrame)
	{
		if (viconClient.GetFrame().Result == Result::Success)
		{
			uint32_t frameNumber = getFrameNumber();
			if (lastFrameNumber != frameNumber)
			{
				lastFrameNumber = frameNumber;
				newFrame = true;
				break;
			}
		}

		if (getTime() - startTime > timeout)
		{
			return false;
		}

		usleep(10000);
	}

	return true;
}

PxViconPose
PxVicon::getPose(void)
{
	PxViconPose pose;
	pose.timestamp = getTime();

	assert(viconClient.GetSubjectCount().SubjectCount == 1);

	std::string subjectName = viconClient.GetSubjectName(0).SubjectName;

	assert(viconClient.GetSegmentCount(subjectName).SegmentCount == 1);

	std::string segmentName =
		viconClient.GetSegmentName(subjectName, 0).SegmentName;

	// Get the global segment translation
	Output_GetSegmentGlobalTranslation translation =
		viconClient.GetSegmentGlobalTranslation(subjectName, segmentName);

	// Get the global segment rotation in EulerXYZ co-ordinates
	Output_GetSegmentGlobalRotationEulerXYZ rotation =
		viconClient.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName);

	PxTransform t1;
	t1.identity();
	t1.setRotation(- rotation.Rotation[2],
				   - rotation.Rotation[1],
				   - rotation.Rotation[0]);

	t1.getRotation(pose.roll, pose.pitch, pose.yaw);

	PxTransform t2;
	t2.identity();
	t2.setRotation(0.0, - M_PI_2, 0.0);

	t1.leftMultiply(t2);

	t1.getRotation(pose.roll, pose.pitch, pose.yaw);

	pose.yaw -= M_PI_2;
	pose.pitch = - pose.pitch;
	pose.roll = - pose.roll;

	// convert values from mm to m
	pose.x = translation.Translation[1] * 0.001;
	pose.y = translation.Translation[0] * 0.001;
	pose.z = - translation.Translation[2] * 0.001;

	return pose;
}

double
PxVicon::getTime(void)
{
	 struct timeval tv;

	 gettimeofday(&tv, NULL);

	 return static_cast<double>(tv.tv_sec) +
			 static_cast<double>(tv.tv_usec) / 1000000.0;
}

uint32_t
PxVicon::getFrameNumber(void) const
{
	Output_GetFrameNumber frameNumber = viconClient.GetFrameNumber();
	return frameNumber.FrameNumber;
}

double
PxVicon::getLatency(void) const
{
	return viconClient.GetLatencyTotal().Total;
}

std::string
PxVicon::toString(const bool value) const
{
    return value ? "True" : "False";
}

std::string
PxVicon::toString(const Direction::Enum direction) const
{
	switch (direction)
	{
	case Direction::Forward:
		return "Forward";
	case Direction::Backward:
		return "Backward";
	case Direction::Left:
		return "Left";
	case Direction::Right:
		return "Right";
	case Direction::Up:
		return "Up";
	case Direction::Down:
		return "Down";
	default:
		return "Unknown";
	}
}

std::string
PxVicon::toString(const DeviceType::Enum deviceType) const
{
	switch (deviceType)
	{
	case DeviceType::ForcePlate:
		return "ForcePlate";
	case DeviceType::Unknown:
	default:
		return "Unknown";
	}
}

std::string
PxVicon::toString(const Unit::Enum unit) const
{
	switch (unit)
	{
	case Unit::Meter:
		return "Meter";
	case Unit::Volt:
		return "Volt";
	case Unit::NewtonMeter:
		return "NewtonMeter";
	case Unit::Newton:
		return "Newton";
	case Unit::Unknown:
	default:
		return "Unknown";
	}
}

#include <boost/program_options.hpp>
#include "mavconn.h"
#include <signal.h>
#include <gsl/gsl_matrix.h>

#if PX_ROS_ENABLED
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif

#include "PxVicon.h"

std::string viconAddress;
double frequency;

bool quit = false;
uint32_t numMessages = 0;

const double EPSILON = 0.0001;

namespace config = boost::program_options;

void shutdown(int sig)
{
	if (sig == SIGINT)
	{
		quit = true;
	}
}

char rotor(void)
{
	static uint8_t count = 0;
	static char displayChar[4] = {'|', '/', '-', '\\'};

	++count;
	if (count > 3)
	{
		count = 0;
	}

	return displayChar[count];
}

void* infoThread(void* clientData)
		{
	double lastTime = PxVicon::getTime();

	while (!quit)
	{
		if (PxVicon::getTime() - lastTime > 1.0)
		{
			double rate = static_cast<double>(numMessages) /
					(PxVicon::getTime() - lastTime);

			fprintf(stderr, "\rPublishing Pose at %.1f Hz  %c   ",
					rate, rotor());

			lastTime = PxVicon::getTime();
			numMessages = 0;
		}

		usleep(100000);
	}
	pthread_exit(NULL);
		}

int main(int argc, char** argv)
{
	config::options_description desc("Allowed options");
	desc.add_options()
		("hostname,h", config::value<std::string>(&viconAddress)->default_value("vispc43.inf.ethz.ch"), "Host name of Vicon server")
		("frequency,f", config::value<double>(&frequency)->default_value(100.0), "Data frequency of pose updates")
		;

	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	lcm_t* lcm = lcm_create("udpm://");

	if (!lcm)
	{
		return 1;
	}

	PxVicon vicon;

	vicon.connect(viconAddress, false);

	signal(SIGINT, shutdown);

	pthread_t thread;
	pthread_create(&thread, NULL, infoThread, NULL);

	mavlink_message_t msg;
	mavlink_attitude_t attMsg;
	mavlink_local_position_t posMsg;

	uint8_t systemid = getSystemID();
	uint8_t componentid = PX_COMP_ID_MAVLINK_BRIDGE_VICON;

	PxViconPose lastPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	bool first = true;

	double lastTime = PxVicon::getTime();
	double timeout = 1.0 / frequency;

	while (!quit)
	{
		if (PxVicon::getTime() - lastTime > timeout)
		{
			if (vicon.waitForFrame())
			{
				PxViconPose pose = vicon.getPose();

				if (fabs(pose.x) < EPSILON &&
						fabs(pose.y) < EPSILON &&
						fabs(pose.z) < EPSILON)
				{
					// zero pose due to markers outside Vicon's field of view
					continue;
				}

				++numMessages;

				if (first)
				{
					lastPose = pose;
					first = false;
					continue;
				}

				double dt = pose.timestamp - lastPose.timestamp;

				double droll = pose.roll - lastPose.roll;
				double dpitch = pose.pitch - lastPose.pitch;
				double dyaw = pose.yaw - lastPose.yaw;

				// FOR SIMULATING A VISION PROCESS
				mavlink_msg_vicon_position_estimate_pack(systemid, componentid,
						&msg, getSystemTimeUsecs(),
						pose.x,
						pose.y,
						pose.z,
						pose.roll,
						pose.pitch,
						pose.yaw);

				mavlink_message_t_publish (lcm, "MAVLINK", &msg);

				// ROS Output ****************************************************
#if PX_ROS_ENABLED
				// Definition of ROS message:
				// 		http://www.ros.org/doc/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html

				ros::init(argc, argv, "topic_set");		// initialize ROS
				ros::NodeHandle n;						// create a handle to this process node
				ros::Publisher position_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);

				geometry_msgs::PoseWithCovarianceStamped ros_msg;
				ros_msg.stamp = (ros::Time)pose.timestamp;		// ros::Time is a secs/nsecs signed 32-bit ints
				ros_msg.pose.pose.position.x = pose.x;
				ros_msg.pose.pose.position.y = pose.y;
				ros_msg.pose.pose.position.z = pose.z;

				// other members of ROS message which can be set:
				// ros_msp.pose.pose.orientation.x
				// ros_msp.pose.pose.orientation.y
				// ros_msp.pose.pose.orientation.z
				// ros_msp.pose.pose.orientation.w

				// create covariance matrix, slower but more convenient with GSL
				gsl_matrix *cov = gsl_matrix_calloc(6, 6); // creates matrix and sets all elements to zero
				gsl_matrix_set_identity(cov);
				// TODO calculate correct covariance matrix for the measurements

				double *matrixArr;
				gsl_matrix_view_array( matrixArr, 6, 6 );
				for( int i = 0; i < 36; ++i )
				{
					ros_msg.covariance[i] = static_cast<float>(matrixArr[i]);
				}

				position_pub.publish(ros_msg);
#endif
				// ROS Output End*************************************************

				// FOR VICON ONLY USE
				//
				//			// publish attitude message
				//			attMsg.usec = static_cast<uint64_t>(dt * 10000000.0);
				//			attMsg.roll = pose.roll;
				//			attMsg.pitch = pose.pitch;
				//			attMsg.yaw = pose.yaw;
				//			attMsg.rollspeed = (droll - dyaw * sin(pose.pitch)) / dt;
				//			attMsg.pitchspeed = (dpitch * cos(pose.roll) +
				//								 dyaw * cos(pose.pitch) * sin(pose.roll)) / dt;
				//			attMsg.yawspeed = (- dpitch * sin(pose.roll) +
				//							   dyaw * cos(pose.pitch) * cos(pose.roll)) / dt;
				//			mavlink_msg_attitude_encode(systemid, componentid, &msg, &attMsg);
				//			mavlink_message_t_publish(lcm, "MAVLINK", &msg);
				//
				//			// publish local position message
				//			posMsg.usec = attMsg.usec;
				//			posMsg.x = pose.x;
				//			posMsg.y = pose.y;
				//			posMsg.z = pose.z;
				//			posMsg.vx = (pose.x - lastPose.x) / dt;
				//			posMsg.vy = (pose.y - lastPose.y) / dt;
				//			posMsg.vz = (pose.z - lastPose.z) / dt;
				//			mavlink_msg_local_position_encode(systemid, componentid,
				//											  &msg, &posMsg);
				//			mavlink_message_t_publish(lcm, "MAVLINK", &msg);

				lastPose = pose;
			}

			lastTime = PxVicon::getTime();
		}
		else
		{
			usleep(1000);
		}
	}

	fprintf(stderr, "\nShutting down...\n");

	vicon.disconnect();

	lcm_destroy(lcm);
	return 0;
}

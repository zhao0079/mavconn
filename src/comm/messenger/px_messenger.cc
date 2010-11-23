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
 *   @brief MAVLINK Messenger
 *
 *   Send Messages from command line or script
 *
 *   @author Laurens Mackay
 *
 */
#include <iostream>
/* BOOST includes */
#include <boost/program_options.hpp>
#include "mavconn.h"

lcm_t * lcm; ///< LCM interprocess reference

namespace config = boost::program_options;

void send_reset(void) {
	char param[15] = "SYS_IMU_RESET";//{'I','M','U','_','R','E','S','E','T'};
	mavlink_message_t msg;
	mavlink_param_set_t p;
	p.param_value = 1;
	p.target_system = (uint8_t) getSystemID();
	p.target_component = (uint8_t) MAV_COMP_ID_IMU;
	strncpy((char*) p.param_id, param, 15);
	p.param_id[14] = 0;
	mavlink_msg_param_set_encode(getSystemID(), PX_COMP_ID_MESSENGER, &msg, &p);

	mavlink_message_t_publish(lcm, "MAVLINK", &msg);
}

int main(int argc, char* argv[]) {
	// Handling Program options

	int reset;
	config::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")(
			"reset,r",
			config::value<int>(&reset)->default_value(0), "RESET IMU")
	//									("serial,s", config::value<uint64_t>(&cam_id)->default_value(0), "Serial # of the camera to select")
	//									("capture-index,c", config::value<int>(&capture_index)->default_value(0), "Capture index. 0 to auto-select")
	//									("exposure,e", config::value<uint32_t>(&exposure)->default_value(2000), "Exposure in microseconds")
	//									("gain,g", config::value<uint32_t>(&gain)->default_value(120), "Gain in FIXME")
	//									("trigger,t", config::bool_switch(&trigger)->default_value(false), "Enable hardware trigger (Firefly MV: INPUT: GPIO0, OUTPUT: GPIO2)")
	//									("silent,s", config::bool_switch(&silent)->default_value(false), "Surpress outputs")
	//									("verbose,v", config::bool_switch(&verbose)->default_value(false), "Verbose output")
	//									("debug,d", config::bool_switch(&debug)->default_value(false), "Debug output")
	;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	lcm = lcm_create("udpm://");
	if (!lcm) {
		printf("\nCouldn't start LCM link, aborting\n");
		return 1;
	}
	if (vm.count("reset") || vm.count("r")) {

		if (reset == 1) {
			send_reset();
			printf("\nRESET IMU\n");
		} else {

			printf("\nNO reset\n");
		}
		return 1;
	}

}

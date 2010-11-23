/*======================================================================

PIXHAWK mcvlib - The Micro Computer Vision Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  @author Lorenz Meier, <lm@student.ethz.ch>
  @author Fabian Landau, <mavteam@student.ethz.ch>
  @author Benjamin Knecht, <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):

Todo:


(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mcvlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mcvlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mcvlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

#include <iostream>
#include <cstdlib>
//#include <unistd.h>

#include <boost/program_options.hpp>

namespace config = boost::program_options;

int main(int argc, char* argv[]) {

// Use the predefined headers to distinguis between different platforms
// e.g. enable the IPC code parts only on Linux systems
	
	// Program options - can be defined either in config file
	// or via command line
	// Program options
    config::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("heartbeat,h", config::bool_switch()->default_value(false), "send heartbeat signal to parent process")
    ;
	
	config::variables_map vm;
    config::store(config::parse_command_line(argc, argv, desc), vm);
    config::notify(vm);
	
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

  return 0;
}

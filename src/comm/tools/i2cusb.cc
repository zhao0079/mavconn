/*======================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

Original Authors:

Contributing Authors (in alphabetical order):

Todo:

(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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

========================================================================*/

/**
 * @file
 *   @brief I2C to USB interface driver for the module from devantech
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
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
#include <SerialStream.h>
#include <SerialPort.h>



// Devantech commands
#define I2CD_CMD	0x53		// direct I2C control command
#define I2C_CMD		0x55		// registered I2C control command
#define CM01_CMD	0x5a		// CM01 command

enum cmds { nop=0, VERSION, NEW_ADDRESS, BATTERY,
			SCAN1, SCAN2, SCAN3, SCAN4, SCAN6, SCAN8, SCAN12, SCAN16,
};
int Addr=0;

enum mode
{
	I2C = 0,
	MOTOR = 1
};

namespace config = boost::program_options;
using std::string;
using namespace std;
using namespace LibSerial;

SerialStream serial_port; ///< The reference to the hardware serial port
// Settings
string port; ///< The serial port name, e.g. /dev/ttyUSB0
bool silent; ///< Wether console output should be enabled
bool verbose; ///< Enable verbose output
bool debug; ///< Enable debug functions and output


// MOTOR STATE
int currMotorstate;
bool consoleMode;


// Prepare log file
std::ofstream logfile("i2c_log.txt");

static bool sendToI2C(const char* bytes, unsigned int numNytes)
{
    return true;
}

static bool getFromI2C(const char* bytes, unsigned int maxBytes)
{
    return true;
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
							("port,p", config::value<string>(&port)->default_value("/dev/ttyUSB0"), "serial port, e.g. /dev/ttyUSB0")
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
	// Handling program options done

	// SETUP SERIAL PORT

	// Settings for Devantech module:

	// 19200 8N2

	// 19200 Baud
	// No parity
	// No hardware flow control
	// No DTR
	// No RTS
	// Char size 8
	// Two stop bits

	SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::BAUD_19200;
	SerialStreamBuf::CharSizeEnum dataBits = SerialStreamBuf::CHAR_SIZE_8;
	SerialStreamBuf::ParityEnum parity =  SerialStreamBuf::PARITY_NONE;
	int stopBits = 2;
	SerialStreamBuf::FlowControlEnum flowControl =  SerialStreamBuf::FLOW_CONTROL_NONE;

	string serialPortName = port;

	// Exit if opening port failed
	// Open the serial port.
	if (!silent) std::cout << "Trying to connect to " << serialPortName << ".. " << std::endl;

	int noErrors = 0;
	serial_port.Open(serialPortName);
	if (!serial_port.good())
	{
		if (!silent) std::cout << "First attempt failed, waiting for port..";
	}
	else
	{
		if (!silent) std::cout << std::endl << "Connected to " << serialPortName << std::endl;
	}

	while (!serial_port.good())
	{

		usleep(100000); ///< Sleep for 100 ms / 0.001 seconds
		serial_port.Open(serialPortName); ///< Try again
		if (!silent) std::cout << "..";
	}

	// Set the baud rate of the serial port.
	serial_port.SetBaudRate(baudRate) ;
	if (!serial_port.good())
	{
		if (!silent) std::cerr << "Error: Could not set the baud rate." << std::endl ;
		noErrors++;
	}

	// Set the number of data bits.
	serial_port.SetCharSize(dataBits) ;
	if (!serial_port.good())
	{
		if (!silent) std::cerr << "Error: Could not set the character size." << std::endl ;
		noErrors++;
	}

	// Disable parity.
	serial_port.SetParity(parity) ;
	if (! serial_port.good())
	{
		if (!silent) std::cerr << "Error: Could not disable the parity." << std::endl ;
		noErrors++;
	}

	// Set the number of stop bits.
	serial_port.SetNumOfStopBits(stopBits) ;
	if (!serial_port.good())
	{
		if (!silent) std::cerr << "Error: Could not set the number of stop bits." << std::endl;
		noErrors++;
	}

	// Turn off hardware flow control.
	serial_port.SetFlowControl(flowControl) ;
	if (!serial_port.good())
	{
		if (!silent) std::cerr << "Error: Could not use hardware flow control." << std::endl ;
		noErrors++;
	}

	// Do not skip whitespaces (would change binary data)
	serial_port.unsetf( std::ios_base::skipws );

	if(noErrors > 0) exit(noErrors);

	//	/// Wait for data on serial port
	//	while(serial_port.rdbuf()->in_avail() == 0)
	//	{
	//		IPC_listen(0);
	//		usleep(100) ;
	//	}


	// MAIN LOOP
	// read data from serial port and IPC and forward packets

//	static int x=0;
	string s;
	//DWORD n;
//	static int search_addr=0xe0;
//	char cmd[10];
//	static int idx=1, dir=0, scan=0, old_idx=1, cm02_ver;
	char sbuf[100];

	string shellSign = "[I2C]$: ";


	// Read out version of interface dongle
	sbuf[0] = CM01_CMD;	// Send command to dongle, not over I2C
	sbuf[1] = VERSION;  // Ask for version
	sbuf[2] = 0x00;     // Random
	sbuf[3] = 0x00;     // Unused
    serial_port.write(sbuf, 4);
    // Sleep 10 ms and wait for response
    usleep(1000*10);
    char ret = serial_port.get(); // Get one byte
	cout << "Devantech USB to I2C Adapter detected, version " << static_cast<unsigned int>(ret) << endl;

	while (true)
	{
		// Read from command line
		string command;
		cin >> hex >> command;

		if (command == "m")
		{
			cout << "Entered Motor commands mode. Use the up-arrow to go increase and the down arrow to decrease the thrust" << endl;
			shellSign = "[motor]$: ";
			cout << shellSign;
		}
		else if (command == "c")
		{
			cout << "Entered console mode. Please use hex format (without spaces) to input the bytes to send over I2C" << endl;
			shellSign = "[I2C]$: ";
			cout << shellSign;
		}

		// Emit next command to be sent

		if(!serial_port.good()) {
			cerr << "Please check your serial port" << endl;
			break; // Quit loop because of error
		}
//		else if(Addr==0) {			// find srf08
//			sbuf[0] = I2C_CMD;			// send sonar read command
//			sbuf[1] = search_addr+1;
//			sbuf[2] = 0x00;
//			sbuf[3] = 0x01;
//		    WriteFile(hCom, &sbuf, 4, &n, NULL);
//			ReadFile(hCom, &sbuf, 1, &n, NULL);
//			if(sbuf[0]<20) {
//				Addr = search_addr;
//				s.Format("0x%02X ", Addr);
//				SetDlgItemText( IDC_SRF08_ADDR, s );
//				SetDlgItemText( IDC_MSG, "SRF Found" );
//			}
//			else {
//				search_addr += 2;
//				if(search_addr>0xfe) search_addr = 0xe0;
//				s.Format("0x%02X", search_addr);
//				SetDlgItemText( IDC_SRF08_ADDR, s );
//				SetDlgItemText( IDC_MSG, "Searching" );
//			}
//			Invalidate(TRUE);
//		}
//		else {
//			sbuf[0] = I2C_CMD;			// send sonar read command
//			sbuf[1] = Addr+1;
//			sbuf[2] = 0x00;
//			sbuf[3] = 0x04;
//		    WriteFile(hCom, &sbuf, 4, &n, NULL);
//
//			ReadFile(hCom, &sbuf, 4, &n, NULL);
//			s.Format("%i", sbuf[0]);
//			SetDlgItemText( IDC_SRF08_VER, s );
//			if(sbuf[0]>20) Addr=0;
//
//			s.Format("%i ", sbuf[1]);
//			SetDlgItemText( IDC_SRF08_LIGHT, s );
//
//			n = sbuf[2]<<8;
//			n |= sbuf[3];
//			s.Format("%i", n);
//			SetDlgItemText( IDC_SRF08_US, s );
//			s.Format("%i", n/58);
//			SetDlgItemText( IDC_SRF08_CM, s );
//			s.Format("%i", n/148);
//			SetDlgItemText( IDC_SRF08_INCH, s );
//
//			sbuf[0] = I2C_CMD;			// send gain limit
//			sbuf[1] = Addr;
//			sbuf[2] = 0x01;
//			sbuf[3] = 0x01;
//			sbuf[4] = 20;
//		    WriteFile(hCom, &sbuf, 5, &n, NULL);
//			ReadFile(hCom, &sbuf, 1, &n, NULL);
//
//			sbuf[0] = I2C_CMD;			// send sonar rangeing (uS) command
//			sbuf[1] = Addr;
//			sbuf[2] = 0x00;
//			sbuf[3] = 0x01;
//			sbuf[4] = 0x52;
//		    WriteFile(hCom, &sbuf, 5, &n, NULL);
//			ReadFile(hCom, &sbuf, 1, &n, NULL);
//		}

		// Read from serial port

		int bytes = serial_port.rdbuf()->in_avail();

		// Break from input shell to next line if something was received
		if (bytes > 0) cout << endl;

		while(bytes > 0)
		{
			char next_byte;
			serial_port.get(next_byte);
			std::cerr << std::hex << static_cast<int>(next_byte) << " ";
			//uint8_t cp = next_byte;

			//serial_port.get(&sbuf, 5);

			bytes = serial_port.rdbuf()->in_avail();
		}
		// Sleep 1 ms (1000 us)
		usleep(1000);
	}
	logfile.close();
	std::cerr << std::endl;
	exit(0);
}


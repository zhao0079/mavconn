/*======================================================================

 PIXHAWK mavlib - The Micro Computer Vision Platform Library
 Please see our website at <http://pixhawk.ethz.ch>


 Original Authors:
 Dominik Honegger, <mail@example.com>
 Lorenz Meier, <lm@student.ethz.ch>
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
#include <unistd.h>
#include <cmath>
#include <cv.h>
#include <highgui.h>
#include <SerialStream.h>
#include <SerialPort.h>
#include <string.h>
#include <inttypes.h>
// MAVLINK includes
#include <generated/messages.h>
// NMEA lib includes
#include <nmea/nmea.h>
// IPC includes
#include <ipcHelperFunctions.h>
#include <cmuipc/ipc.h>
#include <ipc_core.sig>

static void responseHandler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
    CommMessage_t message;
    IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &message, COMM_MAX_DATA_LEN);
    // Process packets for use inside this process
    switch (message.msgid)
    {
        case MESSAGE_ID_HEARTBEAT:
            std::cerr << "Heartbeat received from system " << std::dec << static_cast<int>(message.acid) << std::endl;
            break;
        case MESSAGE_ID_BOOT:
            std::cerr << "Boot detected, software v." << std::endl;// << std::dec << static_cast<int>(messageBootGetFromBuffer_version(message.payload)) << std::endl;
            break;
        default:
            std::cerr << "Unable to decode message from system " << std::dec << static_cast<int>(message.acid) << std::endl;
            break;
    }

    IPC_freeByteArray(callData);
}

int main(int argc, char* argv[])
{

	// TODO Write test code here:

	////////////////
	// Parsing of packages:

    const char *buff[] = {
        "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
        "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
        "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
        "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
        "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
        "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
        "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
    };

    int nmeai;
    nmeaINFO info;
    nmeaPARSER parser;

    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    for(nmeai = 0; nmeai < 6; ++nmeai)
        nmea_parse(&parser, buff[nmeai], (int)strlen(buff[nmeai]), &info);

    nmea_parser_destroy(&parser);

    ////////////////////
    // Calculations in coordinate space:

    const char *buf2[] = {
        "$GPRMC,213916.199,A,4221.0377,N,07102.9778,W,0.00,,010207,,,A*6A\r\n",
        "$GPRMC,213917.199,A,4221.0510,N,07102.9549,W,0.23,175.43,010207,,,A*77\r\n",
        "$GPRMC,213925.000,A,4221.1129,N,07102.9146,W,0.00,,010207,,,A*68\r\n",
        "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n"
    };

    int numpoints = 4;

    nmeaPOS pos[numpoints], pos_moved[numpoints][2];
    double dist[numpoints][2];
    double azimuth[numpoints][2], azimuth_moved[numpoints];
    int result[2];
    int it = 0;

    nmeaPARSER parser2;
    nmea_parser_init(&parser2);

    for(it = 0; it < numpoints; ++it)
    {
        int result;
        nmeaINFO info;
        nmea_zero_INFO(&info);
        result = nmea_parse(&parser2, buf2[it], (int)strlen(buff[it]), &info);
        nmea_info2pos(&info, &pos[it]);
    }

    nmea_parser_destroy(&parser2);

    for(it = 0; it < numpoints; ++it)
    {
        dist[it][0] = nmea_distance(&pos[0], &pos[it]);
        dist[it][1] = nmea_distance_ellipsoid(
            &pos[0], &pos[it], &azimuth[it][0], &azimuth[it][1]
            );
    }

    for(it = 0; it < numpoints; ++it)
    {
        result[0] = nmea_move_horz(&pos[0], &pos_moved[it][0], azimuth[it][0], dist[it][0]);
        result[1] = nmea_move_horz_ellipsoid(
            &pos[0], &pos_moved[it][1], azimuth[it][0], dist[it][0], &azimuth_moved[it]
            );
    }


    /* Output of results */

    printf("Coordinate points:\n");

    for(it = 0; it < numpoints; ++it)
    {
        printf(
            "P%d in radians: lat:%9.6lf lon:%9.6lf  \tin degree: lat:%+010.6lf° lon:%+011.6lf°\n",
            it, pos[it].lat, pos[it].lon, nmea_radian2degree(pos[it].lat), nmea_radian2degree(pos[it].lon)
            );
    }



    printf("\nCalculation results:\n");

    for(it = 0; it < numpoints; ++it)
    {
        printf("\n");
        printf("Distance P0 to P%d\ton spheroid:  %14.3lf m\n", it, dist[it][0]);
        printf("Distance P0 to P%d\ton ellipsoid: %14.3lf m\n", it, dist[it][1]);
        printf("Azimuth  P0 to P%d\tat start: %8.3lf°\tat end: %8.3lf°\n", it, nmea_radian2degree(azimuth[it][0]), nmea_radian2degree(azimuth[it][1]));
        printf("Move     P0 to P%d\t         \tAzimuth at end: %8.3lf°\n", it, nmea_radian2degree(azimuth_moved[it]));
        printf("Move     P0 to P%d\ton spheroid:  %3s lat:%+010.6lf° lon:%+011.6lf°\n", it, result[0] == 1 ? "OK" : "nOK", nmea_radian2degree(pos_moved[it][0].lat), nmea_radian2degree(pos_moved[it][0].lon));
        printf("Move     P0 to P%d\ton ellipsoid: %3s lat:%+010.6lf° lon:%+011.6lf°\n", it, result[0] == 1 ? "OK" : "nOK", nmea_radian2degree(pos_moved[it][1].lat), nmea_radian2degree(pos_moved[it][1].lon));
        printf("Move     P0 to P%d\toriginal:         lat:%+010.6lf° lon:%+011.6lf°\n", it, nmea_radian2degree(pos[it].lat), nmea_radian2degree(pos[it].lon));
    }

    // Initialize Serial Port

	using namespace LibSerial;
	//int noSerialPorts         = 2;
	SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::BAUD_115200;
	SerialStreamBuf::CharSizeEnum dataBits = SerialStreamBuf::CHAR_SIZE_8;
	SerialStreamBuf::ParityEnum parity =  SerialStreamBuf::PARITY_NONE;
	int stopBits = 1;
	SerialStreamBuf::FlowControlEnum flowControl =  SerialStreamBuf::FLOW_CONTROL_NONE;

	std::string serialPortName    = "/dev/ttyUSB0";


	// Connect to IPC central
	// this connects to the inter process communication.
	char* moduleName = const_cast<char*>("gpsserial");
	// TODO Check

	IPC_connect(moduleName);
	// Define message you want to send (this message is defined in ipc/ipc_core.sig)
	// We're using MAVLINK here to transport our GPS data
	IPC_defineFormat(COMM_NAME, COMM_FORMAT);
	IPC_defineMsg(COMMMSG, COMM_MAX_DATA_LEN, COMMMSG_FORMAT);
	IPC_subscribeData(COMMMSG, responseHandler, moduleName);

	// Exit if opening port failed
    // Open the serial port.

	int noErrors = 0;
    SerialStream serial_port;
    serial_port.Open(serialPortName);
    if ( ! serial_port.good() )
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
		<< "Error: Could not open serial port."
		<< std::endl ;
        noErrors++;
    }

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(baudRate) ;
    if (!serial_port.good())
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        noErrors++;
    }

    // Set the number of data bits.
    serial_port.SetCharSize(dataBits) ;
    if (!serial_port.good())
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        noErrors++;
    }

    // Disable parity.
    serial_port.SetParity(parity) ;
    if (! serial_port.good())
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        noErrors++;
    }

    // Set the number of stop bits.
    serial_port.SetNumOfStopBits(stopBits) ;
    if (!serial_port.good())
    {
        std::cerr << "Error: Could not set the number of stop bits." << std::endl;
        noErrors++;
    }

    // Turn off hardware flow control.
    serial_port.SetFlowControl(flowControl) ;
    if (!serial_port.good())
    {
        std::cerr << "Error: Could not use hardware flow control." << std::endl ;
        noErrors++;
    }

    // Do not skip whitespaces (would change binary data)
    serial_port.unsetf( std::ios_base::skipws );

	if(noErrors > 0) exit(noErrors);

    /// Wait for data on serial port
    while(serial_port.rdbuf()->in_avail() == 0)
    {
        usleep(100) ;
    }

    // Once data is available, keep reading data
	while(true) {
		int bytes = serial_port.rdbuf()->in_avail();
		while(bytes > 0)
		{
			char next_byte;
			serial_port.get(next_byte);
			std::cerr << std::hex << static_cast<unsigned short>(next_byte) << " ";

			bool msgReceived = false;

			// TODO Parse the next byte

			// If a message could be decoded, handle it
			if(msgReceived) {

				// Pack MAVLINK GPS message
				CommMessage_t message;
				//messagePackGPS(systemID, lat, lon, time, etc)

                // Send out packets to IPC
                IPC_queryNotify(COMMMSG, COMM_MAX_DATA_LEN, &message, responseHandler, moduleName);

                // listen to receive IPC messages
                IPC_listen(0);
			}
		}
		usleep(100); ///< Sleep for 1 ms, then check again for some bytes
	}
    std::cerr << std::endl;
    // disconnect module properly from IPC
    IPC_disconnect();
    exit(0);
}

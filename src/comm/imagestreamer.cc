/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

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

======================================================================*/

/**
 * @file
 *   @brief FIXME Needs brief description
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <highgui.h>

#include "Core.h"
#include <ipcextension/imageClientModule.h>
#include <ipcHelperFunctions.h>
#include <sys/time.h>
#include <iostream>
#include "generated/messages.h"

using namespace pixhawk;
using namespace std;

//static int receiveImgs = 0;

static void getImage(IplImage* image)
{
	cout << "RECEIVED IMAGE FROM IPC" << endl;

	// Testing / debugging transmission
	static int imgPacketIndex = 0;

	static const int imgWidth = 640;
	static const int imgHeight = 480;

	static const int imgDepth = 8;
	static const int transmissionDepth = 8;
	static const int imgChan = 1;

	static const int imgBytes = (imgWidth * imgHeight * imgChan * transmissionDepth) / 8;
	static int imgDataIndex = imgBytes;
	static unsigned char imgData[imgBytes];

	static uint8_t movingIndex = 0;

	int k;
	for (k = 0; k < imgBytes; k++)
	{
		if (k > 50 * imgWidth + movingIndex && k < 200 * imgWidth + movingIndex)
		{
			imgData[k] = 0x55;
		}
		else
		{
			imgData[k] = 0xFF;
		}
	}

	int imageCounter = 0;

	while (1)
	{
		//if (!sendImage(frame)) printf("Camera Interface: Cannot send frame!\n");
			cout << "Emitting heartbeat" << endl;
			// SEND HEARTBEAT

			CommMessage_t msg;

			// Pack message and get size of encoded byte string
			messagePackHeartbeat(100, &msg, MAV_GENERIC);
			// Allocate buffer with packet data
			sendCommMsg(&msg, "simlink");
			//usleep(1000 * 1000);

			// SEND IMAGE
			// Reset image data index if necessary
			// If all data has been sent, image data index and imgBytes - 1 match
			if (imgDataIndex >= imgBytes - 1)
			{



				// Create moving white bar
				int k;
				for (k = 0; k < imgBytes; k++)
				{
					if (k > 50 * imgWidth + movingIndex * imgWidth && k < 200 * imgWidth + movingIndex * imgWidth)
					{
						imgData[k] = 0x55;
					}
					else
					{
						imgData[k] = 0xFF;
					}
				}
				movingIndex++;




				imgDataIndex = 0;
				imgPacketIndex = getNextRid();

				imageCounter++;
				//usleep(50 * 1000);


				std::cout << "Starting image " << imageCounter << " with ressource id " << imgPacketIndex << std::endl;

				int systemid = 100;
				// Image fragment
				messageStartImage(systemid, &msg, imgPacketIndex, imgWidth, imgHeight, imgDepth, imgChan);
				// Allocate buffer with packet data
				//bufferLength = messageToSendBuffer(send_buf.data(), &msg);
				// Send data
				sendCommMsg(&msg, "simlink");
			}

			// Image fragment
			int bytesToSend = imgBytes - imgDataIndex;
			if (bytesToSend > 255 - 7)
			{
				bytesToSend = 255 - 7;
			}

			messagePackBytestream(100, &msg, imgPacketIndex, imgDataIndex, bytesToSend, imgData+imgDataIndex);
			// Allocate buffer with packet data
			imgDataIndex += bytesToSend;
			//qDebug() << "Sending image payload data (" << bytesToSend << "bytes) with size " << messageSize << "," << (imgBytes - imgDataIndex) << " bytes remain";
			sendCommMsg(&msg, "simlink");
			// Send data

		}

		            IPC_listen(0);
		            usleep(100);




    //char str[256];

    //sprintf(str, "%s%s%i-%i%s", "image_", zeros.c_str() ,receiveImgs, static_cast<int>(currTime), ".bmp");
    //sprintf(str, "%s%05d-%010d%s", "image_",receiveImgs, static_cast<int>(currTime), ".bmp");
	//sprintf(str, "%s%05d%s", "image_",receiveImgs, ".bmp");
    //receiveImgs++;
    //cvSaveImage(std::string(Core::getInstance().getCapturePathString() + std::string(str)).c_str(), image);
}


int main( int argc, char* argv[] )
{
    Core core("simlink");

    // start client with unique module name and function pointer to receive an image
    ImageClient imageClient;
    if (!imageClient.connect("simlink"))
        printf("Couldn't connect to IPC\n");
    imageClient.start(&getImage);

    // pollServer to receive images. After this function call the function specified by the function pointer may be called.
    while (imageClient.pollForMessages(50)){}

    imageClient.disconnect();
    return 0;
}

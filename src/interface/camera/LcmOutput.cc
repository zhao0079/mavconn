/*======================================================================

MAVCONN mcvlib - The Micro Computer Vision Library
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  Fabian Landau
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

This file is part of the MAVCONN project

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

#include "LcmOutput.h"

#define CAPTURED_FRAME_WIDTH 640 // ((65536 - 4 - 4) / 3) // 752 // 100
#define CAPTURED_FRAME_HEIGHT 480 // 1 // 480 // 64
#define CAPTURED_FRAME_N_COLOR_CHANNELS 1

#define IMG_BYTE_SIZE CAPTURED_FRAME_WIDTH * CAPTURED_FRAME_HEIGHT * CAPTURED_FRAME_N_COLOR_CHANNELS



#include <sys/time.h> // remove this

namespace MAVCONN
{
    RegisterOutputOperation(LcmOutput)
        .setDescription("Sends the camera frames over LCM")
        .addName("lcm")
        .addName("l")
        .makeDefault()
    ;

    uint8_t sysid = 42;
    uint8_t compid = 210;

    LcmOutput::LcmOutput(Camera* camera) : OutputOperation(camera), imageServer_(sysid, compid, CAPTURED_FRAME_WIDTH, CAPTURED_FRAME_HEIGHT, IPL_DEPTH_8U, CAPTURED_FRAME_N_COLOR_CHANNELS)
    {
        this->bFirstFrame_ = true;
        this->image_ = cvCreateImage(cvSize(CAPTURED_FRAME_WIDTH, CAPTURED_FRAME_HEIGHT), IPL_DEPTH_8U, 1);

        this->lcm_ = lcm_create(NULL);

        if (!this->lcm_)
            std::cout << "Couldn't connect to LCM" << std::endl;
    }

    LcmOutput::~LcmOutput()
    {
        cvReleaseImage(&this->image_);
    }

    void LcmOutput::processImage(IplImage* original)
    {
        if (!this->lcm_)
        {
            delete this;
            return;
        }

        IplImage* image = original;
        IplImage* temp = 0;

        if (image->nChannels != CAPTURED_FRAME_N_COLOR_CHANNELS)
        {
            if (this->bFirstFrame_)
                std::cout << "Warning: The input image has not the desired number of color channels (" << image->nChannels << " instead of " << CAPTURED_FRAME_N_COLOR_CHANNELS << ")" << std::endl;

            temp = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, CAPTURED_FRAME_N_COLOR_CHANNELS);
            cvCvtColor(image, temp, CV_RGB2GRAY);
            image = temp;
        }

        if (image->width != CAPTURED_FRAME_WIDTH || image->height != CAPTURED_FRAME_HEIGHT)
        {
            if (this->bFirstFrame_)
                std::cout << "Warning: The input image has not the desired size of " << CAPTURED_FRAME_WIDTH << "x" << CAPTURED_FRAME_HEIGHT << " (size: " << image->width << "x" << image->height << ")" << std::endl;

            cvResize(image, this->image_, CV_INTER_LINEAR);
            image = this->image_;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t timestamp = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

        this->imageServer_.sharedMemWriteImage(image, 0, 0, timestamp, 0, 0, 0, this->lcm_);

        this->bFirstFrame_ = false;

        if (temp)
            cvReleaseImage(&temp);
    }
}

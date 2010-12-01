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

#include "LcmInput.h"
#include <sys/time.h> // remove this

namespace MAVCONN
{
    RegisterInputOperation(LcmInput)
        .setDescription("Retrieves frames over LCM")
        .addName("lcm")
        .addName("l")
    ;

    LcmInput::LcmInput(Camera* camera) : InputOperation(camera)
    {
        this->getCamera()->setInputModeName("LCM");

        this->lcm_ = 0;
        this->subscription_ = 0;
        this->interval_ = 0;
    }

    LcmInput::~LcmInput()
    {
        if (this->lcm_)
        {
            if (this->subscription_)
                mavlink_message_t_unsubscribe(this->lcm_, this->subscription_);

            lcm_destroy(this->lcm_);
        }
    }

    void LcmInput::startPlayback()
    {
        this->lcm_ = lcm_create(NULL);

        if (!this->lcm_)
        {
            std::cout << "Couldn't connect to LCM" << std::endl;
            delete this;
            return;
        }

        this->subscription_ = mavlink_message_t_subscribe(this->lcm_, "IMAGES", &LcmInput::image_handler, this);
    }

    /* static */ void LcmInput::image_handler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData)
    {
        LcmInput* _this = static_cast<LcmInput*>(userData);

        static IplImage* image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

        mavlink_image_available_t img;
        mavlink_msg_image_available_decode(msg, &img);
/*
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t timestamp = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
        std::cout << "delay: " << (timestamp - img.timestamp) << std::endl;
*/
        // Check if image could be retrieved
        if (_this->imageClient_.sharedMemCopyImage(msg, image))
            _this->getCamera()->processImage(image);
        else
            std::cout << "ERROR: Retrieving image from shared memory failed!" << std::endl;

        _this->interval_ = _this->timeoutTimer_.getMilliseconds();
        _this->timeoutTimer_.reset();
    }

    void LcmInput::tick()
    {
        int fileno = lcm_get_fileno(this->lcm_);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fileno, &readfds);

        select(fileno + 1, &readfds, NULL, NULL, &tv);

        if (FD_ISSET(fileno, &readfds))
            lcm_handle(this->lcm_);

        if (this->interval_ && this->timeoutTimer_.getMilliseconds() > this->interval_ * 2.5)
            this->getCamera()->stopPlayback(this);
    }
}

/*======================================================================

PIXHAWK mcvlib - The Micro Computer Vision Library
Please see our website at <http://pixhawk.ethz.ch>

Original Authors:
  Fabian Landau
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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

#include "DisplayOutput.h"
#include "FrameOutput.h"
#include "SequenceOutput.h"
#include "VideoOutput.h"
#include <highgui.h>


#define CAPTURED_FRAME_WIDTH 640 // ((65536 - 4 - 4) / 3) // 752 // 100
#define CAPTURED_FRAME_HEIGHT 480 // 1 // 480 // 64
#define CAPTURED_FRAME_N_COLOR_CHANNELS 1

#define IMG_BYTE_SIZE CAPTURED_FRAME_WIDTH * CAPTURED_FRAME_HEIGHT * CAPTURED_FRAME_N_COLOR_CHANNELS


namespace pixhawk
{
    RegisterOutputOperation(DisplayOutput)
        .setDescription("Opens a window to display and control the camera")
        .addName("display")
        .addName("d")
        .addParameter("overlay", &DisplayOutput::setDrawOverlay, "Shows the information overlay", true)
        .makeDefault()
    ;

    DisplayOutput::DisplayOutput(Camera* camera) : OutputOperation(camera)
    {
        this->bStartedProcessing_ = false;
        this->bDrawOverlay_ = true;
        this->bShowHelp_ = false;
        this->bReceivedHeartbeat_ = false;

        this->windowname_ = "Display";
        cvNamedWindow(this->windowname_.c_str());

        cvInitFont(&this->font_, CV_FONT_HERSHEY_PLAIN, 0.9, 1.0, 0, 1);
        cvInitFont(&this->fontLarge_, CV_FONT_HERSHEY_PLAIN, 2.0, 2.0, 0, 1);

        this->input_ = 0;
        this->output_ = 0;
        this->fallback_ = 0;

        this->createFallbackImage();
        cvShowImage(this->windowname_.c_str(), this->fallback_);

        this->framecount_ = 0;
        this->fps_ = 0;
        this->oldtpf_ = (unsigned long)-1;
        this->oldframes_ = 0;
        this->seconds_ = 0;

        this->sequenceOutput_ = 0;
        this->videoOutput_ = 0;

        this->lcm_ = lcm_create(NULL);
        if (this->lcm_)
            this->subscription_ = mavlink_message_t_subscribe(this->lcm_, "MAVLINK", &DisplayOutput::mavlinkHandler, this);
    }

    DisplayOutput::~DisplayOutput()
    {
        if (this->fallback_)
            cvReleaseImage(&this->fallback_);

        if (this->input_)
            cvReleaseImage(&this->input_);

        if (this->sequenceOutput_)
            this->sequenceOutput_->requestDestruction();

        if (this->videoOutput_)
            this->videoOutput_->requestDestruction();

        cvDestroyWindow(this->windowname_.c_str());

        if (this->lcm_)
        {
            if (this->subscription_)
                mavlink_message_t_unsubscribe(this->lcm_, this->subscription_);

            lcm_destroy(this->lcm_);
        }
    }

    void DisplayOutput::startProcessing()
    {
        this->bStartedProcessing_ = true;
        this->framecount_ = 0;
        this->fps_ = 0;
        this->oldtpf_ = (unsigned long)-1;
        this->oldframes_ = 0;
        this->seconds_ = 0;
        this->timer_.reset();
        this->fpstimer_.reset();
        this->statusTimer_.reset();
        this->status_ = "";
    }

    void DisplayOutput::stopProcessing()
    {
        this->bStartedProcessing_ = false;
        cvShowImage(this->windowname_.c_str(), this->fallback_);

        if (this->input_)
            cvReleaseImage(&this->input_);

        if (this->sequenceOutput_)
        {
            this->sequenceOutput_->requestDestruction();
            this->sequenceOutput_ = 0;
        }

        if (this->videoOutput_)
        {
            this->videoOutput_->requestDestruction();
            this->videoOutput_ = 0;
        }
    }

    void DisplayOutput::endProcessing()
    {
        if (!this->bStartedProcessing_)
            delete this;
        else if (this->bDrawOverlay_)
            this->setStatus("stoped");
    }

    void DisplayOutput::processImage(IplImage* image)
    {
        if (this->oldtpf_ != (unsigned long)-1)
        {
            this->oldtpf_ += this->fpstimer_.getMicroseconds();
            ++this->oldframes_;
        }
        else
            this->oldtpf_ = 0;

        this->fpstimer_.reset();

        ++this->framecount_;

        if (!this->input_)
            this->input_ = cvCreateImage(cvSize(CAPTURED_FRAME_WIDTH, CAPTURED_FRAME_HEIGHT), IPL_DEPTH_8U, image->nChannels);
        cvResize(image, this->input_, CV_INTER_LINEAR);

        this->updateDisplay();
    }

    void DisplayOutput::tick()
    {
        if (this->statusTimer_.getMilliseconds() >= 1000 && this->status_ != "")
        {
            this->setStatus("");
        }

        if (this->statusTimer_.getMilliseconds() >= 2000 && (this->sequenceOutput_ || this->videoOutput_))
        {
            this->setStatus("recording");
        }

        if (this->timer_.getSeconds() != this->seconds_)
        {
            this->lcmHandle();

            this->seconds_ = this->timer_.getSeconds();

            if (this->oldtpf_ > 0)
            {
                this->fps_ = (int)(10000000.0f / this->oldtpf_ * this->oldframes_ + 0.5);
                this->fps_ /= 10;
                this->oldtpf_ = 0;
                this->oldframes_ = 0;
            }

            if (this->bDrawOverlay_)
                this->drawOverlay();
        }

        unsigned char key = cvWaitKey(10);
        // exit
        if (key == 27)
        {
            this->getCamera()->requestExit();
        }
        // show/hide overlay
        else if (key == 10)
        {
            this->bDrawOverlay_ = !this->bDrawOverlay_;
            this->updateDisplay();
        }
        // show/hide help
        else if (key == 190)
        {
            this->bShowHelp_ = !this->bShowHelp_;
            if (this->bDrawOverlay_)
                this->drawOverlay();
        }
        // pause/unpause
        else if (key == ' ')
        {
            if (!this->getCamera()->isStoped())
            {
                this->getCamera()->togglePause();
                if (this->getCamera()->isPaused())
                    this->setStatus("paused");
                else
                    this->setStatus("unpaused");
            }
            else
            {
                this->getCamera()->requestRestart();
                this->setStatus("restarted");
            }
        }
        // capture frame
        else if (key == 'f')
        {
            FrameOutput* operation = new FrameOutput(this->getCamera(), true);
            if (this->getCamera()->getSnapshotMask() != "")
                operation->setFilename(this->getCamera()->getSnapshotMask());
            this->setStatus("captured");
        }
        // capture sequence
        else if (key == 's')
        {
            if (!this->sequenceOutput_)
            {
                this->sequenceOutput_ = new SequenceOutput(this->getCamera(), true);
                this->setStatus("recording");
            }
            else
            {
                this->sequenceOutput_->requestDestruction();
                this->sequenceOutput_ = 0;
            }
        }
        // capture sequence
        else if (key == 'v')
        {
            if (!this->videoOutput_)
            {
                this->videoOutput_ = new VideoOutput(this->getCamera(), true);
                this->setStatus("recording");
            }
            else
            {
                this->videoOutput_->requestDestruction();
                this->videoOutput_ = 0;
            }
        }
    }

    void DisplayOutput::updateDisplay()
    {
        if (this->bDrawOverlay_)
            this->drawOverlay();
        else
            cvShowImage(this->windowname_.c_str(), this->input_);
    }

    void DisplayOutput::drawOverlay()
    {
        if (!this->input_)
            return;

        this->output_ = cvCloneImage(this->input_);

        for (int y = 0; y < 20; ++y)
            for (int x = 0; x < this->output_->width; ++x)
                for (int c = 0; c < this->output_->nChannels; ++c)
                    ((unsigned char*)this->output_->imageData)[(x * this->output_->nChannels) + c + this->output_->widthStep * y] /= 1.5;
        for (int y = this->output_->height - 20; y < this->output_->height; ++y)
            for (int x = 0; x < this->output_->width; ++x)
                for (int c = 0; c < this->output_->nChannels; ++c)
                    ((unsigned char*)this->output_->imageData)[(x * this->output_->nChannels) + c + this->output_->widthStep * y] /= 1.5;

        std::string input = "Input: ";
        input += this->getCamera()->getInputModeName();
        if (this->getCamera()->getInputResourceName() != "")
            input += " [" + this->getCamera()->getInputResourceName() + "]";
        cvPutText(this->output_, input.c_str(), cvPoint(5, 15), &this->font_, CV_RGB(255, 255, 255));


        if (!this->getCamera()->isStoped())
        {
            if (!this->getCamera()->isPaused())
            {
                CvPoint points[3];
                points[0] = cvPoint(this->output_->width - 16, 4);
                points[1] = cvPoint(this->output_->width - 16, 14);
                points[2] = cvPoint(this->output_->width - 6,  9);
                cvFillConvexPoly(this->output_, points, 3, CV_RGB(230, 230, 230), CV_AA);
            }
            else
            {
                cvRectangle(this->output_, cvPoint(this->output_->width - 16, 3), cvPoint(this->output_->width - 13, 15), CV_RGB(230, 230, 230), -1, CV_AA);
                cvRectangle(this->output_, cvPoint(this->output_->width - 9, 3), cvPoint(this->output_->width - 6, 15), CV_RGB(230, 230, 230), -1, CV_AA);
            }
        }
        else
        {
            cvRectangle(this->output_, cvPoint(this->output_->width - 16, 5), cvPoint(this->output_->width - 8, 13), CV_RGB(230, 230, 230), -1, CV_AA);
        }


        cvCircle(this->output_, cvPoint(this->output_->width - 30, 9), 6, CV_RGB(255, 255, 255), 1, CV_AA);
        if (this->getCamera()->hasIPC())
            cvCircle(this->output_, cvPoint(this->output_->width - 30, 9), 4, CV_RGB(230, 230, 230), -1, CV_AA);

        cvCircle(this->output_, cvPoint(this->output_->width - 48, 9), 6, CV_RGB(255, 255, 255), 1, CV_AA);
        if (this->bReceivedHeartbeat_ && this->heartbeatTimer_.getMilliseconds() < 1000)
        {
            unsigned char color = 255 - (255.0f * this->heartbeatTimer_.getMilliseconds() / 1000);
            cvCircle(this->output_, cvPoint(this->output_->width - 48, 9), 4, cvScalar(color, color, color), -1, CV_AA);
        }

        if (this->bShowHelp_ && this->output_->height > 310 && this->output_->width > 430)
        {
            for (int y = 170; y < 310; ++y)
                for (int x = 210; x < 430; ++x)
                    for (int c = 0; c < this->output_->nChannels; ++c)
                        ((unsigned char*)this->output_->imageData)[(x * this->output_->nChannels) + c + this->output_->widthStep * y] /= 1.5;

            cvPutText(this->output_, "Capture Frame:",     cvPoint(215, 185), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Capture Sequence:",  cvPoint(215, 205), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Capture Video:",     cvPoint(215, 225), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Show/Hide Overlay:", cvPoint(215, 245), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Show/Hide Help:",    cvPoint(215, 265), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Pause/Unpause:",     cvPoint(215, 285), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "Exit:",              cvPoint(215, 305), &this->font_, CV_RGB(255, 255, 255));

            cvPutText(this->output_, "[F]",     cvPoint(368, 185), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[S]",     cvPoint(368, 205), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[V]",     cvPoint(368, 225), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[ENTER]", cvPoint(368, 245), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[F1]",    cvPoint(368, 265), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[SPACE]", cvPoint(368, 285), &this->font_, CV_RGB(255, 255, 255));
            cvPutText(this->output_, "[ESC]",   cvPoint(368, 305), &this->font_, CV_RGB(255, 255, 255));
        }


        std::string fps = "Fps: ";
        fps += convertValueToString(this->fps_);
        if (this->fps_ == (int)this->fps_)
            fps += ".0";
        cvPutText(this->output_, fps.c_str(), cvPoint(5, this->output_->height - 5), &this->font_, CV_RGB(255, 255, 255));

        std::string frames = "Frame: ";
        frames += convertValueToString(this->framecount_);
        cvPutText(this->output_, frames.c_str(), cvPoint(120, this->output_->height - 5), &this->font_, CV_RGB(255, 255, 255));

        std::string time = "Time: ";
        unsigned int seconds = this->seconds_;
        unsigned int minutes = seconds / 60;
        unsigned int hours   = minutes / 60;
        seconds %= 60;
        minutes %= 60;
        time += convertValueToString(hours);
        time += ':';
        if (minutes < 10)
            time += '0';
        time += convertValueToString(minutes);
        time += ':';
        if (seconds < 10)
            time += '0';
        time += convertValueToString(seconds);
        cvPutText(this->output_, time.c_str(), cvPoint(270, this->output_->height - 5), &this->font_, CV_RGB(255, 255, 255));


        cvPutText(this->output_, this->status_.c_str(), cvPoint(435, this->output_->height - 5), &this->font_, CV_RGB(255, 255, 255));


        cvPutText(this->output_, "Help: [F1]", cvPoint(565, this->output_->height - 5), &this->font_, CV_RGB(255, 255, 255));

        cvShowImage(this->windowname_.c_str(), this->output_);
        cvReleaseImage(&this->output_);
    }

    void DisplayOutput::createFallbackImage()
    {
        this->fallback_ = cvCreateImage(cvSize(CAPTURED_FRAME_WIDTH, CAPTURED_FRAME_HEIGHT), IPL_DEPTH_8U, 1);
        cvZero(this->fallback_);

        std::string input = "Input: ";
        input += this->getCamera()->getInputModeName();
        cvPutText(this->fallback_, input.c_str(), cvPoint(5, 15), &this->font_, CV_RGB(255, 255, 255));

        cvPutText(this->fallback_, "Waiting for Input", cvPoint(178, 245), &this->fontLarge_, CV_RGB(255, 255, 255));
        cvPutText(this->fallback_, "Press [ESC] to exit", cvPoint(238, 270), &this->font_, CV_RGB(255, 255, 255));
    }

    void DisplayOutput::setStatus(const std::string& status)
    {
        if (status != this->status_)
        {
            this->status_ = status;
            if (this->bDrawOverlay_)
                this->drawOverlay();
            if (status != "")
                this->statusTimer_.reset();
        }
    }

    /* static */ void DisplayOutput::mavlinkHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData)
    {
        DisplayOutput* _this = static_cast<DisplayOutput*>(userData);

        if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
            _this->receivedHeartbeat();
    }

    void DisplayOutput::receivedHeartbeat()
    {
        this->bReceivedHeartbeat_ = true;
        this->heartbeatTimer_.reset();
    }

    void DisplayOutput::lcmHandle()
    {
        int fileno = lcm_get_fileno(this->lcm_);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fileno, &readfds);

        select(fileno + 1, &readfds, NULL, NULL, &tv);

        if (FD_ISSET(fileno, &readfds))
            lcm_handle(this->lcm_);
    }
}

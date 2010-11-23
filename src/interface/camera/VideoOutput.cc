/*======================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

Original Authors:
  @author Fabian Landau <mavteam@student.ethz.ch>
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

#include "VideoOutput.h"
#include <highgui.h>
#include <ctime>
#include <algorithm>

namespace pixhawk
{
    RegisterOutputOperation(VideoOutput)
        .setDescription("Writes the camera frames to a video file")
        .addName("video")
        .addName("v")
        .addParameter("file", &VideoOutput::setFilename, "The name of the video file (can contain strftime date format specifiers)", "video_%Y-%m-%d_%H-%M-%S.avi")
        .addParameter("fps", &VideoOutput::setFPS, "Max. frames per second (0 = use all frames)", 0)
        .addParameter("vfps", &VideoOutput::setVideoFPS, "FPS of the target video file (0 = use first fps parameter)", 0)
    ;

    VideoOutput::VideoOutput(Camera* camera, bool bVerbose) : OutputOperation(camera)
    {
        this->bVerbose_ = bVerbose;
        this->setFilename("video_%Y-%m-%d_%H-%M-%S.avi");
        this->setFPS(0);
        this->setVideoFPS(0);
        this->videoWriter_ = 0;
    }

    VideoOutput::~VideoOutput()
    {
        if (this->bVerbose_)
            std::cout << "Stop capturing video (" << this->filename_ << ")" << std::endl;

        if (this->videoWriter_)
            cvReleaseVideoWriter(&this->videoWriter_);
    }

    void VideoOutput::setFilename(const std::string& name)
    {
        size_t length = name.size() + 20;
        time_t rawtime;
        struct tm* timeinfo;
        char timestring[length];

        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(timestring, length, name.c_str(), timeinfo);

        this->filename_ = timestring;
    }

    void VideoOutput::setFPS(float fps)
    {
        this->fps_ = fps;
        if (this->vfps_ == 0)
            this->vfps_ = this->fps_;
    }

    void VideoOutput::setVideoFPS(float fps)
    {
        this->vfps_ = fps;
        if (this->vfps_ == 0)
        {
            if (this->fps_ != 0)
                this->vfps_ = this->fps_;
            else
                this->vfps_ = 30;
        }

        // Todo: Remove this warning if OpcenCV allows MJPG videos with smaller vfps or if we use another codec than MJPG
        if (this->vfps_ != 0 && this->vfps_ < 10)
            std::cout << "Warning: Video FPS smaller than 10 can lead to a crash" << std::endl;
    }

    void VideoOutput::processImage(IplImage* image)
    {
        if (!this->videoWriter_ || !this->fps_ || this->timer_.readyForNextFrame(this->fps_))
        {
            this->timer_.reset(this->fps_);

            if (!this->videoWriter_)
            {
                if (this->bVerbose_)
                    std::cout << "Start capturing video (" << this->filename_ << ")" << std::endl;

                // Four-C video codec:
                // Motion JPEG:  CV_FOURCC('M','J','P','G')
                // Raw YUV:      CV_FOURCC('I','Y','U','V')
                // MPEG-1:       CV_FOURCC('P','I','M','1')
                // Image series: 0
                // Codec menu:   -1
                this->videoWriter_ = cvCreateVideoWriter(this->filename_.c_str(), CV_FOURCC('M','J','P','G'), this->vfps_, cvSize(image->width, image->height)/*, image->nChannels > 1*/);

                if (!this->videoWriter_)
                {
                    std::cout << "Error: Couldn't capture video to file \"" << this->filename_ << "\"" << std::endl;

                    if (this->filename_.find_first_of('.') == std::string::npos)
                        std::cout << "       Did you forget to add a file extension? (example *.avi)" << std::endl;

                    delete this;
                    return;
                }
            }

            // check if the image is grayscale and convert it to a 3-channel image since openCV has some problems with grayscale images currently
            // Todo: check if openCV supports grayscale images without repeating them 3 times per frame
            IplImage* temp = 0;
            if (image->nChannels == 1)
            {
                temp = cvCreateImage(cvSize(image->width, image->height), image->depth, 3);
                cvCvtColor(image, temp, CV_GRAY2RGB);
                image = temp;
            }

            int success = cvWriteFrame(this->videoWriter_, image);

            if (temp)
                cvReleaseImage(&temp);

            if (!success)
            {
                std::cout << "Error: Couldn't write frame to file \"" << this->filename_ << "\"" << std::endl;
                delete this;
                return;
            }
        }
    }
}

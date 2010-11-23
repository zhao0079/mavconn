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

#include "VideoInput.h"
#include <highgui.h>
#include <boost/filesystem.hpp>
#include "SequenceOutput.h"

namespace pixhawk
{
    RegisterInputOperation(VideoInput)
        .setDescription("Plays a video file")
        .addName("video")
        .addName("v")
        .addParameter("file", &VideoInput::setFilename, "The path to the video file")
        .addParameter("fps", &VideoInput::setFPS, "Frames per second (-1 = video default)", -1)
        .addParameter("speed", &VideoInput::setSpeedFactor, "A speed factor to repeat or drop frames.\nNegative values will play the video backward", 1.0f)
        .addParameter("repeat", &VideoInput::setRepeat, "Repeat the video after it ended", false)
        .addParameter("start", &VideoInput::setStart, "The 0-based index of the frame to start playback or the position in percent", "0%")
    ;

    VideoInput::VideoInput(Camera* camera) : InputOperation(camera)
    {
        this->getCamera()->setInputModeName("Video");
        this->getCamera()->setInputResourceName("no directory specified");
        this->bRepeat_ = false;
        this->fps_ = 0;
        this->counter_ = 0;
        this->startFrame_ = 0;
        this->speedFactor_ = 1.0f;
        this->cvCapture_ = 0;
    }

    VideoInput::~VideoInput()
    {
        if (this->cvCapture_)
            cvReleaseCapture(&this->cvCapture_);
    }

    void VideoInput::setFilename(const std::string& filename)
    {
        this->filename_ = filename;
        this->getCamera()->setInputResourceName(filename);
    }

    void VideoInput::startPlayback()
    {
        if (this->filename_ != "")
        {
            this->cvCapture_ = cvCreateFileCapture(this->filename_.c_str());
            if (this->cvCapture_)
            {
                if (this->fps_ == -1)
                {
                    double property = cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FPS);
                    if (property != 0)
                        this->fps_ = (int)property;
                    else
                        this->fps_ = 30;
                }

                if (this->start_ != "")
                {
                    if (*this->start_.rbegin() == '%')
                    {
                        convertStringToValue<unsigned int>(&this->startFrame_, this->start_.substr(0, this->start_.size() - 1));
                        this->startFrame_ *= (cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT) - 1);
                        this->startFrame_ /= 100;
                    }
                    else
                    {
                        convertStringToValue<unsigned int>(&this->startFrame_, this->start_);
                    }

                    if (this->startFrame_ < 0 || this->startFrame_ >= cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT))
                    {
                        std::cout << "Error: Start frame " << this->startFrame_ << " is out of bounds (frames: 0 - " << (cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT) - 1) << ")" << std::endl;
                        delete this;
                        return;
                    }
                }

                if (this->speedFactor_ < 0 && this->startFrame_ == 0)
                {
                    this->startFrame_ = cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT) - 1;
                }

                if (this->startFrame_ != 0)
                {
                    cvSetCaptureProperty(this->cvCapture_, CV_CAP_PROP_POS_FRAMES, this->startFrame_);
                }
            }
            else
            {
                std::cout << "Error: Couln't open file \"" << this->filename_ << "\"" << std::endl;
            }
        }
        else
        {
            std::cout << "Error: Couldn't load sequence, no directory or filename specified" << std::endl;
        }

        if (!this->cvCapture_)
            delete this;
    }

    void VideoInput::tick()
    {
        if (this->counter_ == 0 || !this->fps_ || this->timer_.readyForNextFrame(this->fps_))
        {
            this->timer_.reset(this->fps_);

            unsigned int frame = ceil(this->startFrame_ + this->counter_ * this->speedFactor_);
            IplImage* image = 0;

            if (frame >= 0 && frame < cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT))
            {
                if (this->speedFactor_ != 1.0f)
                    cvSetCaptureProperty(this->cvCapture_, CV_CAP_PROP_POS_FRAMES, frame);
                image = cvQueryFrame(this->cvCapture_);
            }

            if (!image)
            {
                if (this->counter_ == 0)
                {
                    std::cout << "Error: Can't play video." << std::endl;
                    this->bRepeat_ = false;
                }

                if (!this->bRepeat_)
                {
                    delete this;
                    return;
                }
                else
                {
                    this->counter_ = 0;
                    return;
                }
            }
            else
            {
                ++this->counter_;

                this->getCamera()->setInputResourceName(this->filename_ +
                                                        "] [" + convertValueToString(frame) + "/" + convertValueToString(cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_FRAME_COUNT)) +
                                                        "] [" + convertValueToString((int)ceil(cvGetCaptureProperty(this->cvCapture_, CV_CAP_PROP_POS_AVI_RATIO) * 100)) + "%");
                this->getCamera()->processImage(image);
            }
        }
    }
}

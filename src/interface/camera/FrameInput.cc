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

#include "FrameInput.h"
#include <highgui.h>

namespace pixhawk
{
    RegisterInputOperation(FrameInput)
        .setDescription("Loads a single frame from an image file")
        .addName("frame")
        .addName("f")
        .addName("image")
        .addParameter("file", &FrameInput::setFilename, "The path to the image file")
        .addParameter("repeat", &FrameInput::setRepeat, "Repeat the frame forever", false)
        .addParameter("fps", &FrameInput::setFPS, "Frames per second (if repeat is 1)", 1)
    ;

    FrameInput::FrameInput(Camera* camera) : InputOperation(camera)
    {
        this->getCamera()->setInputModeName("Frame");
        this->getCamera()->setInputResourceName("no filename specified");
        this->bRepeat_ = false;
        this->fps_ = 1;
        this->image_ = 0;
        this->bFirstFrame_ = true;
    }

    FrameInput::~FrameInput()
    {
        if (this->image_)
            cvReleaseImage(&this->image_);
    }

    void FrameInput::setFilename(const std::string& filename)
    {
        this->filename_ = filename;
        this->getCamera()->setInputResourceName(filename);
    }

    void FrameInput::startPlayback()
    {
        if (this->filename_ != "")
        {
            this->image_ = cvLoadImage(this->filename_.c_str(), CV_LOAD_IMAGE_UNCHANGED);
            if (!this->image_)
                std::cout << "Error: Couln't open file \"" << this->filename_ << "\"" << std::endl;
        }
        else
        {
            std::cout << "Error: Couldn't load frame, no filename specified" << std::endl;
        }

        if (!this->image_)
            delete this;
    }

    void FrameInput::tick()
    {
        if (!this->bRepeat_)
        {
            this->getCamera()->processImage(this->image_);
            delete this;
        }
        else
        {
            if (this->bFirstFrame_ || !this->fps_ || this->timer_.readyForNextFrame(this->fps_))
            {
                this->timer_.reset(this->fps_);
                this->getCamera()->processImage(this->image_);
                this->bFirstFrame_ = false;
            }
        }
    }
}

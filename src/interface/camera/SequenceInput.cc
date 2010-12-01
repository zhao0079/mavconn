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

#include "SequenceInput.h"
#include <highgui.h>
#include <boost/filesystem.hpp>
#include "SequenceOutput.h"

namespace MAVCONN
{
    RegisterInputOperation(SequenceInput)
        .setDescription("Plays a sequence of continuously numbered image files")
        .addName("sequence")
        .addName("s")
        .addParameter("name", &SequenceInput::setName, "Name of the directory which contains the sequence or the full path to the start-frame")
        .addParameter("fps", &SequenceInput::setFPS, "Frames per second", 30)
        .addParameter("speed", &SequenceInput::setSpeedFactor, "A speed factor to repeat or drop frames.\nNegative values will play the sequence backward", 1.0f)
        .addParameter("repeat", &SequenceInput::setRepeat, "Repeat the sequence after it ended", false)
    ;

    SequenceInput::SequenceInput(Camera* camera) : InputOperation(camera)
    {
        this->getCamera()->setInputModeName("Sequence");
        this->getCamera()->setInputResourceName("no directory specified");
        this->bRepeat_ = false;
        this->fps_ = 30;
        this->counter_ = 0;
        this->startFrame_ = 1;
        this->numberOfFrames_ = 0;
        this->speedFactor_ = 1.0f;
        this->numberWidth_ = 6;
    }

    SequenceInput::~SequenceInput()
    {
    }

    void SequenceInput::setName(const std::string& name)
    {
        this->name_ = name;
        this->getCamera()->setInputResourceName(name);
    }

    void SequenceInput::startPlayback()
    {
        if (this->name_ == "")
        {
            std::cout << "Error: Couldn't load sequence, no directory or filename specified" << std::endl;
            delete this;
            return;
        }

        if (boost::filesystem::exists(this->name_))
        {
            if (boost::filesystem::is_directory(this->name_))
            {
                if (this->name_.size() > 0 && this->name_[this->name_.size() - 1] != '/')
                    this->name_ += '/';

                std::string path = getSequenceFramePath(this->name_, 1, ".", this->numberWidth_);

                try
                {
                    boost::filesystem::directory_iterator file(this->name_);
                    boost::filesystem::directory_iterator end;

                    bool success = false;
                    this->numberOfFrames_ = 0;

                    while (file != end)
                    {
                        if (!success && !boost::filesystem::is_directory(*file) && (*file).string().find(path) != std::string::npos)
                        {
                            this->extension_ = boost::filesystem::extension(*file);
                            success = true;
                        }

                        ++this->numberOfFrames_;
                        ++file;
                    }

                    if (this->speedFactor_ < 0)
                        this->startFrame_ = this->numberOfFrames_;
                }
                catch (...) {}
            }
            else
            {
                size_t start = 0;
                size_t end = this->name_.find_last_of('.');

                for (start = end - 1; start < this->name_.size(); --start)
                    if (this->name_[start] < '0' || this->name_[start] > '9')
                        break;

                if (!convertStringToValue<unsigned int>(&this->startFrame_, this->name_.substr(start + 1, end - start - 1)))
                {
                    this->numberWidth_ = 0;
                }
                else
                {
                    this->numberWidth_ = end - start - 1;
                    this->extension_ = this->name_.substr(end);
                    this->name_ = this->name_.substr(0, start + 1);

                    for (this->numberOfFrames_ = this->startFrame_; boost::filesystem::exists(getSequenceFramePath(this->name_, this->numberOfFrames_ + 1, this->extension_, this->numberWidth_)); ++this->numberOfFrames_);
                }
            }
        }
        else
        {
            std::cout << "Error: Can't play sequence, \"" << this->name_ << "\" doesn't exist." << std::endl;
            delete this;
        }
    }

    void SequenceInput::tick()
    {
        if (this->counter_ == 0 || !this->fps_ || this->timer_.readyForNextFrame(this->fps_))
        {
            this->timer_.reset(this->fps_);

            unsigned int frame = ceil(this->startFrame_ + this->counter_ * this->speedFactor_);
            std::string path = getSequenceFramePath(this->name_, frame, this->extension_, this->numberWidth_);

            IplImage* image = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
            if (!image)
            {
                if (this->counter_ == 0)
                {
                    std::cout << "Error: No frames found (searched for " << path << ".*)" << std::endl;
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
                    this->tick();
                    return;
                }
            }
            else
            {
                ++this->counter_;
                this->getCamera()->setInputResourceName(this->name_ + "] [" + convertValueToString(frame) + "/" + convertValueToString(this->numberOfFrames_));
                this->getCamera()->processImage(image);
                cvReleaseImage(&image);
            }
        }
    }
}

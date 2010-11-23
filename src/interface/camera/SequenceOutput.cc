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

#include "SequenceOutput.h"
#include <highgui.h>
#include <ctime>
#include <algorithm>
#include <boost/filesystem.hpp>

namespace pixhawk
{
    RegisterOutputOperation(SequenceOutput)
        .setDescription("Writes the camera frames to the filesystem as a sequence of image files")
        .addName("sequence")
        .addName("s")
        .addParameter("name", &SequenceOutput::setName, "Name of the sequence, will be used as the directory name (can contain strftime date format specifiers)", "sequence_%Y-%m-%d_%H-%M-%S")
        .addParameter("extension", &SequenceOutput::setExtension, "Extension of the image files", "bmp")
        .addParameter("fps", &SequenceOutput::setFPS, "Max. frames per second (0 = use all frames)", 0)
    ;

    SequenceOutput::SequenceOutput(Camera* camera, bool bVerbose) : OutputOperation(camera)
    {
        this->bVerbose_ = bVerbose;
        this->setName("sequence_%Y-%m-%d_%H-%M-%S");
        this->setExtension("bmp");
        this->setFPS(0);
        this->counter_ = 0;
    }

    SequenceOutput::~SequenceOutput()
    {
        if (this->counter_ > 0 && this->bVerbose_)
            std::cout << "Stop capturing sequence (" << this->name_ << "), captured " << this->counter_ << " images" << std::endl;
    }

    void SequenceOutput::setName(const std::string& name)
    {
        size_t length = name.size() + 20;
        time_t rawtime;
        struct tm* timeinfo;
        char timestring[length];

        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(timestring, length, name.c_str(), timeinfo);

        this->name_ = timestring;
        if (this->name_.size() > 0 && this->name_[this->name_.size() - 1] != '/')
            this->name_ += '/';
    }

    void SequenceOutput::setExtension(const std::string& extension)
    {
        if (extension.size() > 0 && extension[0] == '.')
            this->extension_ = extension;
        else
            this->extension_ = "." + extension;
    }

    void SequenceOutput::processImage(IplImage* image)
    {
        if (this->counter_ == 0 || !this->fps_ || this->timer_.readyForNextFrame(this->fps_))
        {
            this->timer_.reset(this->fps_);

            if (this->counter_ == 0)
            {
                if (this->bVerbose_)
                    std::cout << "Start capturing sequence (" << this->name_ << ")" << std::endl;

                boost::filesystem::create_directory(this->name_);
            }

            ++this->counter_;

            std::string path = getSequenceFramePath(this->name_, this->counter_, this->extension_);

            if (!cvSaveImage(path.c_str(), image))
            {
                std::cout << "Error: Couldn't save frame as \"" << path << "\"." << std::endl;
                delete this;
                return;
            }
        }
    }

    std::string getSequenceFramePath(const std::string& name, unsigned int counter, const std::string& extension, unsigned int numberWidth)
    {
        std::string number = convertValueToString(counter);
        std::string filename;
        for (size_t i = 0; i < numberWidth; ++i)
            filename += '0';
        filename += extension;

        if (numberWidth > 0)
            filename.replace(std::max<int>(0, numberWidth - number.size()), std::min<int>(numberWidth, number.size()), number);

        std::string path = name;
        path += filename;

        return path;
    }
}

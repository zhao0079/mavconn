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

#include "FrameOutput.h"
#include <highgui.h>
#include <ctime>

namespace pixhawk
{
    RegisterOutputOperation(FrameOutput)
        .setDescription("Stores one frame as an image file")
        .addName("frame")
        .addName("f")
        .addName("image")
        .addParameter("file", &FrameOutput::setFilename, "The file name of the destination (can contain strftime date format specifiers and %i for a frame counter)", "frame_%Y-%m-%d_%H-%M-%S_%i")
        .addParameter("extension", &FrameOutput::setExtension, "The file extension in case no file name was provided", "bmp")
    ;

    unsigned int FrameOutput::counter_s = 1;

    FrameOutput::FrameOutput(Camera* camera, bool bVerbose) : OutputOperation(camera)
    {
        this->bVerbose_ = bVerbose;
        this->setFilename("frame_%Y-%m-%d_%H-%M-%S_%i");
        this->setExtension("bmp");
    }

    FrameOutput::~FrameOutput()
    {
    }

    void FrameOutput::processImage(IplImage* image)
    {
        time_t rawtime;
        struct tm* timeinfo;
        size_t length = this->mask_.size() + 20;
        char timestring[length];

        time(&rawtime);
        timeinfo = localtime(&rawtime);
//        strftime(timestring, 32, "frame_%Y-%m-%d_%H-%M-%S_%i", timeinfo);
//        strftime(timestring, 32, "frame_%Y-%m-%d_%i", timeinfo);

        if (!strftime(timestring, length, this->mask_.c_str(), timeinfo))
        {
            if (this->mask_.size() > 0)
                std::cout << "Error: Filename contains too many date format specifiers" << std::endl;
            else
                std::cout << "Error: No filename specified" << std::endl;
        }
        else
        {
            std::string filename = timestring;

            bool found = false;

            size_t pos;
            while ((pos = filename.find("%i")) != std::string::npos)
            {
                filename.replace(pos, 2, convertValueToString(FrameOutput::counter_s));
                found = true;
            }

            if (found)
                ++FrameOutput::counter_s;

            if (filename.find('.') == std::string::npos)
            {
                filename += this->extension_;
            }

            int success = cvSaveImage(filename.c_str(), image);
            if (!success)
                std::cout << "Error: Couldn't save frame as \"" << filename << "\"." << std::endl;
            else if (this->bVerbose_)
                std::cout << "Captured frame (" << filename << ")" << std::endl;
        }

        delete this;
    }
}

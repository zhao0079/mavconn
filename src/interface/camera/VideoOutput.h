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

#ifndef _VideoOutput_H__
#define _VideoOutput_H__

#include "CameraOperations.h"
#include "FPSTimer.h"
#include <highgui.h>

namespace pixhawk
{
    class VideoOutput : public OutputOperation
    {
        public:
            VideoOutput(Camera* camera, bool bVerbose = false);
            ~VideoOutput();

            void processImage(IplImage* image);

            void setFilename(const std::string& name);
            void setFPS(float fps);
            void setVideoFPS(float fps);

        private:
            bool bVerbose_;
            std::string filename_;
            float fps_;
            float vfps_;
            CvVideoWriter* videoWriter_;
            FPSTimer timer_;
    };
}

#endif /* _VideoOutput_H__ */

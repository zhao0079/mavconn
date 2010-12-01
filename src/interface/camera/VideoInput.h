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

#ifndef _VideoInput_H__
#define _VideoInput_H__

#include "CameraOperations.h"
#include "FPSTimer.h"
#include <highgui.h>

namespace MAVCONN
{
    class VideoInput : public InputOperation
    {
        public:
            VideoInput(Camera* camera);
            ~VideoInput();

            void startPlayback();
            void tick();

            void setFilename(const std::string& filename);
            inline void setRepeat(bool bRepeat)
                { this->bRepeat_ = bRepeat; }
            inline void setFPS(float fps)
                { this->fps_ = fps; }
            inline void setSpeedFactor(float factor)
                { this->speedFactor_ = factor; }
            inline void setStart(const std::string& start)
                { this->start_ = start; }

        private:
            std::string  filename_;
            bool         bRepeat_;
            float        fps_;
            FPSTimer     timer_;
            unsigned int counter_;
            unsigned int startFrame_;
            std::string  start_;
            float        speedFactor_;
            CvCapture*   cvCapture_;
    };
}

#endif /* _VideoInput_H__ */

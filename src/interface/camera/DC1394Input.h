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

#ifndef _CameraInput_H__
#define _CameraInput_H__

#include "CameraOperations.h"
#include <highgui.h>

namespace pixhawk
{
    class CameraInput : public InputOperation
    {
        public:
            CameraInput(Camera* camera);
            ~CameraInput();

            void startPlayback();
            void tick();

            inline void setIndex(int index)
                { this->index_ = index; }
            inline void setDelay(unsigned int milliseconds)
                { this->delay_ = milliseconds; }

        private:
            CvCapture* capture_;
            int index_;
            unsigned int delay_;
    };
}

#endif /* _CameraInput_H__ */

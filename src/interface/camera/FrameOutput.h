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

#ifndef _FrameOutput_H__
#define _FrameOutput_H__

#include "CameraOperations.h"

namespace MAVCONN
{
    class FrameOutput : public OutputOperation
    {
        public:
            FrameOutput(Camera* camera, bool bVerbose = false);
            ~FrameOutput();

            void processImage(IplImage* image);

            inline void setFilename(const std::string& mask)
              { this->mask_ = mask; }

            inline void setExtension(const std::string& extension)
                { this->extension_ = "." + extension; }

        private:
            bool bVerbose_;
            std::string mask_;
            std::string extension_;
            static unsigned int counter_s;
    };
}

#endif /* _FrameOutput_H__ */

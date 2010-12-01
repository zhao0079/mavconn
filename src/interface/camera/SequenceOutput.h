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

#ifndef _SequenceOutput_H__
#define _SequenceOutput_H__

#include "CameraOperations.h"
#include "FPSTimer.h"

namespace MAVCONN
{
    class SequenceOutput : public OutputOperation
    {
        public:
            SequenceOutput(Camera* camera, bool bVerbose = false);
            ~SequenceOutput();

            void processImage(IplImage* image);

            void setName(const std::string& name);
            void setExtension(const std::string& extension);
            inline void setFPS(float fps)
                { this->fps_ = fps; }

        private:
            bool bVerbose_;
            std::string name_;
            std::string extension_;
            float fps_;
            unsigned int counter_;
            FPSTimer timer_;
    };

    std::string getSequenceFramePath(const std::string& name, unsigned int counter, const std::string& extension, unsigned int numberWidth = 6);
}

#endif /* _SequenceOutput_H__ */

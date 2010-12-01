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

#ifndef _SequenceInput_H__
#define _SequenceInput_H__

#include "CameraOperations.h"
#include "FPSTimer.h"

namespace MAVCONN
{
    class SequenceInput : public InputOperation
    {
        public:
            SequenceInput(Camera* camera);
            ~SequenceInput();

            void startPlayback();
            void tick();

            void setName(const std::string& name);
            inline void setRepeat(bool bRepeat)
                { this->bRepeat_ = bRepeat; }
            inline void setFPS(float fps)
                { this->fps_ = fps; }
            inline void setSpeedFactor(float factor)
                { this->speedFactor_ = factor; }

        private:
            std::string  name_;
            std::string  extension_;
            bool         bRepeat_;
            float        fps_;
            FPSTimer     timer_;
            unsigned int counter_;
            unsigned int startFrame_;
            unsigned int numberOfFrames_;
            unsigned int numberWidth_;
            float        speedFactor_;
    };
}

#endif /* _SequenceInput_H__ */

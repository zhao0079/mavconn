/*======================================================================

MAVCONN mcvlib - The Micro Computer Vision Library
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  @author Fabian Landau
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

#ifndef _LcmInput_H__
#define _LcmInput_H__

#include "CameraOperations.h"
#include "PxSharedMemClient.h"
#include "timer/Timer.h"

typedef struct _lcm_t lcm_t;

namespace MAVCONN
{
    class LcmInput : public InputOperation
    {
        public:
            LcmInput(Camera* camera);
            ~LcmInput();

            void startPlayback();
            void tick();

        private:
            static void image_handler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData);

            lcm_t* lcm_;
            mavlink_message_t_subscription_t* subscription_;
            PxSharedMemClient imageClient_;
            unsigned long interval_;
            Timer timeoutTimer_;
    };
}

#endif /* _LcmInput_H__ */

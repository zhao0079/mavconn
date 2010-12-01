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

#ifndef _DisplayOutput_H__
#define _DisplayOutput_H__

#include "CameraOperations.h"
#include "timer/Timer.h"
#include "mavconn.h"

namespace MAVCONN
{
    class SequenceOutput;
    class VideoOutput;

    class DisplayOutput : public OutputOperation
    {
        public:
            DisplayOutput(Camera* camera);
            ~DisplayOutput();

            void startProcessing();
            void stopProcessing();
            void endProcessing();

            void processImage(IplImage* image);
            void tick();

            inline void setDrawOverlay(bool bDraw)
                { this->bDrawOverlay_ = bDraw; }

        private:
            void drawOverlay();
            void createFallbackImage();
            void updateDisplay();
            void setStatus(const std::string& status);
            void receivedHeartbeat();
            void lcmHandle();
            static void mavlinkHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData);

            bool bStartedProcessing_;
            bool bDrawOverlay_;
            bool bShowHelp_;
            std::string windowname_;
            CvFont font_;
            CvFont fontLarge_;
            IplImage* fallback_;
            IplImage* input_;
            IplImage* output_;
            unsigned int framecount_;
            unsigned long oldtpf_;
            unsigned int oldframes_;
            float fps_;
            unsigned int seconds_;
            Timer timer_;
            Timer fpstimer_;
            std::string status_;
            Timer statusTimer_;
            SequenceOutput* sequenceOutput_;
            VideoOutput* videoOutput_;
            lcm_t* lcm_;
            mavlink_message_t_subscription_t* subscription_;
            bool bReceivedHeartbeat_;
            Timer heartbeatTimer_;
    };
}

#endif /* _DisplayOutput_H__ */

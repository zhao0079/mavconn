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

#ifndef _Command_H__
#define _Command_H__

#include "inttypes.h"
#include <signal.h>
#include <string>

// forward declarations
typedef struct _lcm_recv_buf_t lcm_recv_buf_t;
typedef struct __mavlink_message mavlink_message_t;

namespace pixhawk
{
    namespace watchdog
    {
        ///! Command codes, used to send and receive commands over lcm
        namespace command
        {
            enum Enum
            {
                Start         = 0,
                Restart       = 1,
                Stop          = 2,
                Mute          = 3,
                Unmute        = 4,

                RequestInfo   = 254,
                RequestStatus = 255
            };
        }

        namespace process
        {
            ///! Process state - each process is in exactly one of those states (except unknown, that's just to initialize it)
            namespace state
            {
                enum Enum
                {
                    Unknown       = 0,
                    Running       = 1,
                    Stopped       = 2,
                    Stopped_OK    = 3,
                    Stopped_ERROR = 4
                };
            }

            static const uint16_t ALL         = (uint16_t)-1;   ///< A magic value for a process-ID which addresses "all processes"
            static const uint16_t ALL_RUNNING = (uint16_t)-2;   ///< A magic value for a process-ID which addresses "all running processes"
            static const uint16_t ALL_CRASHED = (uint16_t)-3;   ///< A magic value for a process-ID which addresses "all crashed processes"
        }

        // forward declarations
        class Watchdog;
        class Process;

        uint16_t getWatchdogID();
        void commandHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData);
        void handleWatchdogCommand(Watchdog* watchdog, const Process& process, uint8_t command);
        void sendWatchdogCommandHeartbeat(Watchdog* watchdog);
        void sendWatchdogCommandProcessInfo(Watchdog* watchdog, const Process& process);
        void sendWatchdogCommandProcessStatus(Watchdog* watchdog, const Process& process);
    }
}

#endif /* _Command_H__ */

/*======================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  @author Fabian Landau <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

#ifndef _WatchdogControl_H__
#define _WatchdogControl_H__

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <highgui.h>

#include "timer/Timer.h"

// forward declarations
typedef struct _lcm_t lcm_t;
typedef struct _mavlink_message_t_subscription_t mavlink_message_t_subscription_t;
typedef struct _lcm_recv_buf_t lcm_recv_buf_t;
typedef struct __mavlink_message mavlink_message_t;

namespace MAVCONN
{
    class WatchdogControl
    {
        ///! Command codes, used to send and receive commands over lcm
        struct Command
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
        };

        ///! This struct represents a process on the watchdog. Used to store all values.
        struct ProcessInfo
        {
            ///! Process state - each process is in exactly one of those states (except unknown, that's just to initialize it)
            struct State
            {
                enum Enum
                {
                    Unknown       = 0,
                    Running       = 1,
                    Stopped       = 2,
                    Stopped_OK    = 3,
                    Stopped_ERROR = 4
                };
            };

            ///! Constructor - initialize the values
            ProcessInfo() : timeout_(0), state_(State::Unknown), muted_(false), crashes_(0), pid_(-1) {}

            std::string name_;      ///< The name of the process
            std::string arguments_; ///< The arguments (argv of the process)

            int32_t timeout_;       ///< Heartbeat timeout value (in microseconds)

            State::Enum state_;     ///< The current state of the process
            bool muted_;            ///< True if the process is currently muted
            uint16_t crashes_;      ///< The number of crashes
            int32_t pid_;           ///< The PID of the process

            Timer requestTimer_;    ///< Internal timer, used to repeat status and info requests after some time (in case of packet loss)
            Timer updateTimer_;     ///< Internal timer, used to measure the time since the last update (used only for graphics)
        };

        ///! This struct identifies a watchdog. It's a combination of system-ID and watchdog-ID. implements operator< to be used as key in a std::map
        struct WatchdogID
        {
            ///! Constructor - initialize the values
            WatchdogID(uint8_t system_id, uint16_t watchdog_id) : system_id_(system_id), watchdog_id_(watchdog_id) {}

            uint8_t system_id_;     ///< The system-ID
            uint16_t watchdog_id_;  ///< The watchdog-ID

            ///! Comparison operator which is used by std::map
            inline bool operator<(const WatchdogID& other) const
                { return (this->system_id_ != other.system_id_) ? (this->system_id_ < other.system_id_) : (this->watchdog_id_ < other.watchdog_id_); }

        };

        ///! This struct represents a watchdog
        struct WatchdogInfo
        {
            ProcessInfo& getProcess(uint16_t index);

            std::vector<ProcessInfo> processes_;    ///< A vector containing all processes running on this watchdog
            Timer updateTimer_;                     ///< Internal timer, used to request a status update of all processes after some time
            Timer timeoutTimer_;                    ///< Internal timer, used to measure the time since the last heartbeat message
        };

        public:
            WatchdogControl();
            ~WatchdogControl();

            void run();

            static const uint16_t ALL         = (uint16_t)-1;   ///< A magic value for a process-ID which addresses "all processes"
            static const uint16_t ALL_RUNNING = (uint16_t)-2;   ///< A magic value for a process-ID which addresses "all running processes"
            static const uint16_t ALL_CRASHED = (uint16_t)-3;   ///< A magic value for a process-ID which addresses "all crashed processes"

        private:
            void sendCommand(const WatchdogID& w_id, uint16_t p_id, Command::Enum command);

            WatchdogInfo& getWatchdog(uint8_t system_id, uint16_t watchdog_id);
            static void mavlinkHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData);
            void lcmHandle();

            std::map<WatchdogID, WatchdogInfo> watchdogs_;      ///< A map containing all watchdogs which are currently active
            lcm_t* lcm_;                                        ///< The lcm connection
            mavlink_message_t_subscription_t* subscription_;    ///< The mavlink subscription

        private:
            void createGraphics();
            void destroyGraphics();
            void tickGraphics();
            bool button(int x, int y, const char* text);
            static void mouseCallback(int event, int x, int y, int flags, void* param);

            std::string windowname_;    ///< The name of the display window
            IplImage* display_;         ///< The image that will be displayed in the window
            CvFont font_;               ///< The used font
            int mouseX_;                ///< Mouse x coordinate (set after a left-click by @ref mouseCallback)
            int mouseY_;                ///< Mouse y coordinate (set after a left-click by @ref mouseCallback)
    };

    ///! Convert a value to std::string
    template <class T>
    std::string convertToString(T value)
    {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }
}

#endif /* _WatchdogControl_H__ */

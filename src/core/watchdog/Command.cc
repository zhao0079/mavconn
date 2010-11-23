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

#include "Command.h"
#include "Watchdog.h"
#include <iostream>
#include <mavconn.h>

namespace pixhawk
{
namespace watchdog
{
    // hardcoded values for lcm
    uint8_t sysid = 42;
    uint8_t compid = 160;

    /**
        @brief Returns the watchdog's ID (currently defined as the PID of the process)
    */
    uint16_t getWatchdogID()
    {
        return getppid();
    }

    /**
        @brief Callback function for lcm to receive MAVLINK messages.
    */
    void commandHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData)
    {
        Watchdog* _this = static_cast<Watchdog*>(userData);

        // check if it's a watchdog command message
        if (msg->msgid == MAVLINK_MSG_ID_WATCHDOG_COMMAND)
        {
            mavlink_watchdog_command_t payload;
            mavlink_msg_watchdog_command_decode(msg, &payload);

            // check if the message addresses this watchdog (sysid and watchdog ID must match)
            if (payload.target_system_id == sysid && payload.watchdog_id == getWatchdogID())
            {
                const std::vector<Process*>& processes = _this->getProcesses();

                // handle the magic process-ID values (ALL, ALL_CRASHED, ALL_RUNNING)
                switch (payload.process_id)
                {
                    case process::ALL:
                    {
                        for (size_t i = 0; i < processes.size(); ++i)
                            handleWatchdogCommand(_this, *processes[i], payload.command_id);
                    }
                    break;

                    case process::ALL_CRASHED:
                    {
                        for (size_t i = 0; i < processes.size(); ++i)
                            if (processes[i]->isCrashed())
                                handleWatchdogCommand(_this, *processes[i], payload.command_id);
                    }
                    break;

                    case process::ALL_RUNNING:
                    {
                        for (size_t i = 0; i < processes.size(); ++i)
                            if (processes[i]->isRunning())
                                handleWatchdogCommand(_this, *processes[i], payload.command_id);
                    }
                    break;

                    default:
                    {
                        // default: just get the process with the given ID
                        try
                        {
                            handleWatchdogCommand(_this, _this->getProcessByCode(payload.process_id), payload.command_id);
                        }
                        catch (...) {}
                    }
                    break;
                }
//std::cout << "<-- received mavlink_watchdog_command_t " << msg->sysid << " / " << payload.watchdog_id << " / " << payload.process_id << " / " << payload.command_id << std::endl;
            }
        }
    }

    /**
        @brief Responds to a watchdog command message, depending on the command-ID
        @param watchdog A pointer to the watchdog
        @param process The concerned process
        @param command The command-ID
    */
    void handleWatchdogCommand(Watchdog* watchdog, const Process& process, uint8_t command)
    {
        switch (command)
        {
            case command::RequestInfo:
                sendWatchdogCommandProcessInfo(watchdog, process);
                break;

            case command::RequestStatus:
                sendWatchdogCommandProcessStatus(watchdog, process);
                break;

            case command::Start:
                process.scheduleStart();
                break;

            case command::Restart:
                process.scheduleRestart();
                break;

            case command::Stop:
                process.scheduleStop();
                break;

            case command::Mute:
                process.mute();
                sendWatchdogCommandProcessStatus(watchdog, process);
                break;

            case command::Unmute:
                process.unmute();
                sendWatchdogCommandProcessStatus(watchdog, process);
                break;
        }
    }

    /**
        @brief Sends a heartbeat message (used by the watchdog itself in it's main-loop)
    */
    void sendWatchdogCommandHeartbeat(Watchdog* watchdog)
    {
        mavlink_watchdog_heartbeat_t payload;
        payload.watchdog_id = getWatchdogID();
        payload.process_count = Watchdog::getInstance().getNumProcesses();

        mavlink_message_t msg;
        mavlink_msg_watchdog_heartbeat_encode(sysid, compid, &msg, &payload);
        mavlink_message_t_publish(watchdog->getLcm(), "MAVLINK", &msg);
//std::cout << "--> sent mavlink_watchdog_heartbeat_t" << std::endl;
    }

    /**
        @brief Sends information (name, arguments, etc) about a process. Sent only on request (see @ref command::RequestInfo)
    */
    void sendWatchdogCommandProcessInfo(Watchdog* watchdog, const Process& process)
    {
        mavlink_watchdog_process_info_t payload;
        payload.watchdog_id = getWatchdogID();
        payload.process_id = process.getCode();

        // copy the name into the array. if the name is too long, copy as much as possible. append a \0 character.
        size_t nameLength = std::min(sizeof(payload.name) - 1, process.getName().size());
        memcpy(payload.name, process.getName().c_str(), nameLength);
        payload.name[nameLength] = '\0';

        // join the arguments to one string. copy the arguments into the array. if the arguments are too long, copy as much as possible. append a \0 character.
        std::string arguments;
        for (unsigned int j = 1; j < process.getArguments().size(); ++j)
        {
            if (j > 1)
                arguments += ' ';
            arguments += process.getArguments()[j];
        }
        size_t argumentsLength = std::min(sizeof(payload.arguments) - 1, arguments.size());
        memcpy(payload.arguments, arguments.c_str(), argumentsLength);
        payload.arguments[argumentsLength] = '\0';

        payload.timeout = process.getHeartbeatTimeout();

        mavlink_message_t msg;
        mavlink_msg_watchdog_process_info_encode(sysid, compid, &msg, &payload);
        mavlink_message_t_publish(watchdog->getLcm(), "MAVLINK", &msg);
//std::cout << "--> sent mavlink_watchdog_process_info_t" << std::endl;
    }

    /**
        @brief Sends a message containing the state of a process. Sent by the watchdog if a process' state changes or on request (see @ref command::RequestStatus)
    */
    void sendWatchdogCommandProcessStatus(Watchdog* watchdog, const Process& process)
    {
        mavlink_watchdog_process_status_t payload;
        payload.watchdog_id = getWatchdogID();
        payload.process_id = process.getCode();

        payload.state = (uint8_t)process::state::Unknown;
        if (process.isRunning())
            payload.state = (uint8_t)process::state::Running;
        if (process.isSuspended())
            payload.state = (uint8_t)process::state::Stopped;
        if (process.isFinished())
            payload.state = (uint8_t)process::state::Stopped_OK;
        if (process.isCrashed())
            payload.state = (uint8_t)process::state::Stopped_ERROR;

        payload.muted = process.isMuted();
        payload.pid = process.getPID();
        payload.crashes = process.getCrashs();

        mavlink_message_t msg;
        mavlink_msg_watchdog_process_status_encode(sysid, compid, &msg, &payload);
        mavlink_message_t_publish(watchdog->getLcm(), "MAVLINK", &msg);
//std::cout << "--> sent mavlink_watchdog_process_status_t" << std::endl;
    }
}
}

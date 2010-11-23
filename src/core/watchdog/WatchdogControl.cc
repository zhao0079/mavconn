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

#include "WatchdogControl.h"
#include "Core.h"
#include <mavconn.h>
#include <iostream>

// If no heartbeat was received for __WATCHDOG_CONTROL_TIMEOUT seconds, the watchdog is removed from the map
#define __WATCHDOG_CONTROL_TIMEOUT 10.0f

// Every __WATCHDOG_CONTROL_UPDATE seconds a status update is requested for alle processes
#define __WATCHDOG_CONTROL_UPDATE 5.0f

// If a (watchdog info) packet got lost, a new packet will be requested after __WATCHDOG_CONTROL_REQUEST_INTERVAL seconds
#define __WATCHDOG_CONTROL_REQUEST_INTERVAL 0.5f

/**
    @brief main function, create and run WatchdogControl.
*/
int main(int argc, char* argv[])
{
    pixhawk::Core core("watchdogcontrol");

    pixhawk::WatchdogControl control;
    control.run();

    return EXIT_SUCCESS;
}

namespace pixhawk
{
    // some hardcoded values for lcm
    uint8_t sysid = 42;
    uint8_t compid = 161;

    /**
        @brief Constructor - connect to lcm and subscribe for MAVLINK messages.
    */
    WatchdogControl::WatchdogControl()
    {
        // connect to lcm and subscribe for mavlink messages
        this->lcm_ = lcm_create("udpm://");
        if (this->lcm_)
            this->subscription_ = mavlink_message_t_subscribe(this->lcm_, "MAVLINK", &WatchdogControl::mavlinkHandler, this);

        this->createGraphics();
    }

    /**
        @brief Destructor - disconnect from lcm.
    */
    WatchdogControl::~WatchdogControl()
    {
        // disconnect from lcm
        if (this->lcm_)
        {
            if (this->subscription_)
                mavlink_message_t_unsubscribe(this->lcm_, this->subscription_);

            lcm_destroy(this->lcm_);
        }

        this->destroyGraphics();
    }

    /**
        @brief The main-loop - manage all watchdogs and processes in the map.
    */
    void WatchdogControl::run()
    {
        while (true)
        {
            // exit if someone pressed ESC
            unsigned char key = cvWaitKey(10);
            if (key == 27)
                break;

            // iterate through all watchdogs in the map
            for (std::map<WatchdogID, WatchdogInfo>::iterator w_it = this->watchdogs_.begin(); w_it != this->watchdogs_.end(); )
            {
                if (w_it->second.timeoutTimer_.getMilliseconds() > __WATCHDOG_CONTROL_TIMEOUT * 1000)
                {
                    // timeout detected, erase the watchdog from the map
                    this->watchdogs_.erase(w_it++);
                }
                else
                {
                    // check if it's time for a status update
                    if (w_it->second.updateTimer_.getMilliseconds() > __WATCHDOG_CONTROL_UPDATE * 1000 + (rand() % 1000))
                    {
                        // request a status update for all processes
                        this->sendCommand(w_it->first, WatchdogControl::ALL, Command::RequestStatus);
                        w_it->second.updateTimer_.reset();
                    }

                    // iterate through all processes in the vector
                    std::vector<ProcessInfo>& processes = w_it->second.processes_;
                    for (size_t p_it = 0; p_it < processes.size(); ++p_it)
                    {
                        ProcessInfo& process = processes[p_it];

                        // if there's enough time passed since the last request, check if name or state are missing and request the corresponding messages again
                        if (process.requestTimer_.getMilliseconds() > __WATCHDOG_CONTROL_REQUEST_INTERVAL * 1000)
                        {
                            if (process.name_ == "")
                            {
                                this->sendCommand(w_it->first, p_it, Command::RequestInfo);
                                process.requestTimer_.reset();
                            }

                            if (process.state_ == ProcessInfo::State::Unknown)
                            {
                                this->sendCommand(w_it->first, p_it, Command::RequestStatus);
                                process.requestTimer_.reset();
                            }
                        }
                    }

                    ++w_it;
                }
            }

            this->lcmHandle();

            this->tickGraphics();
        }
    }

    /**
        @brief poll for lcm messages.
    */
    void WatchdogControl::lcmHandle()
    {
        int fileno = lcm_get_fileno(this->lcm_);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fileno, &readfds);

        select(fileno + 1, &readfds, NULL, NULL, &tv);

        if (FD_ISSET(fileno, &readfds))
            lcm_handle(this->lcm_);
    }

    /**
        @brief Callback for lcm mavlink messages.
    */
    /* static */ void WatchdogControl::mavlinkHandler(const lcm_recv_buf_t* rbuf, const char* channel, const mavlink_message_t* msg, void* userData)
    {
        WatchdogControl* _this = static_cast<WatchdogControl*>(userData);

        switch (msg->msgid)
        {
            case MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT:
            {
                mavlink_watchdog_heartbeat_t payload;
                mavlink_msg_watchdog_heartbeat_decode(msg, &payload);

                // request the watchdog with the given ID
                WatchdogInfo& watchdog = _this->getWatchdog(msg->sysid, payload.watchdog_id);

                // if the proces count doesn't match, the watchdog is either new or has changed - create a new vector with new (and empty) ProcessInfo structs.
                if (watchdog.processes_.size() != payload.process_count)
                    watchdog.processes_ = std::vector<ProcessInfo>(payload.process_count);

                // start the timeout timer
                watchdog.timeoutTimer_.reset();
//std::cout << "<-- received mavlink_watchdog_heartbeat_t " << msg->sysid << " / " << payload.watchdog_id << " / " << payload.process_count << std::endl;
            }
            break;

            case MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO:
            {
                mavlink_watchdog_process_info_t payload;
                mavlink_msg_watchdog_process_info_decode(msg, &payload);

                // request the watchdog and the process with the given IDs
                WatchdogInfo& watchdog = _this->getWatchdog(msg->sysid, payload.watchdog_id);
                ProcessInfo& process = watchdog.getProcess(payload.process_id);

                // store the process information in the ProcessInfo struct
                process.name_ = (const char*)payload.name;
                process.arguments_ = (const char*)payload.arguments;
                process.timeout_ = payload.timeout;
//std::cout << "<-- received mavlink_watchdog_process_info_t " << msg->sysid << " / " << (const char*)payload.name << " / " << (const char*)payload.arguments << " / " << payload.timeout << std::endl;
            }
            break;

            case MAVLINK_MSG_ID_WATCHDOG_PROCESS_STATUS:
            {
                mavlink_watchdog_process_status_t payload;
                mavlink_msg_watchdog_process_status_decode(msg, &payload);

                // request the watchdog and the process with the given IDs
                WatchdogInfo& watchdog = _this->getWatchdog(msg->sysid, payload.watchdog_id);
                ProcessInfo& process = watchdog.getProcess(payload.process_id);

                // store the status information in the ProcessInfo struct
                process.state_ = static_cast<ProcessInfo::State::Enum>(payload.state);
                process.muted_ = payload.muted;
                process.crashes_ = payload.crashes;
                process.pid_ = payload.pid;

                process.updateTimer_.reset();
//std::cout << "<-- received mavlink_watchdog_process_status_t " << msg->sysid << " / " << payload.state << " / " << payload.muted << " / " << payload.crashes << " / " << payload.pid << std::endl;
            }

            case MAVLINK_MSG_ID_WATCHDOG_COMMAND:
            {
                mavlink_watchdog_command_t payload;
                mavlink_msg_watchdog_command_decode(msg, &payload);

                // Check if another instance of watchdog control sent a broadcast status request message
                if (payload.command_id == Command::RequestStatus && payload.process_id == WatchdogControl::ALL)
                    _this->getWatchdog(msg->sysid, payload.watchdog_id).updateTimer_.reset();
            }

            break;
        }
    }

    /**
        @brief Sends a watchdog command to a process on a given watchdog.
        @param w_id The WatchdogID struct (containing system-ID and watchdog-ID) that identifies the watchdog
        @param p_id The process-ID
        @param command The command-ID
    */
    void WatchdogControl::sendCommand(const WatchdogID& w_id, uint16_t p_id, Command::Enum command)
    {
        mavlink_watchdog_command_t payload;
        payload.target_system_id = w_id.system_id_;
        payload.watchdog_id = w_id.watchdog_id_;
        payload.process_id = p_id;
        payload.command_id = (uint8_t)command;

        mavlink_message_t msg;
        mavlink_msg_watchdog_command_encode(sysid, compid, &msg, &payload);
        mavlink_message_t_publish(this->lcm_, "MAVLINK", &msg);
//std::cout << "--> sent mavlink_watchdog_command_t " << payload.target_system_id << " / " << payload.watchdog_id << " / " << payload.process_id << " / " << (int)payload.command_id << std::endl;
    }

    /**
        @brief Returns a WatchdogInfo struct that belongs to the watchdog with the given system-ID and watchdog-ID
    */
    WatchdogControl::WatchdogInfo& WatchdogControl::getWatchdog(uint8_t system_id, uint16_t watchdog_id)
    {
        WatchdogID id(sysid, watchdog_id);

        std::map<WatchdogID, WatchdogInfo>::iterator it = this->watchdogs_.find(id);
        if (it != this->watchdogs_.end())
        {
            // the WatchdogInfo struct already exists in the map, return it
            return it->second;
        }
        else
        {
            // the WatchdogInfo struct doesn't exist - request info and status for all processes and create the struct
            this->sendCommand(id, WatchdogControl::ALL, Command::RequestInfo);
            this->sendCommand(id, WatchdogControl::ALL, Command::RequestStatus);
            return this->watchdogs_[id];
        }
    }

    /**
        @brief Returns a ProcessInfo struct that belongs to the process with the given ID.
    */
    WatchdogControl::ProcessInfo& WatchdogControl::WatchdogInfo::getProcess(uint16_t index)
    {
        // if the index is out of bounds, resize the vector
        if (index >= this->processes_.size())
            this->processes_.resize(index + 1);

        return this->processes_[index];
    }

    // only graphics code below this line
    // ------------------------------------------------------------------------

    /**
        @brief Initializes the display window and registers a mouse callback
    */
    void WatchdogControl::createGraphics()
    {
        this->windowname_ = "Watchdog Control";
        cvNamedWindow(this->windowname_.c_str());

        cvInitFont(&this->font_, CV_FONT_HERSHEY_PLAIN, 0.9, 1.0, 0, 1);

        this->display_ = cvCreateImage(cvSize(800, 1), IPL_DEPTH_8U, 1);

        cvSetMouseCallback(this->windowname_.c_str(), &WatchdogControl::mouseCallback, this);

        this->mouseX_ = -1;
        this->mouseY_ = -1;
    }

    /**
        @brief Destroys the graphic resources
    */
    void WatchdogControl::destroyGraphics()
    {
        if (this->display_)
            cvReleaseImage(&this->display_);;

        cvDestroyWindow(this->windowname_.c_str());
    }

    /**
        @brief Updates the graphics
    */
    void WatchdogControl::tickGraphics()
    {
        // make a black image
        cvZero(this->display_);

        int offset = 0;
        std::string line;

        for (std::map<WatchdogID, WatchdogInfo>::iterator w_it = this->watchdogs_.begin(); w_it != this->watchdogs_.end(); ++w_it)
        {
            if (w_it != this->watchdogs_.begin())
                offset += 10;

            offset += 20;

            std::vector<ProcessInfo>& processes = w_it->second.processes_;
            for (size_t p_it = 0; p_it < processes.size(); ++p_it)
                offset += 20;
        }

        // resize the image if necessary
        if (offset + 1 != this->display_->height)
        {
            int width = this->display_->width;

            cvReleaseImage(&this->display_);
            this->display_ = cvCreateImage(cvSize(width, offset + 1), IPL_DEPTH_8U, 1);
        }

        offset = 0;

        // iterate through all watchdogs in the map
        for (std::map<WatchdogID, WatchdogInfo>::iterator w_it = this->watchdogs_.begin(); w_it != this->watchdogs_.end(); ++w_it)
        {
            // if this is not the first watchdog in the map, make a smal gap
            if (w_it != this->watchdogs_.begin())
                offset += 10;

            std::vector<ProcessInfo>& processes = w_it->second.processes_;

            // draw a circle and fill it after a heartbeat message
            cvCircle(this->display_, cvPoint(10, 10 + offset), 6, cvScalar(255), 1, CV_AA);
            if (w_it->second.timeoutTimer_.getMilliseconds() < 1000)
            {
                unsigned char color = 255 - (255.0f * w_it->second.timeoutTimer_.getMilliseconds() / 1000);
                cvCircle(this->display_, cvPoint(10, 10 + offset), 4, cvScalar(color, color, color), -1, CV_AA);
            }

            // draw a button to request a status update for all processes
            if (this->button(30, 10 + offset, "U"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::RequestStatus);

            // display information about the watchdog
            line = "Watchdog (system: " + convertToString((int)w_it->first.system_id_) + ", id: " + convertToString(w_it->first.watchdog_id_) + ") - " + convertToString(processes.size()) + " Process";
            if (processes.size() > 1)
                line += "es";
            cvPutText(this->display_, line.c_str(), cvPoint(45, 15 + offset), &this->font_, cvScalar(255));

            // draw a button to start all processes
            if (this->button(500, 10 + offset, ">"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::Start);
            // draw a button to start all crashed processes
            if (this->button(515, 10 + offset, ">"))
                this->sendCommand(w_it->first, WatchdogControl::ALL_CRASHED, Command::Start);
            this->display_->imageData[515+3 + this->display_->widthStep * (10+3 + offset)] = 255;
            this->display_->imageData[515+4 + this->display_->widthStep * (10+4 + offset)] = 255;
            this->display_->imageData[515+4 + this->display_->widthStep * (10+2 + offset)] = 255;
            this->display_->imageData[515+2 + this->display_->widthStep * (10+4 + offset)] = 255;
            this->display_->imageData[515+2 + this->display_->widthStep * (10+2 + offset)] = 255;

            // draw a button to restart all processes
            if (this->button(545, 10 + offset, "R"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::Restart);
            // draw a button to restart all running processes
            if (this->button(560, 10 + offset, "R"))
                this->sendCommand(w_it->first, WatchdogControl::ALL_RUNNING, Command::Restart);
            this->display_->imageData[560+4 + this->display_->widthStep * (10+2 + offset)] = 255;
            this->display_->imageData[560+5 + this->display_->widthStep * (10+3 + offset)] = 255;
            this->display_->imageData[560+4 + this->display_->widthStep * (10+4 + offset)] = 255;

            // draw a button to stop all processes
            if (this->button(590, 10 + offset, "S"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::Stop);

            // draw a button to unmute all processes
            if (this->button(620, 10 + offset, "<"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::Unmute);
            // draw a button to mute all processes
            if (this->button(635, 10 + offset, "<"))
                this->sendCommand(w_it->first, WatchdogControl::ALL, Command::Mute);
            cvLine(this->display_, cvPoint(635-6, 10-6 + offset), cvPoint(635+6, 10+6 + offset), cvScalar(255));

            // move to the next line by incrementing the offset
            offset += 20;

            // iterate through all processes on this watchdog
            for (size_t p_it = 0; p_it < processes.size(); ++p_it)
            {
                ProcessInfo& process = processes[p_it];

                // draw a circle and fill it after a status update
                cvCircle(this->display_, cvPoint(30, 10 + offset), 6, cvScalar(255), 1, CV_AA);
                if (process.updateTimer_.getMilliseconds() < 1000)
                {
                    unsigned char color = 255 - (255.0f * process.updateTimer_.getMilliseconds() / 1000);
                    cvCircle(this->display_, cvPoint(30, 10 + offset), 4, cvScalar(color, color, color), -1, CV_AA);
                }

                // draw a button to request a status update for this process
                if (this->button(50, 10 + offset, "U"))
                    this->sendCommand(w_it->first, p_it, Command::RequestStatus);

                // display name and arguments of this process
                static const size_t MAXLENGTH = 35;
                std::string name;
                if (process.name_.size() <= MAXLENGTH)
                    name = process.name_;
                else
                    line = process.name_.substr(0, MAXLENGTH - 3) + "...";
                std::string arguments;
                if (process.arguments_.size() + name.size() <= MAXLENGTH)
                    arguments = process.arguments_;
                else
                    arguments = process.arguments_.substr(0, MAXLENGTH - 3 - name.size()) + "...";
                line = convertToString(p_it) + ": " + convertToString(process.pid_) + " " + name + " " + arguments;
                cvPutText(this->display_, line.c_str(), cvPoint(65, 15 + offset), &this->font_, cvScalar(255));

                // display the state of this process
                switch (process.state_)
                {
                    case ProcessInfo::State::Unknown: line = "unknown"; break;
                    case ProcessInfo::State::Running: line = "running"; break;
                    case ProcessInfo::State::Stopped: line = "stopped"; break;
                    case ProcessInfo::State::Stopped_OK: line = "stopped (ok)"; break;
                    case ProcessInfo::State::Stopped_ERROR: line = "stopped (error)"; break;
                    default: line = "not implemented"; break;
                }
                cvPutText(this->display_, line.c_str(), cvPoint(475, 15 + offset), &this->font_, cvScalar(255));

                // display whether this process is muted or not
                cvPutText(this->display_, "<", cvPoint(605, 15 + offset), &this->font_, cvScalar(255));
                if (process.muted_)
                    cvLine(this->display_, cvPoint(605-1, 15-11 + offset), cvPoint(605+11, 15+1 + offset), cvScalar(255));

                // display the number of crashes of this process
                line = convertToString(process.crashes_);
                cvPutText(this->display_, line.c_str(), cvPoint(635, 15 + offset), &this->font_, cvScalar(255));

                // draw a button to start this process
                if (this->button(700, 10 + offset, ">"))
                    this->sendCommand(w_it->first, p_it, Command::Start);
                // draw a button to restart this process
                if (this->button(715, 10 + offset, "R"))
                    this->sendCommand(w_it->first, p_it, Command::Restart);
                // draw a button to stop this process
                if (this->button(730, 10 + offset, "S"))
                    this->sendCommand(w_it->first, p_it, Command::Stop);

                // draw a button to unmute this process
                if (this->button(760, 10 + offset, "<"))
                    this->sendCommand(w_it->first, p_it, Command::Unmute);
                // draw a button to mute this process
                if (this->button(775, 10 + offset, "<"))
                    this->sendCommand(w_it->first, p_it, Command::Mute);
                cvLine(this->display_, cvPoint(775-6, 10-6 + offset), cvPoint(775+6, 10+6 + offset), cvScalar(255));

                // move to the next line by incrementing the offset
                offset += 20;
            }
        }

        // display the image
        cvShowImage(this->windowname_.c_str(), this->display_);

        // reset the mouse coordinates
        this->mouseX_ = -1;
        this->mouseY_ = -1;
    }

    /**
        @brief Create a button, centered on the x and y coordinates with a text in the middle (it's just a square). Handles also clicks on this buton.
    */
    bool WatchdogControl::button(int x, int y, const char* text)
    {
        // check if there's a mouse click on this button
        if (this->mouseX_ != -1 && this->mouseY_ != -1)
        {
            if (this->mouseX_ >= x-6 && this->mouseX_ <= x+6 && this->mouseY_ >= y-6 && this->mouseY_ <= y+6)
            {
                // yes, it's a click - display a filled square and return true
                cvRectangle(this->display_, cvPoint(x-6, y-6), cvPoint(x+6, y+6), cvScalar(255), CV_FILLED);
                return true;
            }
        }

        // no click at this point, display the regular button and return false
        cvRectangle(this->display_, cvPoint(x-6, y-6), cvPoint(x+6, y+6), cvScalar(255));
        cvPutText(this->display_, text, cvPoint(x-6, y+5), &this->font_, cvScalar(255));
        return false;
    }

    /**
        @brief A mouse callback (registered in @ref createGraphics)
    */
    /* static */ void WatchdogControl::mouseCallback(int event, int x, int y, int flags, void* param)
    {
        WatchdogControl* _this = static_cast<WatchdogControl*>(param);

        // if its a left-mouse-button-up event, set the mouse coordinates
        if (event &= CV_EVENT_LBUTTONUP)
        {
            _this->mouseX_ = x;
            _this->mouseY_ = y;
        }
    }
}

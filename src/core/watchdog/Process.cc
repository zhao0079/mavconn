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

#include "Process.h"
#include <climits>
#include <ctime>
#include <iostream>
#include <sstream>
#include "Watchdog.h"

namespace MAVCONN
{
namespace watchdog
{
    /**
     *
     */
    Process::Process()
    {
        this->pid_ = -1;
        this->code_ = 0;
        this->filedescriptor_ = 0;

        this->bRunning_ = false;
        this->bSuspended_ = false;
        this->bMuted_ = false;
        this->bHasOutput_ = false;

        this->heartbeat_microseconds_max_ = INT_MAX;
        this->heartbeat_microseconds_until_kill_ = INT_MAX;
        this->bTimeouted_ = false;

        this->bCrashed_ = false;
        this->crashs_ = 0;
        this->restartdelay_microseconds_ = __WATCHDOG_RE_RESTART_DELAY_DEFAULTVALUE;
        this->restartdelay_microseconds_max_ = __WATCHDOG_RE_RESTART_DELAY_DEFAULTVALUE;
        this->bAutoRestart_ = __WATCHDOG_AUTORESTART_PROCESSES_DEFAULTVALUE;
        this->bIgnoreReturnvalue_ = __WATCHDOG_IGNORE_RETURNVALUE_DEFAULTVALUE;

        this->bScheduledHeartbeatTimerReset_ = false;
        this->bScheduledStart_ = true;
        this->bScheduledStop_ = false;
        this->bScheduledRestart_ = false;
    }

    Process::~Process()
    {
        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::string time = asctime(timeinfo);

        this->logstream_ << std::endl;
        this->logstream_ << "Closed log (" << time.substr(0, time.size() - 1) << ")" << std::endl;
        this->logstream_.close();
    }

    /**
     *
     */
    void Process::started()
    {
        this->bRunning_ = true;
        this->bSuspended_ = false;

        this->bScheduledStart_ = false;
        this->bScheduledStop_ = false;
        this->bScheduledRestart_ = false;

        // Start the restart delay if the process has crashed
        if (this->bCrashed_)
        {
            this->bCrashed_ = false;
            this->restartdelay_microseconds_ = this->restartdelay_microseconds_max_;
        }

        this->resetHeartbeatTimer();
        this->bTimeouted_ = false;


        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::string time = asctime(timeinfo);

        this->logstream_ << std::endl;
        this->logstream_ << "Process started (" << time.substr(0, time.size() - 1) << ")" << std::endl;
        this->logstream_ << std::endl;
    }

    /**
     *
     */
    void Process::stoped()
    {
        this->bRunning_ = false;
        this->bSuspended_ = this->bScheduledStop_ && !this->bScheduledRestart_;

        this->bScheduledStop_ = false;
        this->bScheduledStart_ = this->bScheduledRestart_;
        this->bScheduledRestart_ = false;

        this->setHeartbeatTimer(INT_MAX);
        this->resetOutput();


        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::string time = asctime(timeinfo);

        this->logstream_ << std::endl;
        this->logstream_ << "Process stoped (" << time.substr(0, time.size() - 1) << ")" << std::endl;
        this->logstream_ << std::endl;
    }

    void Process::crashed()
    {
        this->bCrashed_ = true;
        this->crashs_++;
    }

    void Process::scheduleStart() const
    {
        // Schedule start if Process is not already running
        if (!this->bRunning_)
        {
            this->bScheduledStart_ = true;
            this->bScheduledRestart_ = true;
        }

        this->bSuspended_ = false;
        this->bScheduledStop_ = false;
    }

    void Process::scheduleStop() const
    {
        // Schedule stop if Process is currently running
        if (this->bRunning_)
            this->bScheduledStop_ = true;

        this->bSuspended_ = true;
        this->bScheduledStart_ = false;
        this->bScheduledRestart_ = false;
    }

    void Process::scheduleRestart() const
    {
        if (this->bRunning_)
        {
            this->bScheduledStop_ = true;
            this->bScheduledRestart_ = true;
            this->bScheduledStart_ = false;
        }
        else
        {
            this->bScheduledStart_ = true;
            this->bScheduledRestart_ = true;
            this->bScheduledStop_ = false;
        }

        this->bSuspended_ = false;
    }

    void Process::startLogStream(const std::string& path)
    {
        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::string time = asctime(timeinfo);

        std::string name = this->getName();

        size_t pos = name.find_last_of('/');
        if (pos != std::string::npos)
            name = name.substr(pos + 1, std::string::npos);

        std::ostringstream oss;
        oss << this->getCode();

        std::string filename;
        filename += oss.str();
        filename += "_";
        filename += name;
        filename += ".log";

        std::string totalpath = path + filename;
        this->logstream_.open(totalpath.c_str(), std::fstream::out);

        this->logstream_ << "Started log (" << time.substr(0, time.size() - 1) << ")" << std::endl;
        this->logstream_ << std::endl;
        this->logstream_ << "Process name:" << std::endl;
        this->logstream_ << " " << this->getName() << std::endl;
        this->logstream_ << std::endl;

        if (this->arguments_.size() > 1)
        {
            this->logstream_ << "Arguments:" << std::endl;
            for (size_t i = 1; i < this->arguments_.size(); ++i)
                this->logstream_ << " " << this->arguments_[i] << std::endl;
            this->logstream_ << std::endl;
        }
    }
}
}

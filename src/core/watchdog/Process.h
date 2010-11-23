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

#ifndef _Process_H__
#define _Process_H__

#include <string>
#include <vector>
#include <fstream>

namespace pixhawk
{
    namespace watchdog
    {
        /**
            @brief The Process class represents a process in the watchdog. It contains all necessary information and values.
        */
        class Process
        {
            public:
                Process();
                ~Process();

                inline void setPID(pid_t pid)                                       { this->pid_ = pid; }
                inline void setCode(uint16_t code)                                  { this->code_ = code; }
                inline void setFiledescriptor(int fd)                               { this->filedescriptor_ = fd; }
                inline void setName(const std::string& name)                        { this->name_ = name; }
                inline void setArguments(const std::vector<std::string>& arguments) { this->arguments_ = arguments; }
                inline void setOutputindentation(const std::string& indentation)    { this->outputindentation_ = indentation; }

                inline                           pid_t getPID()                  const { return this->pid_; }
                inline                        uint16_t getCode()                 const { return this->code_; }
                inline                             int getFiledescriptor()       const { return this->filedescriptor_; }
                inline              const std::string& getName()                 const { return this->name_; }
                inline const std::vector<std::string>& getArguments()            const { return this->arguments_; }
                inline              const std::string& getOutputindentation()    const { return this->outputindentation_; }
                inline                    std::string* getNamePtr()                    { return &this->name_; }
                inline       std::vector<std::string>* getArgumentsPtr()               { return &this->arguments_; }
                inline                    std::string* getOutputindentationPtr()       { return &this->outputindentation_; }

                void started();
                void stoped();
                void crashed();

                inline bool isRunning()   const { return this->bRunning_; }
                inline bool isSuspended() const { return this->bSuspended_; }
                inline bool isFinished()  const { return (!this->bRunning_ && !this->bSuspended_ && !this->bScheduledStart_); }
                inline bool isCrashed()   const { return this->bCrashed_; }
                inline bool isMuted()     const { return this->bMuted_; }

                void scheduleStart() const;
                void scheduleStop() const;
                void scheduleRestart() const;

                inline bool scheduledStart() const { return this->bScheduledStart_; }
                inline bool scheduledStop()  const { return this->bScheduledStop_; }

                inline void startHeartbeatTimer(int microseconds) { this->heartbeat_microseconds_max_ = microseconds; this->resetHeartbeatTimer(); }
                inline void tickHeartbeatTimer(int microseconds)  { this->heartbeat_microseconds_until_kill_ -= microseconds; }
                inline void setHeartbeatTimer(int microseconds)   { this->heartbeat_microseconds_until_kill_ = microseconds; }
                inline void resetHeartbeatTimer()                 { this->heartbeat_microseconds_until_kill_ = this->heartbeat_microseconds_max_; this->bScheduledHeartbeatTimerReset_ = false; }
                inline int  getHeartbeatTimer() const             { return this->heartbeat_microseconds_until_kill_; }
                inline int  getHeartbeatTimeout() const           { return this->heartbeat_microseconds_max_; }

                inline void timeout()         { this->bTimeouted_ = true; }
                inline bool timeouted() const { return this->bTimeouted_; }

                inline void scheduleHeartbeatTimerReset()  const { this->bScheduledHeartbeatTimerReset_ = true; }
                inline bool scheduledHeartbeatTimerReset() const { return this->bScheduledHeartbeatTimerReset_; }

                inline int  getCrashs() const { return this->crashs_; }

                inline void setRestartDelay(int microseconds)       { this->restartdelay_microseconds_max_ = microseconds; }
                inline void tickRestartDelayTimer(int microseconds) { if (this->restartdelay_microseconds_ > 0) { this->restartdelay_microseconds_ -= microseconds; } }
                inline int  getRestartDelayTimer()            const { return this->restartdelay_microseconds_; }

                inline void setAutoRestart(bool autorestart) { this->bAutoRestart_ = autorestart; }
                inline bool getAutoRestart() const           { return this->bAutoRestart_; }

                inline void setIgnoreReturnvalue(bool ignore) { this->bIgnoreReturnvalue_ = ignore; }
                inline bool getIgnoreReturnvalue() const      { return this->bIgnoreReturnvalue_; }

                inline void mute()              const { this->setMuted(true); }
                inline void unmute()            const { this->setMuted(false); }
                inline void setMuted(bool mute) const { this->bMuted_ = mute; }

                inline void scheduleOutput()    const { this->bHasOutput_ = true; }
                inline void resetOutput()             { this->bHasOutput_ = false; }
                inline bool hasOutput()         const { return this->bHasOutput_; }

                void startLogStream(const std::string& path);
                inline std::ofstream& getLogStream() { return this->logstream_; }

            private:
                pid_t pid_;                                  ///< The PID of the process in the system
                uint16_t code_;                              ///< The code of the process (somewhere between 0 and the max. number of processes - 1)
                int filedescriptor_;                         ///< The filedescriptor of the pipe between process and watchdog

                std::string name_;                           ///< The name of the process (the executable)
                std::vector<std::string> arguments_;         ///< The arguments passed to the process (argv[])
                std::string outputindentation_;              ///< Indentation whitespaces to allign process output

                bool bRunning_;                              ///< True if the process is running right now
                mutable bool bSuspended_;                    ///< True if the process is temporarily suspended
                mutable bool bHasOutput_;                    ///< If true, the process has passed output to the watchdog
                mutable bool bMuted_;                        ///< True if the process is muted

                int heartbeat_microseconds_max_;             ///< Number of microseconds allowed to pass between two heartbeat signals of the process
                int heartbeat_microseconds_until_kill_;      ///< Number of microseconds until the process gets killed because of a timeout
                bool bTimeouted_;                            ///< Becomes true if the process reached the timeout

                bool bCrashed_;                              ///< True if the process has stoped because of a crash
                int crashs_;                                 ///< The number of times the process crashed
                int restartdelay_microseconds_;              ///< Number of microseonds until the process is allowed to restart
                int restartdelay_microseconds_max_;          ///< Delay until the process is allowed to restart
                bool bAutoRestart_;                          ///< If true, the process will be restarted automatically after a crash
                bool bIgnoreReturnvalue_;                    ///< If true, the returnvalue of a process will be ignored and always handled as a crash

                mutable bool bScheduledHeartbeatTimerReset_; ///< If true, the timer has to be reset in the next iteration of the watchdog
                mutable bool bScheduledStart_;               ///< If true, the process will be started in the next iteration of the watchdog
                mutable bool bScheduledStop_;                ///< If true, the process will be stoped in the next iteration of the watchdog
                mutable bool bScheduledRestart_;             ///< If true, the process will be restarted in the next iteration of the watchdog

                std::ofstream logstream_;
        };
    }
}

#endif /* _Process_H__ */

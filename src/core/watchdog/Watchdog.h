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

#ifndef _Watchdog_H__
#define _Watchdog_H__

#include <boost/program_options.hpp>
#include "timer/Timer.h"
#include "Process.h"
#include "Command.h"
#include "Core.h"

// Define signals for systems which do not have
// the SIGRT signals (e.g. Darwin / Mac Os)
#ifndef SIGRTMIN
#define _POSIX_C_SOURCE
#define SIGRTMIN SIGPROF
#endif
#ifndef SIGRTMAX
#define SIGRTMAX SIGUSR2
#endif

// Time in milliseconds after which a process gets killed if it doesn't send a heartbeat signal to the watchdog
#define __WATCHDOG_TIME_TO_KILL_IN_SECONDS_DEFAULTVALUE 3000

// The time a process has to wait until restart if he crashes immediately after a previous restart
#define __WATCHDOG_RE_RESTART_DELAY_DEFAULTVALUE 1000

// If a process died or got killed, it will be restarted if this option is set to true
#define __WATCHDOG_AUTORESTART_PROCESSES_DEFAULTVALUE true

// Ignore the returnvalue of a process and always restart it
#define __WATCHDOG_IGNORE_RETURNVALUE_DEFAULTVALUE false

// If true, the watchdog exits if all processes have finished properly
#define __WATCHDOG_AUTOEXIT_DEFAULTVALUE true

// The sleeptime of the watchdog each tick in milliseconds
#define __WATCHDOG_SLEEPTIME_DEFAULTVALUE 1

// The signalnumber used for the heartbeat signal
#define __WATCHDOG_HEARTBEAT_SIGNAL SIGRTMIN+3

// The signalnumber used for output callbacks
#define __WATCHDOG_IO_SIGNAL SIGRTMIN+5

// Print status and error messages to the console
#define __WATCHDOG_VERBOSE_DEFAULTVALUE true

// If true, all processes are muted
#define __WATCHDOG_MUTE_DEFAULTVALUE false

// If true, the output of all processes is logged
#define __WATCHDOG_LOG_DEFAULTVALUE true

// If true, the log files are flushed after every line of output
#define __WATCHDOG_FLUSH_DEFAULTVALUE false

// Timeinterval between two heartbeat messages in milliseconds
#define __WATCHDOG_HEARTBEAT_INTERVAL_DEFAULTVALUE 2000

typedef struct _lcm_t lcm_t;
typedef struct _mavlink_message_t_subscription_t mavlink_message_t_subscription_t;

namespace pixhawk
{
    namespace watchdog
    {
        class Watchdog
        {
            public:
                Watchdog();
                ~Watchdog();

                static inline const Watchdog& getInstance()
                    { return (*Watchdog::instance_s); }

                void parseConfigValues(int argc, char* argv[], const Core& core);
                void parseProcesses();
                void registerSignalHandlers();
                void unregisterSignalHandlers();
                void run();

                inline const boost::program_options::variables_map& getConfigValuesMap() const
                    { return this->vm_; }

                const Process& getProcessByCode(uint16_t code) const throw(std::invalid_argument);
                const Process& getProcessByPID(pid_t pid) const throw(std::invalid_argument);
                const Process& getProcessByFiledescriptor(int fd) const throw(std::invalid_argument);

                inline const std::vector<Process*>& getProcesses() const
                    { return this->processes_; }

                inline size_t getNumProcesses() const
                    { return this->processes_.size(); }

                static inline bool isVerbose()
                    { return Watchdog::getInstance().getConfigValuesMap()["verbose"].as<bool>(); }

                inline lcm_t* getLcm() const
                    { return this->lcm_; }

                static int HEARTBEAT_SIGNAL;
                static int IO_SIGNAL;

            private:
                Process& getProcessByPID(pid_t pid) throw(std::invalid_argument);

                void startProcess(Process& process);
                void stopProcess(Process& process);
                void handleChildExit(pid_t pid, int stat_loc);

                void lcmConnect(const char* url = NULL);
                void lcmDisconnect();
                void lcmHandle();

                static std::string getLogPath();

                static std::string getClientExitDescription(int status, int signalnr, bool* normalexit = 0);
                static std::string getSignalDescription(int signalnr);

//                static void registerSigchldHandler();
                static void registerHeartbeatHandler();
                static void registerSigioHandler();
                static void unregisterSignalHandler(int signalnr);

                static std::string parseNameAndArguments(const std::string& commandline, std::string* name, std::vector<std::string>* arguments);
                static void parseAdditionalArguments(Process& process, const std::string& arguments);
                static void trimFront(std::string* line);
                static std::vector<char*> splitLines(char* text);

//                static void sigchldSignalHandler(int sig, siginfo_t* info, void* context);
                static void heartbeatSignalHandler(int sig, siginfo_t* info, void* context);
                static void sigioSignalHandler(int sig, siginfo_t* info, void* context);

                std::vector<Process*> processes_;                   ///< All processes that should be managed by this watchdog
                bool bExit_;                                        ///< If true, the program leaves the mainloop
                Timer timer_;                                       ///< A timer to measure the running time of the processes
                boost::program_options::variables_map vm_;          ///< The program options value map
                lcm_t* lcm_;                                        ///< Lcm connection
                mavlink_message_t_subscription_t* subscription_;    ///< Lcm message subscription
                Timer heartbeatTimer_;                              ///< A timer used to wait some time between two heartbeat messages


                static Watchdog* instance_s;
        };

        /**
            @brief Sends a heratbeat signal to the parent process.
        */
        inline int sendHeartbeatSignal()
        {
            return kill(getppid(), Watchdog::HEARTBEAT_SIGNAL);
        }
    }
}

#endif /* _Watchdog_H__ */

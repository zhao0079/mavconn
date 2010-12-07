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

#include "Watchdog.h"

#include <climits>
#include <iostream>
#include <fstream>
#include <sys/wait.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>

#include <mavconn.h>

std::string logPath("log/");

// Namespace shortcut
namespace config = boost::program_options;

namespace MAVCONN
{
namespace watchdog
{
    Watchdog* Watchdog::instance_s = 0;
    int Watchdog::HEARTBEAT_SIGNAL = __WATCHDOG_HEARTBEAT_SIGNAL;
    int Watchdog::IO_SIGNAL = __WATCHDOG_IO_SIGNAL;

    /**
        @brief Constructor
    */
    Watchdog::Watchdog()
    {
        assert(!Watchdog::instance_s);

        Watchdog::instance_s = this;

        this->bExit_ = false;

        this->lcm_ = 0;
        this->subscription_ = 0;

        this->lcmConnect();
    }

    /**
        @brief Destructor
    */
    Watchdog::~Watchdog()
    {
        this->lcmDisconnect();

        // Destroy the process structs
        for (unsigned int i = 0; i < this->processes_.size(); ++i)
            delete this->processes_[i];

        Watchdog::instance_s = 0;
    }

    /**
        @brief Gets the command line arguments and the values from the config file.
    */
    void Watchdog::parseConfigValues(int argc, char* argv[])
    {
        // Program options
        config::options_description proc("Start process");
        proc.add_options()
            ("process,p", config::value<std::vector<std::string> >()->composing(), "processes to load")
        ;
        config::options_description desc1("Allowed watchdog options");
        desc1.add_options()
            ("help", "produce help message")
            ("config,c", config::bool_switch()->default_value(false), "load the config-file additional to the processes on the commandline")
            ("file,f", config::value<std::string>(), "config file to load (default: processes.cfg)")
            ("netsend,n", config::bool_switch()->default_value(false), "send lcm messages over network")
            ("verbose,v", config::value<bool>()->default_value(__WATCHDOG_VERBOSE_DEFAULTVALUE), "Print status and error messages to the console")
            ("log,l", config::value<bool>()->default_value(__WATCHDOG_LOG_DEFAULTVALUE), "If true, the output of all processes is logged")
            ("flush,u", config::value<bool>()->default_value(__WATCHDOG_FLUSH_DEFAULTVALUE), "If true, the log files are flushed after every line of output")
            ("heartbeat,h", config::value<unsigned int>()->default_value(__WATCHDOG_HEARTBEAT_INTERVAL_DEFAULTVALUE), "Time in milliseconds between two heartbeat messages of the watchdog")
            ("sleeptime,s", config::value<unsigned int>()->default_value(__WATCHDOG_SLEEPTIME_DEFAULTVALUE), "The time the watchdog sleeps each tick in milliseconds")
            ("autoexit,e", config::value<bool>()->default_value(__WATCHDOG_AUTOEXIT_DEFAULTVALUE), "If true, the watchdog exits if all processes have finished properly")
        ;
        config::options_description desc2("Allowed process options");
        desc2.add_options()
            ("timeout,t", config::value<unsigned int>()->default_value(__WATCHDOG_TIME_TO_KILL_IN_SECONDS_DEFAULTVALUE), "Time in milliseconds after which a process gets killed if it doesn't send a heartbeat signal to the watchdog")
            ("autorestart,a", config::value<bool>()->default_value(__WATCHDOG_AUTORESTART_PROCESSES_DEFAULTVALUE), "If a process died or got killed, it will be restarted if this option is set to true")
            ("ignore,i", config::value<bool>()->default_value(__WATCHDOG_IGNORE_RETURNVALUE_DEFAULTVALUE), "Ignore the returnvalue if a process exits and always restart it")
            ("restartdelay,r", config::value<unsigned int>()->default_value(__WATCHDOG_RE_RESTART_DELAY_DEFAULTVALUE), "The time in milliseconds a process has to wait until it can be restarted again after a previous restart")
            ("mute,m", config::value<bool>()->default_value(__WATCHDOG_MUTE_DEFAULTVALUE), "If true, all processes are muted")
        ;
        desc1.add(desc2).add(proc);

        config::store(config::parse_command_line(argc, argv, desc1), this->vm_);

        // Load the config file if no processes were defined on the commandline (or if the --config argument is true)
        if (!this->vm_.count("process") || this->vm_["config"].as<bool>() || this->vm_.count("file"))
        {
            std::string filename = "config";

            if (this->vm_.count("file"))
                filename += this->vm_["file"].as<std::string>();
            else
                filename += "processes.cfg";

            std::filebuf file;
            file.open(filename.c_str(), std::ios::in);

            if (file.is_open())
            {
                std::istream stream(&file);
                config::store(config::parse_config_file(stream, proc), this->vm_);
            }
            else if (this->vm_.count("file"))
                std::cout << "Error: Config file \"" << filename << "\" doesn't exist." << std::endl;
        }

        config::notify(this->vm_);

        // Return --help information
        if (this->vm_.count("help"))
        {
            std::cout << desc1 << std::endl;
            exit(EXIT_SUCCESS);
        }

        // Reconnect to another lcm url if netsend is switched on
        if (this->vm_.count("netsend") && this->vm_["netsend"].as<bool>())
        {
            this->lcmDisconnect();
            this->lcmConnect("udpm://");
        }
    }

    /**
        @brief Parses the process commandline-arguments
    */
    void Watchdog::parseProcesses()
    {
        if (this->vm_.count("process"))
        {
            std::string logpath;
            if (this->vm_["log"].as<bool>())
                logpath = Watchdog::getLogPath();

            unsigned int longestName = 0;

            if (this->vm_["verbose"].as<bool>())
                std::cout << "Processes:" << std::endl;

            const std::vector<std::string>& processlist = this->vm_["process"].as<std::vector<std::string> >();
            for (unsigned int i = 0; i < processlist.size(); ++i)
            {
                if (this->vm_["verbose"].as<bool>())
                    std::cout << "  " << i << ": " << processlist[i] << std::endl;

                Process& process = (*new Process());
                this->processes_.push_back(&process);

                process.setCode(i);
                std::string additionalarguments = Watchdog::parseNameAndArguments(processlist[i], process.getNamePtr(), process.getArgumentsPtr());

                if (this->vm_.count("timeout"))
                    process.startHeartbeatTimer(this->vm_["timeout"].as<unsigned int>() * 1000);
                if (this->vm_.count("autorestart"))
                    process.setAutoRestart(this->vm_["autorestart"].as<bool>());
                if (this->vm_.count("ignore"))
                    process.setIgnoreReturnvalue(this->vm_["ignore"].as<bool>());
                if (this->vm_.count("restartdelay"))
                    process.setRestartDelay(this->vm_["restartdelay"].as<unsigned int>() * 1000);

                if (this->vm_["log"].as<bool>())
                    process.startLogStream(logpath);

                Watchdog::parseAdditionalArguments(process, additionalarguments);

                if (this->vm_["log"].as<bool>())
                {
                    process.getLogStream() << "------------------------------------------------------" << std::endl;
                    process.getLogStream() << std::endl;

                    if (this->vm_["flush"].as<bool>())
                        process.getLogStream().flush();
                }

                if (process.getName().length() > longestName)
                    longestName = process.getName().length();
            }

            for (unsigned int i = 0; i < processlist.size(); ++i)
                for (unsigned int j = this->processes_[i]->getName().length(); j < longestName; ++j)
                    this->processes_[i]->getOutputindentationPtr()->push_back(' ');
        }

        if (this->processes_.size() == 0)
            this->bExit_ = true;
    }

    /* static */ std::string Watchdog::getLogPath()
    {
        int max = 0;

        try
        {
            boost::filesystem::directory_iterator file("log");
            boost::filesystem::directory_iterator end;

            while (file != end)
            {
                if (boost::filesystem::is_directory(*file))
                {
                    std::string name = boost::filesystem::basename(*file);

                    if (name.find("watchdog") == 0)
                    {
                        size_t pos;
                        for (pos = name.size() - 1; pos < name.size(); --pos)
                            if (name[pos] < '0' || name[pos] > '9')
                                break;

                        if (pos != std::string::npos)
                        {
                            std::istringstream iss(name.substr(pos + 1));
                            int number;
                            iss >> number;

                            if (number > max)
                                max = number;
                        }
                    }
                }
                ++file;
            }
        }
        catch (...) {}

        std::string path;

        if (max == 0)
        {
            path = logPath + "watchdog1/";
        }
        else
        {
            std::ostringstream oss;
            oss << (max + 1);

            path = logPath + "watchdog" + oss.str() + "/";
        }

        boost::filesystem::create_directories(path);
        return path;
    }

    void Watchdog::run()
    {
        this->timer_.reset();
        unsigned int count = 0;

        // Mainloop
        while (!this->bExit_)
        {
            unsigned int ticktime = this->timer_.getMicroseconds();
            this->timer_.reset();
/*
            if (count % 1000 == 0)
            {
                for (unsigned int i = 0; i < this->processes_.size(); ++i)
                {
                    if (i > 0)
                        std::cout << ", ";
                    std::cout << this->processes_[i]->microseconds_until_kill_;
                }
                std::cout << std::endl;
            }
*/
            // Loop through all processes
            for (unsigned int i = 0; i < this->processes_.size(); ++i)
            {
                Process& process = *this->processes_[i];

                if (process.isRunning())
                {
                    // Heartbeat timer
                    if (process.getHeartbeatTimeout() > 0)
                    {
                        if (process.scheduledHeartbeatTimerReset())
                            process.resetHeartbeatTimer();
                        else
                            process.tickHeartbeatTimer(ticktime); // Only tick if no heartbeat signal was received to avoid false timeouts if the watchdogs iteration takes too long (ticktime > timeout)

                        // Check if the process has timed out and kill it if necessary
                        if (process.getHeartbeatTimer() < 0)
                        {
                            if (this->vm_["verbose"].as<bool>())
                                std::cout << "Process \"" << process.getName() << "\" timed out, scheduling restart." << std::endl;

                            process.scheduleRestart();
                            process.setHeartbeatTimer(INT_MAX); // Reset killtime to avoid sending multiple kill signals
                            process.timeout();
                        }
                    }

                    // Output
                    if (process.hasOutput())
                    {
                        process.resetOutput();

                        #define _TEXT_READ_LENGTH 1024
                        char text[_TEXT_READ_LENGTH];

                        ssize_t result = read(process.getFiledescriptor(), &text, _TEXT_READ_LENGTH - 1);

                        if (result == -1)
                        {
                            if (this->vm_["verbose"].as<bool>())
                                perror("read() failed");
                        }
                        else if (result > 0)
                        {
                            text[result] = '\0';

                            std::vector<char*> lines = splitLines(text);

                            for (unsigned int j = 0; j < lines.size(); ++j)
                            {
                                if (!this->vm_["mute"].as<bool>() && !process.isMuted())
                                    std::cout << "> " << process.getName() << process.getOutputindentation() << ": " << lines[j] << std::endl;

                                if (this->vm_["log"].as<bool>())
                                {
                                    process.getLogStream() << "> " << lines[j] << std::endl;

                                    if (this->vm_["flush"].as<bool>())
                                        process.getLogStream().flush();
                                }
                            }
                        }
                    }
                }

                // Restartdelay timer
                process.tickRestartDelayTimer(ticktime);

                if (process.scheduledStop())
                    this->stopProcess(process);

                if (process.scheduledStart() && process.getRestartDelayTimer() <= 0)
                    this->startProcess(process);
            }

            // Call wait() with WNOHANG to find exited processes
            while (true)
            {
                int stat_loc;
                pid_t exitedprocess = waitpid(-1, &stat_loc, WNOHANG);
                if (exitedprocess > 0)
                    this->handleChildExit(exitedprocess, stat_loc);
                else
                    break;
            }

            if (this->vm_["autoexit"].as<bool>())
            {
                // Check watchdog finish
                bool finished = true;
                for (unsigned int i = 0; i < this->processes_.size(); ++i)
                {
                    if (!this->processes_[i]->isFinished())
                    {
                        finished = false;
                        break;
                    }
                }
                if (finished)
                {
                    if (this->vm_["verbose"].as<bool>())
                        std::cout << "All processes have finished normally." << std::endl;

                    this->bExit_ = true;
                }
            }

            ++count;

            if (this->heartbeatTimer_.getMilliseconds() > this->vm_["heartbeat"].as<unsigned int>())
            {
                this->heartbeatTimer_.reset();
                sendWatchdogCommandHeartbeat(this);
            }

            this->lcmHandle();
            usleep(this->vm_["sleeptime"].as<unsigned int>() / 2 * 1000);
        }
    }

    void Watchdog::lcmConnect(const char* url)
    {
        // connect to lcm and subscribe for mavlink messages
        this->lcm_ = lcm_create(url);
        if (this->lcm_)
            this->subscription_ = mavlink_message_t_subscribe(this->lcm_, "MAVLINK", &commandHandler, this);
    }

    void Watchdog::lcmDisconnect()
    {
        // disconnect from lcm
        if (this->lcm_)
        {
            if (this->subscription_)
                mavlink_message_t_unsubscribe(this->lcm_, this->subscription_);

            lcm_destroy(this->lcm_);
        }
    }

    void Watchdog::lcmHandle()
    {
        int fileno = lcm_get_fileno(this->lcm_);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = this->vm_["sleeptime"].as<unsigned int>() / 2 * 1000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fileno, &readfds);

        select(fileno + 1, &readfds, NULL, NULL, &tv);

        if (FD_ISSET(fileno, &readfds) && errno != EINTR) // check if errno is EINTR (this happens if SIGIO interrupts select)
            lcm_handle(this->lcm_);

        errno = 0;
    }

    /**
        @brief Starts a new process.
        @param process The process-struct of the process that should be started.

        This function starts a new process by using fork() and exec(). It passes the arguments, which are stored in the
        process-struct, to the new process. In the parent process the case of an immediate exit of the child is handled.
        In the other case (the child runs) the variables of the process-struct are set accordingly.
    */
    void Watchdog::startProcess(Process& process)
    {
        // Open pipe
        int pipe_file_descriptor[2];
        if (pipe(pipe_file_descriptor))
        {
            if (this->vm_["verbose"].as<bool>())
            {
                std::cout << "pipe() failed, couldn't open pipe for \"" << process.getName() << "\": ";
                perror(NULL);
            }
        }

        // Duplicate this process
        pid_t pid = fork();

        if (pid == 0)
        {
            // We're the child

            // Prepare the pipe
            close(pipe_file_descriptor[0]);   // close the read end of the pipe
            dup2(pipe_file_descriptor[1], 1); // make 1 same as write-to end of pipe (1 is std::cout)
            dup2(pipe_file_descriptor[1], 2); // make 1 same as write-to end of pipe (2 is std::cerror)
            close(pipe_file_descriptor[1]);   // close excess file descriptor

            // Create the program arguments
            char* argv[process.getArguments().size() + 1]; // +1 for the trailing 0
            for (unsigned int i = 0; i < process.getArguments().size(); ++i)
                argv[i] = const_cast<char*>(process.getArguments()[i].c_str());
            argv[process.getArguments().size()] = 0; // set the last argument to 0

            // Create the new process
            if (execvp(process.getName().c_str(), argv))
            {
                if (this->vm_["verbose"].as<bool>())
                {
                    std::cout << "exec() failed, couldn't start \"" << process.getName() << "\": ";
                    perror(NULL);
                }
            }

            // We shoulnd't ever reach this line. If we do, exit.
            _exit(EXIT_FAILURE);
        }
        else if (pid > 0)
        {
            // We're the parent
            if (this->vm_["verbose"].as<bool>())
                std::cout << "Created child \"" << process.getName() << "\" with PID " << pid << std::endl;

            close(pipe_file_descriptor[1]);   // close the write end of the pipe

            // Prepare the pipe to get callbacks if the process sends output
            // FIXME Violates POSIX standard, works only on Linux :(. See man fnctl for POSIX-alternatives
            // Is a fix hard to achieve?
#ifdef __linux
            if (fcntl(pipe_file_descriptor[0], F_SETSIG, IO_SIGNAL) && this->vm_["verbose"].as<bool>()) // F_SETSIG defines SIGIO as the raised signal whenever the pipe gets readable
                perror("fcntl() failed (F_SETSIG)");
            if (fcntl(pipe_file_descriptor[0], F_SETOWN, getpid()) && this->vm_["verbose"].as<bool>()) // F_SETOWN defines our process as the destination of the SIGIO signals
                perror("fcntl() failed (F_SETOWN)");
            if (fcntl(pipe_file_descriptor[0], F_SETFL, O_ASYNC) && this->vm_["verbose"].as<bool>()) // F_SETFL sets the O_ASYNC flag to activate the signal sending
                perror("fcntl() failed (F_SETFL)");
#endif
/*
            // Call wait with WNOHANG to kill the child if it crashes before we could set up the signal handler
            int stat_loc;
            pid_t exitedprocess = waitpid(pid, &stat_loc, WNOHANG);
            if (exitedprocess)
            {
                this->handleChildExit(exitedprocess, stat_loc);
                return;
            }
*/
            // Set all process values
            process.started();
            process.setPID(pid);
            process.setFiledescriptor(pipe_file_descriptor[0]);

            sendWatchdogCommandProcessStatus(this, process);
        }
        else
        {
            // Fork failed
            if (this->vm_["verbose"].as<bool>())
            {
                std::cout << "fork() failed, couldn't start \"" << process.getName() << "\": ";
                perror(NULL);
            }
        }
    }

    /**
        @brief Stops a process by sending a kill signal.
    */
    void Watchdog::stopProcess(Process& process)
    {
        // Send kill signal to stop the process
        if (kill(process.getPID(), SIGKILL) && this->vm_["verbose"].as<bool>())
            perror("kill() failed");
    }

    /**
        @brief Handles the exit of a child process.
    */
    void Watchdog::handleChildExit(pid_t pid, int stat_loc)
    {
        bool normalexit = false;
        std::string description = Watchdog::getClientExitDescription(WEXITSTATUS(stat_loc), WTERMSIG(stat_loc), &normalexit);

        if (this->vm_["verbose"].as<bool>())
            std::cout << "Child exited. " << description << ". ";

        try
        {
            Process& process = this->getProcessByPID(pid);

            if (this->vm_["verbose"].as<bool>())
                std::cout << "(PID: " << process.getPID() << ", " << process.getName() << ")" << std::endl;

            if (this->vm_["log"].as<bool>())
            {
                process.getLogStream() << std::endl;
                process.getLogStream() << "Received Signal: " << description << std::endl;

                if (this->vm_["flush"].as<bool>())
                    process.getLogStream().flush();
            }

            bool scheduledStop = process.scheduledStop();

            process.stoped();

            if ((!normalexit || process.getIgnoreReturnvalue()) && !process.isSuspended())
            {
                if (!scheduledStop || process.timeouted()) // crashed or killed
                    process.crashed();

                if (process.getAutoRestart())
                    process.scheduleStart();
            }

            close(process.getFiledescriptor()); // Close the pipe
            process.setFiledescriptor(-1);

            sendWatchdogCommandProcessStatus(this, process);
        }
        catch (...)
        {
            if (this->vm_["verbose"].as<bool>())
                std::cout << "(PID: " << pid << ", unknown process)" << std::endl;
        }
    }

    /**
        @brief Handle the SIGCHLD signal which gets raised if a child process exits.

        If a child process exits (either controlled by exit(int) or unwanted if an error occurrs or a signal gets raised.
        The signal handler tries to find the reason for the exit and sets the process-struct variables accordingly to initiate a restart of the process if needed.
    */
    /* static */
/*
    void Watchdog::sigchldSignalHandler(int sig, siginfo_t* info, void* context)
    {
        if (Watchdog::isVerbose())
            std::cout << "SIGCHLD was raised. ";

        pid_t pid = -1;
        bool normalexit = false;

        if (info)
        {
            // Print error if available
            if (info->si_errno && Watchdog::isVerbose())
                perror("error in siginfo_t");

            pid = info->si_pid;

            // Wait for the child to exit
            int stat_loc;
            if (waitpid(pid, &stat_loc, 0) == -1)
            {
                if (Watchdog::isVerbose())
                {
                    perror("waitpid() failed");
                    std::cout << Watchdog::getClientExitDescription(info->si_status, info->si_status, &normalexit) << ".";
                }
                else
                    Watchdog::getClientExitDescription(info->si_status, info->si_status, &normalexit);
            }
            else
            {
                // Check the exit reason
                if (Watchdog::isVerbose())
                    std::cout << Watchdog::getClientExitDescription(WEXITSTATUS(stat_loc), WTERMSIG(stat_loc), &normalexit) << ".";
                else
                    Watchdog::getClientExitDescription(WEXITSTATUS(stat_loc), WTERMSIG(stat_loc), &normalexit);
            }
        }
        else
        {
            // No siginfo object available, use wait()
            pid = wait(NULL);
            if (Watchdog::isVerbose())
                std::cout << "unknown reason (no siginfo_t available).";
        }

        handleChildExit(pid, normalexit);
    }
*/
    /**
        @brief Handle the heartbeat signal which gets raised by every child process to tell the watchdog it's still alive.
    */
    /* static */
    void Watchdog::heartbeatSignalHandler(int sig, siginfo_t* info, void* context)
    {
        if (Watchdog::isVerbose())
            std::cout << "heartbeat signal was raised. ";

        if (info)
        {
            // Print error if available
            if (info->si_errno && Watchdog::isVerbose())
                perror("error in siginfo_t");

            try
            {
                const Process& process = Watchdog::getInstance().getProcessByPID(info->si_pid);

                if (Watchdog::isVerbose())
                    std::cout << "(PID: " << process.getPID() << ", " << process.getName() << ")" << std::endl;

                // Reset the time
                process.scheduleHeartbeatTimerReset();
            }
            catch (...)
            {
                if (Watchdog::isVerbose())
                    std::cout << "(PID: " << info->si_pid << ", unknown process)" << std::endl;
            }
        }
        else
        {
            if (Watchdog::isVerbose())
                std::cout << "unknown process (no siginfo_t available)" << std::endl;
        }
    }

    /**
        @brief Handlers the SIGIO signal which gets raised whenever a child process sent a message to the pipe.
    */
    /* static */
    void Watchdog::sigioSignalHandler(int sig, siginfo_t* info, void* context)
    {
        if (info)
        {
            // Print error if available
            if (info->si_errno && Watchdog::isVerbose())
                perror("sigio signal was raised. error in siginfo_t");

            try
            {
#ifdef __linux
                const Process& process = Watchdog::getInstance().getProcessByFiledescriptor(info->si_fd);
#else
                const Process& process = Watchdog::getInstance().getProcessByPID(info->si_pid);
#endif
                process.scheduleOutput();
            }
            catch (...)
            {
                if (Watchdog::isVerbose())
                {
#ifdef __linux
                	std::cout << "sigio signal was raised, but process is unknown (FD: " << info->si_fd << ")" << std::endl;;
#else
                	std::cout << "sigio signal was raised, but process is unknown (PID: " << info->si_pid << ")" << std::endl;;
#endif
                }

            }
        }
        else if (Watchdog::isVerbose())
            std::cout << "sigio signal was raised, but no siginfo_t available" << std::endl;;

    }

    /**
        @brief Registers all signal handlers.
    */
    void Watchdog::registerSignalHandlers()
    {
//        Watchdog::registerSigchldHandler();
        Watchdog::registerHeartbeatHandler();
        Watchdog::registerSigioHandler();
    }

    /**
        @brief Unregisters all signal handlers.
    */
    void Watchdog::unregisterSignalHandlers()
    {
//        Watchdog::unregisterSignalHandler(SIGCHLD);
        Watchdog::unregisterSignalHandler(HEARTBEAT_SIGNAL);
        Watchdog::unregisterSignalHandler(IO_SIGNAL);
    }

    /**
        @brief Registers a handler for the SIGCHLD signal.

        This signal handler is used to get a notification if a child process exits.
    */
    /* static */
/*
    void Watchdog::registerSigchldHandler()
    {
        struct sigaction sa;

        sa.sa_sigaction = &Watchdog::sigchldSignalHandler;

        sa.sa_flags = 0;
        sa.sa_flags |= SA_SIGINFO; // Set this to get siginfo_t in the signal handler
        sa.sa_flags |= SA_RESTART; // Restart interruptible functions after signal handling
        sa.sa_flags |= SA_NODEFER; // Don't block other signals of this type
//        sa.sa_flags |= SA_NOCLDWAIT; // Set this so we don't have to call wait() to remove the child after it exited

//        sigaddset(&sa.sa_mask, SIGCHLD);
//        sigaddset(&sa.sa_mask, SIGUSR1);
//        sigaddset(&sa.sa_mask, SIGUSR2);
//        sigaddset(&sa.sa_mask, SIGIO);

        if (sigaction(SIGCHLD, &sa, NULL) && Watchdog::isVerbose())
            perror("sigaction() failed, couldn'd add signal handler for SIGCHLD.");
    }
*/
    /**
        @brief Registers a handler for the heartbeat signal.

        This signal handler is used to get a notification if a child raises the heartbeat signal to tell
        the watchdog it's still alive.
    */
    /* static */
    void Watchdog::registerHeartbeatHandler()
    {
        struct sigaction sa;

        sa.sa_sigaction = &Watchdog::heartbeatSignalHandler;

        sa.sa_flags = 0;
        sa.sa_flags |= SA_SIGINFO; // Set this to get siginfo_t in the signal handler
        sa.sa_flags |= SA_RESTART; // Restart interruptible functions after signal handling
        sa.sa_flags |= SA_NODEFER; // Don't block other signals of this type

        if (sigaction(HEARTBEAT_SIGNAL, &sa, NULL) && Watchdog::isVerbose())
            perror("sigaction() failed, couldn'd add signal handler for heartbeat signal.");
    }

    /**
        @brief Registers a handler for the SIGIO signal.

        This signal handler is used to get a notification whenever a message was sent from
        one of the child processes to the output pipe. The watchdog can then read the data
        from the filedescriptor.
    */
    /* static */
    void Watchdog::registerSigioHandler()
    {
        struct sigaction sa;

        sa.sa_sigaction = &Watchdog::sigioSignalHandler;

        sa.sa_flags = 0;
        sa.sa_flags |= SA_SIGINFO; // Set this to get siginfo_t in the signal handler
        sa.sa_flags |= SA_RESTART; // Restart interruptible functions after signal handling
        sa.sa_flags |= SA_NODEFER; // Don't block other signals of this type

        if (sigaction(IO_SIGNAL, &sa, NULL) && Watchdog::isVerbose())
            perror("sigaction() failed, couldn'd add signal handler for SIGIO.");
    }

    /**
        @brief Unregisters a signal handler for a given signal number by reapplying the standard handler.
    */
    /* static */
    void Watchdog::unregisterSignalHandler(int signalnr)
    {
        struct sigaction sa;

        sa.sa_handler = SIG_DFL;
        sa.sa_flags = 0;

        if (sigaction(signalnr, &sa, NULL) && Watchdog::isVerbose())
            perror((std::string("sigaction() failed, couldn'd unregister signal handler for signal ") + Watchdog::getSignalDescription(signalnr)).c_str());
    }

    /**
        @brief Returns a string which explains the reason why a client exited.

        @param status Contains the exit status of the process.
        @param signalnr The signalnumber that caused the client to exit.
        @param normalexit Optional parameter. If set, the function writes true in this variable, if the client exited normaly (with exit(0)).
    */
    /* static */
    std::string Watchdog::getClientExitDescription(int status, int signalnr, bool* normalexit)
    {
        if (normalexit)
            *normalexit = false;

        // Get the description of the signal
        std::string signaldescription;
        if (signalnr)
            signaldescription = Watchdog::getSignalDescription(signalnr);

        if (signaldescription != "")
        {
            // A signal description exists - the process was killed by a signal
            return "Child error signal: " + signaldescription;
        }
        else if (status == 0)
        {
            // No signal description exists and the exit status is 0 - everythin is fine
            if (normalexit)
                *normalexit = true;
            return "Child exited normally";
        }
        else
        {
            // No signal description exists and the exit status is not zero - the process exited with an error
            std::ostringstream oss;
            oss << status;

            return "Child exited with return value " + oss.str();
        }
    }

    /**
        @brief Returns the description for a given signalnumber.
    */
    /* static */
    std::string Watchdog::getSignalDescription(int signalnr)
    {
        switch (signalnr)
        {
            case SIGHUP:     return "SIGHUP - Hangup"; break;
            case SIGINT:     return "SIGINT - Interrupt"; break;
            case SIGQUIT:    return "SIGQUIT - Quit"; break;
            case SIGILL:     return "SIGILL - Illegal instruction"; break;
            case SIGTRAP:    return "SIGTRAP - Trace trap"; break;
            case SIGABRT:    return "SIGABRT - Abort"; break;
            case SIGBUS:     return "SIGBUS - BUS error"; break;
            case SIGFPE:     return "SIGFPE - Floating-point exception"; break;
            case SIGKILL:    return "SIGKILL - Kill"; break;
            case SIGUSR1:    return "SIGUSR1 - User-defined signal 1"; break;
            case SIGSEGV:    return "SIGSEGV - Segmentation violation"; break;
            case SIGUSR2:    return "SIGUSR2 - User-defined signal 2"; break;
            case SIGPIPE:    return "SIGPIPE - Broken pipe"; break;
            case SIGALRM:    return "SIGALRM - Alarm clock"; break;
            case SIGTERM:    return "SIGTERM - Termination"; break;
#ifdef SIGSTKFLT
            case SIGSTKFLT:  return "SIGSTKFLT - Stack fault."; break;
#endif
            case SIGCHLD:    return "SIGCHLD - Child status has changed"; break;
            case SIGCONT:    return "SIGCONT - Continue"; break;
            case SIGSTOP:    return "SIGSTOP - Stop"; break;
            case SIGTSTP:    return "SIGTSTP - Keyboard stop"; break;
            case SIGTTIN:    return "SIGTTIN - Background read from tty"; break;
            case SIGTTOU:    return "SIGTTOU - Background write to tty"; break;
            case SIGURG:     return "SIGURG - Urgent condition on socket"; break;
            case SIGXCPU:    return "SIGXCPU - CPU limit exceeded"; break;
            case SIGXFSZ:    return "SIGXFSZ - File size limit exceeded"; break;
            case SIGVTALRM:  return "SIGVTALRM - Virtual alarm clock"; break;
            case SIGPROF:    return "SIGPROF - Profiling alarm clock"; break;
            case SIGWINCH:   return "SIGWINCH - Window size change"; break;
            case SIGIO:      return "SIGIO - I/O now possible"; break;
#ifdef SIGPWR
            case SIGPWR:     return "SIGPWR - Power failure restart"; break;
#endif
            case SIGSYS:     return "SIGSYS - Bad system call"; break;
            default:
            {
                if (signalnr >= SIGRTMIN && signalnr <= SIGRTMAX)
                {
                    std::ostringstream oss;
                    oss << (signalnr - SIGRTMIN + 1);
                    return "SIGRT" + oss.str();
                }
            };
        }
        return "unknown";
    }

    /**
        @brief Helper function, parses program name and commandline arguments from a string.

        @param _commandline The input string
        @param name The name will be written into this variable
        @param arguments The arguments will be added to this vector

        @return A string containing the additional arguments. See @ref parseAdditionalArguments.

        The input string contains the whole commandline string. The first token (non-whitespace characters
        followed by a whitespace) defines the name, all following tokens are treated as program arguments.
    */
    /* static */
    std::string Watchdog::parseNameAndArguments(const std::string& _commandline, std::string* name, std::vector<std::string>* arguments)
    {
        std::string output;
        std::string commandline = _commandline;
        Watchdog::trimFront(&commandline);

        // The first token is the name
        size_t pos = commandline.find_first_of(' ');
        *name = commandline.substr(0, pos);

        {
            size_t pos = (*name).find_last_of(':');
            if (pos != std::string::npos)
            {
                // There are additional arguments
                output = (*name).substr(0, pos);
                *name = (*name).substr(pos + 1, std::string::npos);
            }
        }

        arguments->push_back(*name); // Note: The name is the first program argument, therefore it's also added to the argument vector

        // Loop until all tokens were added to the vector
        while (pos != std::string::npos)
        {
            size_t oldpos = pos + 1;
            pos = commandline.find_first_of(' ', oldpos);

            arguments->push_back(commandline.substr(oldpos, pos - oldpos));
        }

        return output;
    }

    /**
        @brief Helper function, parses additional arguments which are passed to the watchdog in the process string.

        @param process The process to which the arguments belong
        @param _arguments The string with additional arguments

        You can add additional arguments to the watchdog for every process by using the syntax
        -p "arg1:arg2:...:argN:processpath/processname -commandlinearg1 ...". This function parses the
        first series of arguments which aren't passed to the process but rather define a configuration.
        Read the documentation for a list of available arguments.
    */
    /* static */
    void Watchdog::parseAdditionalArguments(Process& process, const std::string& _arguments)
    {
        std::string arguments = ":" + _arguments;
        size_t pos = 0;

        if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>())
            process.getLogStream() << "Additional arguments:" << std::endl;

        while (pos != std::string::npos)
        {
            size_t oldpos = pos + 1;
            pos = arguments.find_first_of(':', oldpos);

            std::string argument = arguments.substr(oldpos, pos - oldpos);

            if (argument.size() < 1)
                continue;

            if (argument[0] == 's')
            {
                // autostart
                if (argument.size() >= 2 && argument[1] == '0')
                {
                    // autostart deactivated
                    process.scheduleStop();

                    if (Watchdog::isVerbose())                                          std::cout << "    Autostart deactivated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Autostart deactivated" << std::endl;
                }
                else
                {
                    // autostart activated

                    if (Watchdog::isVerbose())                                          std::cout << "    Autostart activated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Autostart activated" << std::endl;
                }
            }
            else if (argument[0] == 't')
            {
                // heartbeat timeout
                if (argument.size() >= 2)
                {
                    std::istringstream iss(argument.substr(1, std::string::npos));
                    float timeout = 0;
                    iss >> timeout;
                    process.startHeartbeatTimer(timeout * 1000);

                    if (Watchdog::isVerbose())                                          std::cout << "    Heartbeat timeout: " << timeout << " milliseconds" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Heartbeat timeout: " << timeout << " milliseconds" << std::endl;
                }
            }
            else if (argument[0] == 'a')
            {
                // autorestart
                if (argument.size() >= 2 && argument[1] == '0')
                {
                    // autorestart deactivated
                    process.setAutoRestart(false);

                    if (Watchdog::isVerbose())                                          std::cout << "    Autorestart deactivated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Autorestart deactivated" << std::endl;
                }
                else
                {
                    // autorestart activated
                    process.setAutoRestart(true);

                    if (Watchdog::isVerbose())                                          std::cout << "    Autorestart activated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Autorestart activated" << std::endl;
                }
            }
            else if (argument[0] == 'i')
            {
                // ignore
                if (argument.size() >= 2 && argument[1] == '0')
                {
                    // ignore returnvalue deactivated
                    process.setIgnoreReturnvalue(false);

                    if (Watchdog::isVerbose())                                          std::cout << "    Ignore returnvalue deactivated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Ignore returnvalue deactivated" << std::endl;
                }
                else
                {
                    // ignore returnvalue activated
                    process.setIgnoreReturnvalue(true);

                    if (Watchdog::isVerbose())                                          std::cout << "    Ignore returnvalue activated" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Ignore returnvalue activated" << std::endl;
                }
            }
            else if (argument[0] == 'r')
            {
                // restart delay
                if (argument.size() >= 2)
                {
                    std::istringstream iss(argument.substr(1, std::string::npos));
                    float delay = 0;
                    iss >> delay;
                    process.setRestartDelay(delay * 1000);

                    if (Watchdog::isVerbose())                                          std::cout << "    Restart delay: " << delay << " milliseconds" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Restart delay: " << delay << " milliseconds" << std::endl;
                }
            }
            else if (argument[0] == 'm')
            {
                // mute
                if (argument.size() >= 2 && argument[1] == '0')
                {
                    // not muted
                    process.unmute();

                    if (Watchdog::isVerbose())                                          std::cout << "    Not muted" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Not muted" << std::endl;
                }
                else
                {
                    // muted
                    process.mute();

                    if (Watchdog::isVerbose())                                          std::cout << "    Muted" << std::endl;
                    if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>()) process.getLogStream() << " Muted" << std::endl;
                }
            }
        }

        if (Watchdog::getInstance().getConfigValuesMap()["log"].as<bool>())
            process.getLogStream() << std::endl;
    }

    /**
        @brief Removes whitespaces from the begin of a string.
    */
    /* static */
    void Watchdog::trimFront(std::string* _line)
    {
        std::string& line = *_line;

        while (line.length() > 0 && line[0] == ' ')
            line.erase(0, 1);
    }

    /**
        @brief Returns a vector with pointers to each start of a line in a text. '\\n' characters are replaced by '\\0' to allow line-by-line output.

        @param text The input text (with multiple lines).
        @return A vector with pointers to the start character of each line
    */
    /* static */
    std::vector<char*> Watchdog::splitLines(char* text)
    {
        std::vector<char*> output;
        output.push_back(&text[0]);

        unsigned int i = 0;
        while (text[i] != '\0')
        {
            if (text[i] == '\n')
            {
                text[i] = '\0';

                if (text[i+1] != '\0')
                    output.push_back(&text[i+1]);
            }

            ++i;
        }

        return output;
    }

    const Process& Watchdog::getProcessByCode(uint16_t code) const throw(std::invalid_argument)
    {
        if (code < this->processes_.size())
            return (*this->processes_[code]);
        else
            throw (std::invalid_argument("code is out of range"));
    }

    const Process& Watchdog::getProcessByPID(pid_t pid) const throw(std::invalid_argument)
    {
        for (size_t i = 0; i < this->processes_.size(); ++i)
        {
            if (this->processes_[i]->getPID() == pid)
                return (*this->processes_[i]);
        }
        throw (std::invalid_argument("no process with this PID"));
    }

    Process& Watchdog::getProcessByPID(pid_t pid) throw(std::invalid_argument)
    {
        for (size_t i = 0; i < this->processes_.size(); ++i)
        {
            if (this->processes_[i]->getPID() == pid)
                return (*this->processes_[i]);
        }
        throw (std::invalid_argument("no process with this PID"));
    }

    const Process& Watchdog::getProcessByFiledescriptor(int fd) const throw(std::invalid_argument)
    {
        for (size_t i = 0; i < this->processes_.size(); ++i)
        {
            if (this->processes_[i]->getFiledescriptor() == fd)
                return (*this->processes_[i]);
        }
        throw (std::invalid_argument("no process with this filedescriptor"));
    }
}
}

/*
//        if (setpriority(PRIO_PROCESS, newprocess, 5) && vm["verbose"].as<bool>())
//            perror("setpriority() failed");
*/

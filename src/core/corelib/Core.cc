/*
 *   ORXONOX - the hottest 3D action shooter ever to exist
 *                    > www.orxonox.net <
 *
 *
 *   License notice:
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; either version 2
 *   of the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *   Author:
 *      Fabian 'x3n' Landau
 *      Reto Grieder
 *   Co-authors:
 *      ...
 *
 */

// 5/10/2009: Adapted to the Pixhawk Project by Fabian Landau

/**
    @file
    @brief Implementation of the Core class.
*/

#include "Core.h"

#include <cassert>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <boost/filesystem.hpp>

#ifdef PIXHAWK_PLATFORM_WINDOWS
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#elif defined(__APPLE__) & defined(__MACH__)
#  include <sys/param.h>
#  include <mach-o/dyld.h>
#else /* Linux */
#  include <sys/types.h>
#  include <unistd.h>
#endif

#include "MAVCONNPathConfig.h"
#include "Debug.h"
#include "Exception.h"
#include "SignalHandler.h"

namespace pixhawk
{
    //! Path to the parent directory of the ones above if program was installed with relativ pahts
    static boost::filesystem::path rootPath_g;
    static boost::filesystem::path executablePath_g;            //!< Path to the executable
    static boost::filesystem::path mediaPath_g;                 //!< Path to the media file folder
    static boost::filesystem::path configPath_g;                //!< Path to the config file folder
    static boost::filesystem::path logPath_g;                   //!< Path to the log file folder
    static boost::filesystem::path capturePath_g;               //!< Path to the capture file folder

    Core* Core::singletonRef_s  = 0;

    Core::Core(const std::string& appname) : appname_(appname)
    {
        assert(Core::singletonRef_s == 0);
        Core::singletonRef_s = this;

        this->signalHandler_ = 0;
        this->softDebugLevel_ = 0;
        this->softDebugLevelConsole_ = 0;
        this->softDebugLevelLogfile_ = 0;
        this->bInitializeRandomNumberGenerator_ = false;
        this->isDevBuild_ = false;
        this->loaded_ = false;

        this->initialise();
    }

    void Core::initialise()
    {
        // Determine and set the location of the executable
        this->setExecutablePath();

        // Determine whether we have an installed or a binary dir run
        // The latter occurs when simply running from the build directory
        this->checkDevBuild();

        // Make sure the directories we write in exist or else make them
        this->createDirectories();

        // create a signal handler (only active for linux)
        // This call is placed as soon as possible, but after the directories are set
        this->signalHandler_ = new SignalHandler();
        this->signalHandler_->doCatch(executablePath_g.string(), Core::getLogPathString(), this->appname_ + "_crash.log");

        // Set the correct log path. Before this call, /tmp (Unix) or %TEMP% was used
//        OutputHandler::getOutStream().setLogPath(Core::getLogPathString()); // TODO

        this->loaded_ = true;
    }

    /**
        @brief Sets the bool to true to avoid static functions accessing a deleted object.
    */
    Core::~Core()
    {
        this->loaded_ = false;

        delete this->signalHandler_;

        assert(Core::singletonRef_s);
        Core::singletonRef_s = 0;
    }

    /* *
        @brief Returns the softDebugLevel for the given device (returns a default-value if the class ist right about to be created).
        @param device The device
        @return The softDebugLevel
    *//*
    int Core::getSoftDebugLevel(OutputHandler::OutputDevice device)
    {
        switch (device)
        {
        case OutputHandler::LD_All:
            return Core::getInstance().softDebugLevel_;
        case OutputHandler::LD_Console:
            return Core::getInstance().softDebugLevelConsole_;
        case OutputHandler::LD_Logfile:
            return Core::getInstance().softDebugLevelLogfile_;
        default:
            assert(0);
            return 2;
        }
    }
*/
     /* *
        @brief Sets the softDebugLevel for the given device. Please use this only temporary and restore the value afterwards, as it overrides the configured value.
        @param device The device
        @param level The level
     *//*
     void Core::setSoftDebugLevel(OutputHandler::OutputDevice device, int level)
     {
        if (device == OutputHandler::LD_All)
            Core::getInstance().softDebugLevel_ = level;
        else if (device == OutputHandler::LD_Console)
            Core::getInstance().softDebugLevelConsole_ = level;
        else if (device == OutputHandler::LD_Logfile)
            Core::getInstance().softDebugLevelLogfile_ = level;

        OutputHandler::setSoftDebugLevel(device, level);
     }
*/
    /*static*/ const boost::filesystem::path& Core::getMediaPath()
    {
        return mediaPath_g;
    }
    /*static*/ std::string Core::getMediaPathString()
    {
        return mediaPath_g.string() + '/';
    }

    /*static*/ const boost::filesystem::path& Core::getConfigPath()
    {
        return configPath_g;
    }
    /*static*/ std::string Core::getConfigPathString()
    {
        return configPath_g.string() + '/';
    }

    /*static*/ const boost::filesystem::path& Core::getCapturePath()
    {
        return capturePath_g;
    }
    /*static*/ std::string Core::getCapturePathString()
    {
        return capturePath_g.string() + '/';
    }

    /*static*/ const boost::filesystem::path& Core::getLogPath()
    {
        return logPath_g;
    }
    /*static*/ std::string Core::getLogPathString()
    {
        return logPath_g.string() + '/';
    }

    void Core::initializeRandomNumberGenerator()
    {
        static bool bInitialized = false;
        if (!bInitialized && this->bInitializeRandomNumberGenerator_)
        {
            srand(time(0));
            rand();
            bInitialized = true;
        }
    }

    /**
    @brief
        Compares the executable path with the working directory
    */
    void Core::setExecutablePath()
    {
#ifdef PIXHAWK_PLATFORM_WINDOWS
        // get executable module
        TCHAR buffer[1024];
        if (GetModuleFileName(NULL, buffer, 1024) == 0)
            ThrowException(General, "Could not retrieve executable path.");

#elif defined(PIXHAWK_PLATFORM_APPLE)
        char buffer[1024];
        unsigned int path_len = 1023;
        if (_NSGetExecutablePath(buffer, &path_len))
            ThrowException(General, "Could not retrieve executable path.");

#else /* Linux */
        /* written by Nicolai Haehnle <prefect_@gmx.net> */

        /* Get our PID and build the name of the link in /proc */
        char linkname[64]; /* /proc/<pid>/exe */
        if (snprintf(linkname, sizeof(linkname), "/proc/%i/exe", getpid()) < 0)
        {
            /* This should only happen on large word systems. I'm not sure
               what the proper response is here.
               Since it really is an assert-like condition, aborting the
               program seems to be in order. */
            assert(false);
        }

        /* Now read the symbolic link */
        char buffer[1024];
        int ret;
        ret = readlink(linkname, buffer, 1024);
        /* In case of an error, leave the handling up to the caller */
        if (ret == -1)
            ThrowException(General, "Could not retrieve executable path.");

        /* Ensure proper NUL termination */
        buffer[ret] = 0;
#endif

        executablePath_g = boost::filesystem::path(buffer);
#ifndef PIXHAWK_PLATFORM_APPLE
        executablePath_g = executablePath_g.branch_path(); // remove executable name
#endif
    }

    /**
    @brief
        Checks for "pixhawk_dev_build.keep_me" in the executable diretory.
        If found it means that this is not an installed run, hence we
        don't write the logs and config files to ~/.pixhawk
    */
    void Core::checkDevBuild()
    {
        if (boost::filesystem::exists(executablePath_g / "pixhawk_dev_build.keep_me"))
        {
//            COUT(1) << "Running from the build tree." << std::endl;
            Core::isDevBuild_ = true;
            mediaPath_g   = PIXHAWK_MEDIA_DEV_PATH;
            configPath_g  = PIXHAWK_CONFIG_DEV_PATH;
            logPath_g     = PIXHAWK_LOG_DEV_PATH;
            capturePath_g = PIXHAWK_CAPTURE_DEV_PATH;
        }
        else
        {
#ifdef INSTALL_COPYABLE // --> relative paths
            // Also set the root path
            boost::filesystem::path relativeExecutablePath(PIXHAWK_RUNTIME_INSTALL_PATH);
            rootPath_g = executablePath_g;
            while (!boost::filesystem::equivalent(rootPath_g / relativeExecutablePath, executablePath_g) && !rootPath_g.empty())
                rootPath_g = rootPath_g.branch_path();
            if (rootPath_g.empty())
                ThrowException(General, "Could not derive a root directory. Might the binary installation directory contain '..' when taken relative to the installation prefix path?");

            // Using paths relative to the install prefix, complete them
            mediaPath_g      = rootPath_g / PIXHAWK_MEDIA_INSTALL_PATH;
            configPath_g     = rootPath_g / PIXHAWK_CONFIG_INSTALL_PATH;
            logPath_g        = rootPath_g / PIXHAWK_LOG_INSTALL_PATH;
            capturePath_g    = rootPath_g / PIXHAWK_CAPTURE_INSTALL_PATH;
#else
            // There is no root path, so don't set it at all

            // Get user directory
#  ifdef PIXHAWK_PLATFORM_UNIX /* Apple? */
            char* userDataPathPtr(getenv("HOME"));
#  else
            char* userDataPathPtr(getenv("APPDATA"));
#  endif
            if (userDataPathPtr == NULL)
                ThrowException(General, "Could not retrieve user data path.");
            boost::filesystem::path userDataPath(userDataPathPtr);
            userDataPath /= ".pixhawk";

            mediaPath_g      = userDataPath / PIXHAWK_MEDIA_INSTALL_PATH;
            configPath_g     = userDataPath / PIXHAWK_CONFIG_INSTALL_PATH;
            logPath_g        = userDataPath / PIXHAWK_LOG_INSTALL_PATH;
            capturePath_g    = userDataPath / PIXHAWK_CAPTURE_INSTALL_PATH;
#endif
        }
    }

    /*
    @brief
        Checks for the log and the config directory and creates them
        if necessary. Otherwise me might have problems opening those files.
    */
    void Core::createDirectories()
    {
        std::vector<std::pair<boost::filesystem::path, std::string> > directories;
        directories.push_back(std::pair<boost::filesystem::path, std::string>
            (boost::filesystem::path(mediaPath_g), "media"));
        directories.push_back(std::pair<boost::filesystem::path, std::string>
            (boost::filesystem::path(configPath_g), "config"));
        directories.push_back(std::pair<boost::filesystem::path, std::string>
            (boost::filesystem::path(logPath_g),    "log"));
        directories.push_back(std::pair<boost::filesystem::path, std::string>
            (boost::filesystem::path(capturePath_g),    "capture"));

        for (std::vector<std::pair<boost::filesystem::path, std::string> >::iterator it = directories.begin();
            it != directories.end(); ++it)
        {
            if (boost::filesystem::exists(it->first) && !boost::filesystem::is_directory(it->first))
            {
                ThrowException(General, std::string("The ") + it->second + " directory has been preoccupied by a file! \
                                         Please remove " + it->first.string());
            }
            if (boost::filesystem::create_directories(it->first)) // function may not return true at all (bug?)
            {
                COUT(4) << "Created " << it->second << " directory" << std::endl;
            }
        }
    }
}

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

// 5/10/2009: Adapted to the MAVCONN Project by Fabian Landau

/**
    @file
    @brief Declaration of the Core class.

    The Core class is a singleton, only used to configure some variables
    in the core through the config-file.
*/

#ifndef _Core_H__
#define _Core_H__

#include "MAVCONNConfig.h"

#include <cassert>
#include <string>
//#include "util/OutputHandler.h" // TODO

// boost::filesystem header has quite a large tail, use forward declaration
namespace boost
{
    namespace filesystem
    {
        struct path_traits;
        template<class String, class Traits> class basic_path;
        typedef basic_path< std::string, path_traits> path;
    }
}

namespace MAVCONN
{
    class SignalHandler; // forward declaration

    //! The Core class is a singleton, only used to configure some config-values.
    class Core
    {
        public:
            Core(const std::string& appname);
            ~Core();

            static Core& getInstance() { assert(Core::singletonRef_s); return *Core::singletonRef_s; }

//            static int   getSoftDebugLevel(OutputHandler::OutputDevice device = OutputHandler::LD_All);
//            static void  setSoftDebugLevel(OutputHandler::OutputDevice device, int level);

            static const boost::filesystem::path& getMediaPath();
            static std::string getMediaPathString();

            static const boost::filesystem::path& getConfigPath();
            static std::string getConfigPathString();

            static const boost::filesystem::path& getLogPath();
            static std::string getLogPathString();

            static const boost::filesystem::path& getCapturePath();
            static std::string getCapturePathString();

        private:
            Core(const Core&);

            void initialise();

            void checkDevBuild();
            void setExecutablePath();
            void createDirectories();

            void initializeRandomNumberGenerator();

            // Singletons
            SignalHandler*        signalHandler_;

            std::string appname_;
            int softDebugLevel_;                            //!< The debug level
            int softDebugLevelConsole_;                     //!< The debug level for the console
            int softDebugLevelLogfile_;                     //!< The debug level for the logfile
            bool bInitializeRandomNumberGenerator_;         //!< If true, srand(time(0)) is called
            bool isDevBuild_;                               //!< True for builds in the build directory (not installed)
            bool loaded_;                                   //!< Only true if constructor was interrupted

            static Core* singletonRef_s;
    };
}

#endif /* _Core_H__ */

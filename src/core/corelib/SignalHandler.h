/*======================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  @author Fabian Landau <mavteam@student.ethz.ch>
  @author Christoph Renner
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

/**
    @file
    @brief Declaration of the SignalHandler class.
*/

#ifndef _SignalHandler_H__
#define _SignalHandler_H__

#include "MAVCONNConfig.h"

#include <cassert>
#include <list>
#include <string>

namespace MAVCONN
{
    typedef int (*SignalCallback)( void * someData );
}

#ifdef MAVCONN_PLATFORM_LINUX
#include <signal.h>

namespace MAVCONN
{
    struct SignalRec
    {
        int signal;
        sig_t handler;
    };

    struct SignalCallbackRec
    {
        SignalCallback cb;
        void * someData;
    };


    typedef std::list<SignalRec> SignalRecList;
    typedef std::list<SignalCallbackRec> SignalCallbackList;

    class SignalHandler
    {
        public:
            SignalHandler()  { assert(SignalHandler::singletonRef_s == 0); SignalHandler::singletonRef_s = this; }
            ~SignalHandler() { assert(SignalHandler::singletonRef_s != 0); SignalHandler::singletonRef_s = NULL; }
            inline static SignalHandler& getInstance() { assert(SignalHandler::singletonRef_s); return *SignalHandler::singletonRef_s; }

            void registerCallback( SignalCallback cb, void * someData );

            void doCatch( const std::string & appName, const std::string & filepath, const std::string & filename );
            void dontCatch();

        private:
            static void sigHandler( int sig );

            void catchSignal( int sig );
            SignalRecList sigRecList;

            SignalCallbackList callbackList;

            static SignalHandler* singletonRef_s;

            std::string appName;
            std::string filepath;
            std::string filename;
    };
}

#else /* MAVCONN_PLATFORM_LINUX */

namespace MAVCONN
{
    class SignalHandler
    {
        public:
            SignalHandler()  { assert(SignalHandler::singletonRef_s == 0); SignalHandler::singletonRef_s = this; }
            ~SignalHandler() { assert(SignalHandler::singletonRef_s != 0); SignalHandler::singletonRef_s = 0; }
            inline static SignalHandler& getInstance() { assert(SignalHandler::singletonRef_s); return *SignalHandler::singletonRef_s; }
            void doCatch( const std::string & appName, const std::string & filepath, const std::string & filename ) {}
            void dontCatch() {}
            void registerCallback( SignalCallback cb, void * someData ) {}

        private:
            static SignalHandler* singletonRef_s;
    };
}

#endif /* MAVCONN_PLATFORM_LINUX */

#endif /* _SignalHandler_H__ */

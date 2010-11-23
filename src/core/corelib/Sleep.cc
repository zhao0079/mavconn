/*======================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

Original Authors:
  @author Fabian Landau <mavteam@student.ethz.ch>
  @author Reto Grieder
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

/**
    @file
    @brief Implementation of three sleep functions.
*/

#include "Sleep.h"
#include "Debug.h"

#ifdef PIXHAWK_PLATFORM_WINDOWS
#include <windows.h>

namespace orxonox
{
    void usleep(unsigned long microseconds)
    {
        if (microseconds < 1000)
            COUT(2) << "Warning: Windows can not sleep less than 1ms, ignoring" << std::endl;
        Sleep(microseconds / 1000);
    }

    void msleep(unsigned long milliseconds)
    {
        Sleep(milliseconds);
    }

    void sleep(unsigned long seconds)
    {
        Sleep(seconds * 1000);
    }
}

#else /* Linux/Apple */
#include <unistd.h>

namespace pixhawk
{
    void usleep(unsigned long usec)
    {
        ::usleep(usec);
    }
    void msleep(unsigned long msec)
    {
        ::usleep(msec * 1000);
    }
    void sleep(unsigned long sec)
    {
        ::usleep(sec * 1000000);
    }
}

#endif

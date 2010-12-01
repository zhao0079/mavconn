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

#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <sys/time.h>
#include <ctime>

namespace MAVCONN
{
    class Clock
    {
        public:
            Clock();
            ~Clock();
            unsigned long getMilliseconds();
            unsigned long getMicroseconds();
            unsigned long getOffset();
            void addOffset(unsigned long off);
        private:
            unsigned long offset;
    };
}

#endif /* _CLOCK_H_ */

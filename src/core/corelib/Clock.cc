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

#include "Clock.h"

namespace pixhawk{

Clock::Clock(){}

Clock::~Clock(){}

unsigned long Clock::getMilliseconds()
{
    struct timeval now;
	gettimeofday(&now, NULL);
    return now.tv_sec*1000 + now.tv_usec/1000 + offset/1000;
}

unsigned long Clock::getMicroseconds()
{
    struct timeval now;
	gettimeofday(&now, NULL);
	return now.tv_sec*1000000 + now.tv_usec + offset;
}

unsigned long Clock::getOffset()
{
    return offset;
}

void Clock::addOffset(unsigned long off)
{
    offset += off;
}

}

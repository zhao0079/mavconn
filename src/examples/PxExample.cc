/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

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

======================================================================*/

/**
 * @file
 *   @brief Implementation of the class PxExample.
 *
 *   @author FirstName LastName <mavteam@student.ethz.ch>
 *
 */

#include "PxExample.h"

PxExample::PxExample(void)
: m_bMemberVariable1(false),
  m_iMemberVariable2(0)
{
}

PxExample::PxExample(const PxExample &rExample)
: m_bMemberVariable1(rExample.m_bMemberVariable1),
  m_iMemberVariable2(rExample.m_iMemberVariable2)
{
}

/**
 * @param iCount			Description of the parameter iCount
 * @param rOtherExample		Description of the parameter rOtherExample
 *
 * @return Description of what the function returns
 *
 * Here comes the detailed description of this function.
 *
 * Try to give an example how to use this function whenever possible:
 *
 * @code
 * PxExample example1;
 * PxExample example2;
 * example1.somePublicFunction(1, example2);
 * @endcode
 */
bool PxExample::somePublicFunction(int iCount, const PxExample &rOtherExample) const
{
	return true;
}

/**
 * Here comes the detailed description of this function.
 *
 * For class internal function no usage examples have to be given.
 */
void PxExample::somePrivateFunction(void)
{
	return;
}

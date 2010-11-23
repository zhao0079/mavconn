/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

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

======================================================================*/

/**
 * @file
 *   @brief Definition of the class PxExample.
 *
 *   @author FirstName LastName <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup examples */
/*@{*/


#ifndef PX_EXAMPLE_H
#define PX_EXAMPLE_H

#include <cv.h>

/**
 * @brief Brief description of this class
 *
 * More detailed description of this class.
 *
 */
class PxExample
{
public:
	/** @brief Standard constructor */
	PxExample(void);
	/** @brief Copy constructor */
	PxExample(const PxExample &rExample);
	/** @brief Destructor */
	~PxExample(void) { }

public:
	/** @brief Brief description of somePublicFunction */
	bool somePublicFunction(int iCount, const PxExample &rOtherExample) const;

public:
	/** @name Name of the Group
	 * You can also group functions. This is the Description of the Group.
	 */
	/*@{*/
	/** @brief Brief Description */
	void someGrouppedFunction1(void);
	/** @brief Brief Description */
	void someGrouppedFunction2(void);
	/*@}*/

private:
	/** @brief Brief description of somePrivateFunction */
	void somePrivateFunction(void);

private:
	bool m_bMemberVariable1;	///< Description of m_bMemberVariable1
	int m_iMemberVariable2;		///< Description of m_iMemberVariable2
};

#endif //PX_EXAMPLE_H

/*@}*/

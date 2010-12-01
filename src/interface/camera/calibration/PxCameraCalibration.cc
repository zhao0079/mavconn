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
 *   @brief Implementation of the class PxCameraCalibrationOmnidirectional.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *   @author David de Bos <mavteam@student.ethz.ch>
 *
 */

#include "PxCameraCalibration.h"

/**
* @return	The intrinsic matrix.
*/
const PxMatrix3x3 &PxCameraCalibration::getIntrinsicMatrix(void) const
{
	return m_intrisicMatrix;
}

/**
* @return	The inverse intrinsic matrix.
*/
const PxMatrix3x3 &PxCameraCalibration::getInverseIntrinsicMatrix(void) const
{
	return m_intrisicMatrixInverse;
}

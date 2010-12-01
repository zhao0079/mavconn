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
 *   @brief Definition of the class PxCameraCalibration.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *   @author David de Bos <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup camera_calibration */
/*@{*/


#ifndef _PX_CAMERA_CALIBRATION_H_
#define _PX_CAMERA_CALIBRATION_H_

#include <cv.h>
#include <PxMatrix.h>

/**
 * @brief The superclass for calibration data wrapper classes.
 *
 * PxCameraCalibration defines the interface to load calibration data for cameras and undistort images and points.
 *
 */
class PxCameraCalibration
{
public:
	/** @brief Creates an undistortion mapping for cvRemap. */
	virtual void initUndistortMap(IplImage &rMapX, IplImage &rMapY) const = 0;

	/** @brief Undistorts one or more points. */
	virtual void undistortPoints(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, const int count) const = 0;

	/** @brief Returns the intrinsic matrix. */
	const PxMatrix3x3 &getIntrinsicMatrix(void) const;

	/** @brief Returns the inverse of the intrinsic matrix. */
	const PxMatrix3x3 &getInverseIntrinsicMatrix(void) const;

protected:
	PxMatrix3x3 m_intrisicMatrix;
	PxMatrix3x3 m_intrisicMatrixInverse;
};

#endif //_PX_CAMERA_CALIBRATION_H_

/*@}*/

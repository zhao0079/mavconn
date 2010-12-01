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
 *   @brief Definition of the class PxCameraCalibrationOmnidirectional.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup camera_calibration */
/*@{*/

#ifndef _PX_CAMERA_CALIBRATION_OMNIDIRECTIONAL_H_
#define _PX_CAMERA_CALIBRATION_OMNIDIRECTIONAL_H_

#include <cv.h>
#include <PxCameraCalibration.h>

/**
 * @brief The calibration data wrapper classes for omnidirectional cameras.
 *
 * PxCameraCalibrationOmnidirectional loads calibration data for omnidirectional
 * cameras and offers functions to undistort images and points.
 *
 * One speciality of omnidirectional calibration is that there is no K matrix
 * inherently defined. One has to chose an focal length for a virtual camera and
 * then all points are undistorted accordingly.
 *
 * \code
 *         focal   0    center_x
 * K    =    0   focal  center_y
 *           0     0       1
 *
 *         1/focal   0     center_x/focal
 * K^-1 =     0    1/focal center_y/focal
 *            0      0           1
 * \endcode
 *
 * The camera center for the undistorted image is chosen to be the center of the
 * image size which was used during the calibration.
 *
 */
class PxCameraCalibrationOmnidirectional : public PxCameraCalibration
{
public:
	PxCameraCalibrationOmnidirectional(const char *filename, const float focal);

public:
	/** @brief Creates an undistortion mapping for cvRemap. */
	virtual void initUndistortMap(IplImage &map1, IplImage &map2) const;

	/** @brief Undistorts one or more points. */
	virtual void undistortPoints(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, int count) const;

	/** @brief Distorts one or more points. */
	void distortPoints(const CvPoint2D32f* pSrc, CvPoint2D32f* pDest, int count) const;

private:
	float *m_pDistortionCoefficients;				///< The coefficients of the nth-degree polynomial from the omnidirectional calibration
	int m_NumDistCoeffs;							///< The number of coefficients of the polynomial
	float *m_pInverseDistortionCoefficients;		///< The coefficients of the nth-degree inverse polynomial from the omnidirectional calibration
	int m_NumInvDistCoeffs;							///< The number of coefficients of the inverse polynomial
	float m_aAffineCoefficients[3];					///< The coefficients of the affine distortion
	float m_aInverseAffineCoefficients[3];			///< The coefficients of the inverse affine distortion
	float m_aCameraCenter[2];						///< The coordinates in the image of the camera center
	float m_focal;									///< The desired focal length for the rectification
	float m_imageCenterX;							///< The x-coordinate of the virtual camera center
	float m_imageCenterY;							///< The y-coordinate of the virtual camera center
};

#endif //_PX_CAMERA_CALIBRATION_OMNIDIRECTIONAL_H_

/*@}*/

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
 *   @brief Definition of the class PxCameraStereoCalibration.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup camera_calibration */
/*@{*/


#ifndef _PX_CAMERA_STEREO_CALIBRATION_H_
#define _PX_CAMERA_STEREO_CALIBRATION_H_

#include <cv.h>
#include <PxCameraCalibrationStandard.h>
#include <PxMatrix.h>

/**
 * @brief The superclass for calibration data wrapper classes.
 *
 * PxCameraCalibration defines the interface to load calibration data for cameras and undistort images and points.
 *
 */
class PxCameraStereoCalibration
{
public:
	/** @brief Constructor. */
	PxCameraStereoCalibration(const char *filename, const CvSize &frameSize);
	/** @brief Destructor. */
	~PxCameraStereoCalibration(void);

public:
	/** @brief Returns the x undistortion mapping of the left camera for cvRemap. */
	const IplImage *getUndistortMapXLeft(void) const { return m_mapxL; }
	/** @brief Returns the y undistortion mapping of the left camera for cvRemap. */
	const IplImage *getUndistortMapYLeft(void) const { return m_mapyL; }
	/** @brief Returns the x undistortion mapping of the right camera for cvRemap. */
	const IplImage *getUndistortMapXRight(void) const { return m_mapxR; }
	/** @brief Returns the y undistortion mapping of the right camera for cvRemap. */
	const IplImage *getUndistortMapYRight(void) const { return m_mapyR; }

	/** @brief Undistorts one or more points from the left camera. */
	void undistortPointsLeft(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, const int count) const;
	/** @brief Undistorts one or more points from the right camera. */
	void undistortPointsRight(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, const int count) const;

	/** @brief Returns the intrinsic matrix of the left camera. */
	const PxMatrix3x3 &getIntrinsicMatrixLeft(void) const { return m_intrisicMatrixL; }
	/** @brief Returns the intrinsic matrix of the right camera. */
	const PxMatrix3x3 &getIntrinsicMatrixRight(void) const { return m_intrisicMatrixR; }

	/** @brief Returns the inverse of the intrinsic matrix of the left camera. */
	const PxMatrix3x3 &getInverseIntrinsicMatrixLeft(void) const { return m_intrisicMatrixInverseL; }
	/** @brief Returns the inverse of the intrinsic matrix of the right camera. */
	const PxMatrix3x3 &getInverseIntrinsicMatrixRight(void) const { return m_intrisicMatrixInverseR; }

	/** @brief Returns the calibration of the left camera. */
	const PxCameraCalibrationStandard &getOriginalCalibrationLeft(void) const;
	/** @brief Returns the calibration of the right camera. */
	const PxCameraCalibrationStandard &getOriginalCalibrationRight(void) const;

	const double *getTranslation(void) const { return m_translation; }
	const double *getDisparityToDepthMatrix(void) const { return m_Q; }

protected:
	PxCameraCalibrationStandard* m_calibL;
	PxCameraCalibrationStandard* m_calibR;
	double m_translation[3];
	double m_rotation[3];
	double m_rotationL[3][3];
	double m_rotationR[3][3];

	CvSize m_frameSize;
	IplImage *m_mapxL;
	IplImage *m_mapyL;
	IplImage *m_mapxR;
	IplImage *m_mapyR;

	double m_R1[3][3];
	double m_R2[3][3];
	double m_P1[3][4];
	double m_P2[3][4];
	double m_Q[16];

	PxMatrix3x3 m_RectTransformL;
	PxMatrix3x3 m_RectTransformR;

	PxMatrix3x3 m_intrisicMatrixL;
	PxMatrix3x3 m_intrisicMatrixInverseL;
	PxMatrix3x3 m_intrisicMatrixR;
	PxMatrix3x3 m_intrisicMatrixInverseR;
};

#endif //_PX_CAMERA_STEREO_CALIBRATION_H_

/*@}*/

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
 *   @brief Implementation of the class PxCameraCalibrationStandard.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *   @author David de Bos <mavteam@student.ethz.ch>
 *
 */

#include "PxCameraCalibrationStandard.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
using namespace std;

/**
 * @param filename		Path to the calibration file
 */
PxCameraCalibrationStandard::PxCameraCalibrationStandard(const char *filename)
{

	// Load the calibration file
    ifstream calfile;
    calfile.open(filename);
    if (!calfile) {
        cout << "Unable to open file " << filename << endl;
        exit(1); // terminate with error
    }

    // Ignore the fileheader for now
    string fileHeader;
    getline(calfile,fileHeader);

    float buf;
    // Process the values
    calfile >> buf; calfile >> buf; // ignore image size
    calfile >> m_cc[0]; calfile >> m_cc[1]; // get camera center
    calfile >> m_focal[0]; calfile >> m_focal[1]; // get focal length
	calfile >> m_kc[0]; calfile >> m_kc[1]; calfile >> m_kc[2]; calfile >> m_kc[3]; calfile >> m_kc[4]; // get distortion parameters

    // Close the calibration file
    calfile.close();

	// Set intrinsic matrix
    m_intrisicMatrix[0] = m_focal[0]; m_intrisicMatrix[1] = 0.0f;       m_intrisicMatrix[2] = m_cc[0];
    m_intrisicMatrix[3] = 0.0f;       m_intrisicMatrix[4] = m_focal[1]; m_intrisicMatrix[5] = m_cc[1];
    m_intrisicMatrix[6] = 0.0f;       m_intrisicMatrix[7] = 0.0f;       m_intrisicMatrix[8] = 1.0f;

    // Set inverse intrinsic matrix
    m_intrisicMatrixInverse[0] = 1.0f/m_focal[0]; m_intrisicMatrixInverse[1] = 0.0f;            m_intrisicMatrixInverse[2] = -m_cc[0]/m_focal[0];
    m_intrisicMatrixInverse[3] = 0.0f;            m_intrisicMatrixInverse[4] = 1.0f/m_focal[1]; m_intrisicMatrixInverse[5] = -m_cc[1]/m_focal[1];
    m_intrisicMatrixInverse[6] = 0.0f;            m_intrisicMatrixInverse[7] = 0.0f;            m_intrisicMatrixInverse[8] = 1.0f;

    distortion = cvMat(5, 1, CV_32FC1, m_kc);
}

/**
 * @param rMapX		The map for the x coordinates
 * @param rMapY		The map for the y coordinates
 *
 * Creates the undistortion mapping of a calibrated camera (for usage with cvRemap).
 *
 * All matrices, scalars and maps have to be floats!
 *
 * The size of rMapX and rMapY has to be equal to the size of the image which will be undistorted.
 */
void PxCameraCalibrationStandard::initUndistortMap(IplImage &rMapX, IplImage &rMapY) const
{
	// Just use the OpenCV InitUndistortMap function
	cvInitUndistortMap(&m_intrisicMatrix, &distortion, &rMapX, &rMapY);
}

/**
* \param pSrc		An array of distorted image points (or pointer to a single CvPoint2D32f if only one point should be undistorted, use with count=1)
* \param pDest		An array where the undistorted image points are written into, can be equal to _src
* \param count		Number of the input and output points
*
* Computes the undistorted image coordinates of all the input points taken with radial and tangential distortion.
*
* All matrices, scalars and maps have to be floats!
*/
void PxCameraCalibrationStandard::undistortPoints(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, int count) const
{
	const int iters = 6;
	const float ifx = m_intrisicMatrixInverse[0]; // 1.f / focal_x;
	const float ify = m_intrisicMatrixInverse[4]; // 1.f / focal_y;
	const float fx = m_intrisicMatrix[0];
	const float fy = m_intrisicMatrix[4];
	const float cx = m_cc[0];
	const float cy = m_cc[1];

	for(int i = 0; i < count; i++ )
	{
		float x = pSrc[i].x;
		float y = pSrc[i].y;

		float x0 = x = (x - cx)*ifx;
		float y0 = y = (y - cy)*ify;

		// compensate distortion iteratively
		for(int j = 0; j < iters; j++ )
		{
			float r2 = x*x + y*y;
			float icdist = 1.f/(1 + ((m_kc[4]*r2 + m_kc[1])*r2 + m_kc[0])*r2);
			float deltaX = 2*m_kc[2]*x*y + m_kc[3]*(r2 + 2*x*x);
			float deltaY = m_kc[2]*(r2 + 2*y*y) + 2*m_kc[3]*x*y;
			x = (x0 - deltaX)*icdist;
			y = (y0 - deltaY)*icdist;
		}
		pDest[i].x = x*fx + cx;
		pDest[i].y = y*fy + cy;
	}
}

void PxCameraCalibrationStandard::distortPoints(const CvPoint2D32f* pSrc, CvPoint2D32f* pDest, int count) const
{
	//there is no opencv function for this
}

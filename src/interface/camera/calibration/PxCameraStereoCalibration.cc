/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

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
 *   @brief Implementation of the class PxCameraStereoCalibration.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

#include "PxCameraStereoCalibration.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
using namespace std;

/**
 * @param filename		Path to the calibration file
 * @param frameSize     Size of the image
 */
PxCameraStereoCalibration::PxCameraStereoCalibration(const char *filename, const CvSize &frameSize)
: m_calibL(NULL), m_calibR(NULL), m_frameSize(frameSize)
{
	// Load the calibration file
    ifstream calfile;
    calfile.open(filename, ifstream::in);
    if (!calfile) {
    	perror("Unable to open file");
        //cout << "Unable to open file\n";
        exit(1); // terminate with error
    }

    // read the filename of the left camera calibration file
    string fileCalibL;
    getline(calfile,fileCalibL);
    m_calibL = new PxCameraCalibrationStandard(fileCalibL.c_str());

    // read the filename of the right camera calibration file
	string fileCalibR;
	getline(calfile,fileCalibR);
	m_calibR = new PxCameraCalibrationStandard(fileCalibR.c_str());

    // Process the values
    calfile >> m_rotation[0]; calfile >> m_rotation[1]; calfile >> m_rotation[2]; // rotation
    calfile >> m_translation[0]; calfile >> m_translation[1]; calfile >> m_translation[2]; // translation

    // Close the calibration file
    calfile.close();

    m_mapxL = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);
    m_mapyL = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);
    m_mapxR = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);
    m_mapyR = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);

    // now create the rectifying undistortion maps
    // since OpenCV supports only doubles for these operations, funny conversions follow...

    CvMat R1 = cvMat(3, 3, CV_64F, m_R1);
	CvMat R2 = cvMat(3, 3, CV_64F, m_R2);
    CvMat P1 = cvMat(3, 4, CV_64F, m_P1);
    CvMat P2 = cvMat(3, 4, CV_64F, m_P2);
    CvMat Q = cvMat(4, 4, CV_64F, m_Q);

    //Stereo rig calibration
    double intrinsicDataLeft[3][3];
    double distortionDataLeft[5];
    double intrinsicDataRight[3][3];
    double distortionDataRight[5];
    double rotMatData[3][3];
    CvMat intrinsicMatrixLeft = cvMat(3, 3, CV_64F, intrinsicDataLeft);
	CvMat distortionMatrixLeft = cvMat(5, 1, CV_64F, distortionDataLeft);
	CvMat intrinsicMatrixRight = cvMat(3, 3, CV_64F, intrinsicDataRight);
	CvMat distortionMatrixRight = cvMat(5, 1, CV_64F, distortionDataRight);
	CvMat rotationMatrix = cvMat(3, 3, CV_64F, rotMatData);

	cvConvert(&m_calibL->getIntrinsicMatrix(), &intrinsicMatrixLeft);
	cvConvert(&m_calibR->getIntrinsicMatrix(), &intrinsicMatrixRight);
	cvConvert(&m_calibL->distortion, &distortionMatrixLeft);
	cvConvert(&m_calibR->distortion, &distortionMatrixRight);

	CvMat translationVector = cvMat(3, 1, CV_64F, m_translation);
	CvMat rotationVector = cvMat(3, 1, CV_64F, m_rotation);
	cvRodrigues2(&rotationVector, &rotationMatrix);

	cvStereoRectify(&intrinsicMatrixLeft, &intrinsicMatrixRight,
				    &distortionMatrixLeft, &distortionMatrixRight,
				    frameSize, &rotationMatrix, &translationVector,
				    &R1, &R2, &P1, &P2, &Q);
	cvInitUndistortRectifyMap(&m_calibL->getIntrinsicMatrix(), &m_calibL->distortion, &R1, &P1, m_mapxL, m_mapyL);
	cvInitUndistortRectifyMap(&m_calibR->getIntrinsicMatrix(), &m_calibR->distortion, &R2, &P2, m_mapxR, m_mapyR);

	/*int widthStep = m_mapxL->widthStep;
	for (int y = 0; y < frameSize.height; y++)
	{
		for (int x = 0; x < frameSize.width; x++)
		{
			if(((float*)m_mapxL->imageData)[y*widthStep+x] < 0.f || ((float*)m_mapyL->imageData)[y*widthStep+x] < 0.f)// || ((float*)m_mapxL->imageData)[y*widthStep+x] > (float)frameSize.width)
			{
				((float*)m_mapxL->imageData)[y*widthStep+x] = 0.f;
				((float*)m_mapyL->imageData)[y*widthStep+x] = 0.f;
			}
			// || ((float*)m_mapyL->imageData)[y*widthStep+x] > (float)frameSize.height)
			if(((float*)m_mapxR->imageData)[y*widthStep+x] < 0.f)// || ((float*)m_mapxR->imageData)[y*widthStep+x] > (float)frameSize.width)
			{
				((float*)m_mapxR->imageData)[y*widthStep+x] = 0.f;
				((float*)m_mapyR->imageData)[y*widthStep+x] = 0.f;
			}
			if(((float*)m_mapyR->imageData)[y*widthStep+x] < 0.f)// || ((float*)m_mapyR->imageData)[y*widthStep+x] > (float)frameSize.height)
			{
				((float*)m_mapxR->imageData)[y*widthStep+x] = 0.f;
				((float*)m_mapyR->imageData)[y*widthStep+x] = 0.f;
			}
		}
	}*/

	// create the rectifying transformations (Px * Rx) - this is for undistortion of points
	double PP[3][3];
	double tmp[3][3];
	CvMat _P3x3, _PP=cvMat(3, 3, CV_64F, PP);
	CvMat _tmp=cvMat(3, 3, CV_64F, tmp);
	cvConvert( cvGetCols(&P1, &_P3x3, 0, 3), &_PP );
	cvMatMul( &_PP, &R1, &_tmp );
	cvConvert(&_tmp, &m_RectTransformL);
	cvConvert( cvGetCols(&P2, &_P3x3, 0, 3), &_PP );
	cvMatMul( &_PP, &R2, &_tmp );
	cvConvert(&_tmp, &m_RectTransformR);

	// create the new float intrinsic matrices

	m_intrisicMatrixL[0] = (float)m_P1[0][0]; m_intrisicMatrixL[1] = 0.f;               m_intrisicMatrixL[2] = (float)m_P1[0][2];
	m_intrisicMatrixL[3] = 0.f;               m_intrisicMatrixL[4] = (float)m_P1[1][1]; m_intrisicMatrixL[5] = (float)m_P1[1][2];
	m_intrisicMatrixL[6] = 0.f;               m_intrisicMatrixL[7] = 0.f;               m_intrisicMatrixL[8] = 1.f;

	m_intrisicMatrixR[0] = (float)m_P2[0][0]; m_intrisicMatrixR[1] = 0.f;               m_intrisicMatrixR[2] = (float)m_P2[0][2];
	m_intrisicMatrixR[3] = 0.f;               m_intrisicMatrixR[4] = (float)m_P2[1][1]; m_intrisicMatrixR[5] = (float)m_P2[1][2];
	m_intrisicMatrixR[6] = 0.f;               m_intrisicMatrixR[7] = 0.f;               m_intrisicMatrixR[8] = 1.f;

	m_intrisicMatrixInverseL[0] = (float)(1.0/m_P1[0][0]); m_intrisicMatrixInverseL[1] = 0.0f;                    m_intrisicMatrixInverseL[2] = (float)(-m_P1[0][2]/m_P1[0][0]);
	m_intrisicMatrixInverseL[3] = 0.0f;                    m_intrisicMatrixInverseL[4] = (float)(1.0/m_P1[1][1]); m_intrisicMatrixInverseL[5] = (float)(-m_P1[1][2]/m_P1[1][1]);
	m_intrisicMatrixInverseL[6] = 0.0f;                    m_intrisicMatrixInverseL[7] = 0.0f;                    m_intrisicMatrixInverseL[8] = 1.0f;

	m_intrisicMatrixInverseR[0] = (float)(1.0/m_P2[0][0]); m_intrisicMatrixInverseR[1] = 0.0f;                    m_intrisicMatrixInverseR[2] = (float)(-m_P2[0][2]/m_P2[0][0]);
	m_intrisicMatrixInverseR[3] = 0.0f;                    m_intrisicMatrixInverseR[4] = (float)(1.0/m_P2[1][1]); m_intrisicMatrixInverseR[5] = (float)(-m_P2[1][2]/m_P2[1][1]);
	m_intrisicMatrixInverseR[6] = 0.0f;                    m_intrisicMatrixInverseR[7] = 0.0f;                    m_intrisicMatrixInverseR[8] = 1.0f;
}

PxCameraStereoCalibration::~PxCameraStereoCalibration(void)
{
	if (m_calibL) delete m_calibL;
	if (m_calibR) delete m_calibR;
}

/**
* \param pSrc		An array of distorted image points (or pointer to a single CvPoint2D32f if only one point should be undistorted, use with count=1)
* \param pDest		An array where the undistorted image points are written into, can be equal to _src
* \param count		Number of the input and output points
*
* Computes the undistorted image coordinates of all the input points in the left image.
*
* All matrices, scalars and maps have to be floats!
*/
void PxCameraStereoCalibration::undistortPointsLeft(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, int count) const
{
	const int iters = 5;
	const float ifx = m_calibL->m_intrisicMatrixInverse[0]; // 1.f / focal_x;
	const float ify = m_calibL->m_intrisicMatrixInverse[4]; // 1.f / focal_y;
	const float cx = m_calibL->m_cc[0];
	const float cy = m_calibL->m_cc[1];
	const float *m_kc = m_calibL->m_kc;

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

		float xx = m_RectTransformL[0]*x + m_RectTransformL[1]*y + m_RectTransformL[2];
		float yy = m_RectTransformL[3]*x + m_RectTransformL[4]*y + m_RectTransformL[5];
		float ww = 1.f/(m_RectTransformL[6]*x + m_RectTransformL[7]*y + m_RectTransformL[8]);

		pDest[i].x = xx*ww;
		pDest[i].y = yy*ww;
	}
}

/**
* \param pSrc		An array of distorted image points (or pointer to a single CvPoint2D32f if only one point should be undistorted, use with count=1)
* \param pDest		An array where the undistorted image points are written into, can be equal to _src
* \param count		Number of the input and output points
*
* Computes the undistorted image coordinates of all the input points in the right image.
*
* All matrices, scalars and maps have to be floats!
*/
void PxCameraStereoCalibration::undistortPointsRight(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, int count) const
{
	const int iters = 5;
	const float ifx = m_calibR->m_intrisicMatrixInverse[0]; // 1.f / focal_x;
	const float ify = m_calibR->m_intrisicMatrixInverse[4]; // 1.f / focal_y;
	const float cx = m_calibR->m_cc[0];
	const float cy = m_calibR->m_cc[1];
	const float *m_kc = m_calibR->m_kc;

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

		float xx = m_RectTransformR[0]*x + m_RectTransformR[1]*y + m_RectTransformR[2];
		float yy = m_RectTransformR[3]*x + m_RectTransformR[4]*y + m_RectTransformR[5];
		float ww = 1.f/(m_RectTransformR[6]*x + m_RectTransformR[7]*y + m_RectTransformR[8]);

		pDest[i].x = xx*ww;
		pDest[i].y = yy*ww;
	}
}

/**
* @return	The calibration object of the left camera.
*/
const PxCameraCalibrationStandard &PxCameraStereoCalibration::getOriginalCalibrationLeft(void) const
{
	assert(m_calibL != NULL);
	return *m_calibL;
}

/**
* @return	The calibration object of the right camera.
*/
const PxCameraCalibrationStandard &PxCameraStereoCalibration::getOriginalCalibrationRight(void) const
{
	assert(m_calibR != NULL);
	return *m_calibR;
}


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
 *
 */

#include "PxCameraCalibrationOmnidirectional.h"
#include <limits>
#include <fstream>

/**
 * @param filename		Path to the calibration file
 * @param focal			The virtual focal length
 */
PxCameraCalibrationOmnidirectional::PxCameraCalibrationOmnidirectional(const char *filename, const float focal)
: m_pDistortionCoefficients(NULL),
  m_NumDistCoeffs(0),
  m_pInverseDistortionCoefficients(NULL),
  m_NumInvDistCoeffs(0),
  m_focal(focal)
{
	// Load the calibration file
	std::ifstream calfile;
	calfile.open(filename);
	if (!calfile) {
		printf("Unable to open file\n");
		exit(1); // terminate with error
	}

	int line = 0;
	while (!calfile.eof())
	{
		if (line >= 5)
			break;
		char c = (char)calfile.peek();
		if (c == '#')
			calfile.ignore(1024, '\n');
		else if (c == '\n' || c == ' ' || c == '\r' /* windows... */)
			calfile.ignore(1);
		else
		{
			int count = 0;
			switch(line)
			{
			case 0:
				calfile >> count;
				m_pDistortionCoefficients = (float *) malloc(sizeof(float)*count);
				m_NumDistCoeffs = count;
				for (int i=0; i < count; i++)
				{
					calfile >> m_pDistortionCoefficients[i];
				}
				line++;
				break;
			case 1:
				calfile >> count;
				m_pInverseDistortionCoefficients = (float *) malloc(sizeof(float)*count);
				m_NumInvDistCoeffs = count;
				for (int i=0; i < count; i++)
				{
					calfile >> m_pInverseDistortionCoefficients[i];
				}
				line++;
				break;
			case 2:
				calfile >> m_aCameraCenter[1]; calfile >> m_aCameraCenter[0];
				line++;
				break;
			case 3:
				calfile >> m_aAffineCoefficients[0]; calfile >> m_aAffineCoefficients[1]; calfile >> m_aAffineCoefficients[2];
				line++;
				break;
			case 4:
				calfile >> m_imageCenterY; calfile >> m_imageCenterX;
				m_imageCenterX *= 0.5f;
				m_imageCenterY *= 0.5f;
				line++;
				break;
			}
		}
	}
	// Close the calibration file
	calfile.close();

	//compute inverse of the 2x2 affine distortion matrix
	const float det = 1.f / (m_aAffineCoefficients[0] - m_aAffineCoefficients[1]*m_aAffineCoefficients[2]);
	m_aInverseAffineCoefficients[0] = -m_aAffineCoefficients[2]*det;
	m_aInverseAffineCoefficients[1] = -m_aAffineCoefficients[1]*det;
	m_aInverseAffineCoefficients[2] = m_aAffineCoefficients[0]*det;

	// Set intrinsic matrix
	m_intrisicMatrix[0] = m_focal; m_intrisicMatrix[1] = 0.0f;    m_intrisicMatrix[2] = m_imageCenterX;
	m_intrisicMatrix[3] = 0.0f;    m_intrisicMatrix[4] = m_focal; m_intrisicMatrix[5] = m_imageCenterY;
	m_intrisicMatrix[6] = 0.0f;    m_intrisicMatrix[7] = 0.0f;    m_intrisicMatrix[8] = 1.0f;

	// Set inverse intrinsic matrix
	m_intrisicMatrixInverse[0] = 1.0f/m_focal; m_intrisicMatrixInverse[1] = 0.0f;         m_intrisicMatrixInverse[2] = -m_imageCenterX/m_focal;
	m_intrisicMatrixInverse[3] = 0.0f;         m_intrisicMatrixInverse[4] = 1.0f/m_focal; m_intrisicMatrixInverse[5] = -m_imageCenterY/m_focal;
	m_intrisicMatrixInverse[6] = 0.0f;         m_intrisicMatrixInverse[7] = 0.0f;         m_intrisicMatrixInverse[8] = 1.0f;
}

/**
 * @param rMapX		The map for the x coordinates
 * @param rMapY		The map for the y coordinates
 *
 * Creates the undistortion mapping of a calibrated omnidirectional lens (for usage with cvRemap).
 *
 * All matrices, scalars and maps have to be floats!
 *
 * The size of rMapX and rMapY has to be equal to the size of the image which will be undistorted.
 */
void PxCameraCalibrationOmnidirectional::initUndistortMap(IplImage &rMapX, IplImage &rMapY) const
{
	for( int i = 0; i < rMapX.height; i++ )
	{
		float* m1f = (float*)(rMapX.imageData + rMapX.widthStep*i);
		float* m2f = (float*)(rMapY.imageData + rMapY.widthStep*i);

		for( int j = 0; j < rMapX.width; j++)
		{
			const float Nx = j-m_imageCenterX;
			const float Ny = i-m_imageCenterY;
			float norm = sqrt(Nx*Nx + Ny*Ny);
			if (norm == 0.f)
				norm = std::numeric_limits<float>::min();

			const float normInv = 1.f / norm;

			const float theta = atan(-m_focal * normInv);
			float rho = 0.f;
			for (int c=m_NumInvDistCoeffs-1; c>=0; c--)
			{	// Horner's method for polynom evaluation
				rho = theta * rho + m_pInverseDistortionCoefficients[c];
			}

			const float x = Nx * normInv * rho;
			const float y = Ny * normInv * rho;

			m1f[j] = x*m_aAffineCoefficients[0] + y*m_aAffineCoefficients[1] + m_aCameraCenter[0];
			m2f[j] = x*m_aAffineCoefficients[2] + y                          + m_aCameraCenter[1];
		}
	}
}

/**
* \param pSrc		An array of distorted image points (or pointer to a single CvPoint2D32f if only one point should be undistorted, use with count=1)
* \param pDest		An array where the undistorted image points are written into, can be equal to _src
* \param count		Number of the input and output points
*
* Computes the undistorted image coordinates of all the input points taken with omnidirectional distortion.
*
* All matrices, scalars and maps have to be floats!
*/
void PxCameraCalibrationOmnidirectional::undistortPoints(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, int count) const
{
	for( int i = 0; i < count; i++ )
	{
		const float tx = pSrc[i].x - m_aCameraCenter[0];
		const float ty = pSrc[i].y - m_aCameraCenter[1];
		const float sx = tx                                 + ty*m_aInverseAffineCoefficients[0];
		const float sy = tx*m_aInverseAffineCoefficients[1] + ty*m_aInverseAffineCoefficients[2];
		const float norm = sqrt(sx*sx + sy*sy);

		float w = 0.f;
		for (int c=m_NumDistCoeffs-1; c>=0; c--)
		{	// Horner's method for polynome evaluation
			w = norm*w + m_pDistortionCoefficients[c];
		}

		w = 1.f / w * -m_focal;
		pDest[i].x = sx*w + m_imageCenterX;
		pDest[i].y = sy*w + m_imageCenterY;
	}
}

void PxCameraCalibrationOmnidirectional::distortPoints(const CvPoint2D32f* pSrc, CvPoint2D32f* pDest, int count) const
{
	for( int i = 0; i < count; i++ )
	{
		const float Nx = pSrc[i].x-m_imageCenterX;
		const float Ny = pSrc[i].y-m_imageCenterY;
		float norm = sqrt(Nx*Nx + Ny*Ny);
		if (norm == 0.f)
			norm = std::numeric_limits<float>::min();

		const float normInv = 1.f / norm;

		const float theta = atan(-m_focal * normInv);
		float rho = 0.f;
		for (int c=m_NumInvDistCoeffs-1; c>=0; c--)
		{	// Horner's method for polynom evaluation
			rho = theta * rho + m_pInverseDistortionCoefficients[c];
		}

		const float x = Nx * normInv * rho;
		const float y = Ny * normInv * rho;

		pDest[i].x = x*m_aAffineCoefficients[0] + y*m_aAffineCoefficients[1] + m_aCameraCenter[0];
		pDest[i].y = x*m_aAffineCoefficients[2] + y                          + m_aCameraCenter[1];
	}
}

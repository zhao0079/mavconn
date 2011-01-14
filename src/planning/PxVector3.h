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
 *   @brief Definition of the Pixhawk Vector3 class
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup planning */
/*@{*/

#ifndef _PX_VECTOR3_H_
#define _PX_VECTOR3_H_

#include <math.h>


/**
 * @brief Pixhawk 3D vector class, no support for OpenCV CvMat.
 *
 */
class PxVector3
{
public:
	/** @brief standard constructor */
	PxVector3(void) {}
	/** @brief copy constructor */
	PxVector3(const PxVector3 &v) { for (int i=0; i < 3; i++) { m_vec[i] = v.m_vec[i]; } }
	/** @brief x,y,z constructor */
	PxVector3(const float _x, const float _y, const float _z) { m_vec[0] = _x; m_vec[1] = _y; m_vec[2] = _z; }
	/** @brief broadcast constructor */
	PxVector3(const float _f) { for (int i=0; i < 3; i++) { m_vec[i] = _f; } }

private:
	/** @brief private constructor (not used here, for SSE compatibility) */
	PxVector3(const float (&_vec)[3]) { for (int i=0; i < 3; i++) { m_vec[i] = _vec[i]; } }

public:
	/** @brief assignment operator */
	void operator= (const PxVector3 &r) { for (int i=0; i < 3; i++) { m_vec[i] = r.m_vec[i]; } }
	/** @brief const element access */
	float operator[] (const int i) const { return m_vec[i]; }
	/** @brief element access */
	float &operator[] (const int i) { return m_vec[i]; }

	// === arithmetic operators ===
	/** @brief element-wise negation */
	friend PxVector3 operator- (const PxVector3 &v) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = -v.m_vec[i]; } return ret; }
	friend PxVector3 operator+ (const PxVector3 &l, const PxVector3 &r) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] + r.m_vec[i]; } return ret; }
	friend PxVector3 operator- (const PxVector3 &l, const PxVector3 &r) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] - r.m_vec[i]; } return ret; }
	friend PxVector3 operator* (const PxVector3 &l, const PxVector3 &r) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * r.m_vec[i]; } return ret; }
	friend PxVector3 operator/ (const PxVector3 &l, const PxVector3 &r) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] / r.m_vec[i]; } return ret; }

	friend void operator+= (PxVector3 &l, const PxVector3 &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] + r.m_vec[i]; } }
	friend void operator-= (PxVector3 &l, const PxVector3 &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] - r.m_vec[i]; } }
	friend void operator*= (PxVector3 &l, const PxVector3 &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * r.m_vec[i]; } }
	friend void operator/= (PxVector3 &l, const PxVector3 &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] / r.m_vec[i]; } }

	friend PxVector3 operator+ (const PxVector3 &l, float f) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] + f; } return ret; }
	friend PxVector3 operator- (const PxVector3 &l, float f) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] - f; } return ret; }
	friend PxVector3 operator* (const PxVector3 &l, float f) { PxVector3 ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * f; } return ret; }
	friend PxVector3 operator/ (const PxVector3 &l, float f) { PxVector3 ret; float inv = 1.f/f; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * inv; } return ret; }

	friend void operator+= (PxVector3 &l, float f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] + f; } }
	friend void operator-= (PxVector3 &l, float f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] - f; } }
	friend void operator*= (PxVector3 &l, float f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * f; } }
	friend void operator/= (PxVector3 &l, float f) { float inv = 1.f/f; for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * inv; } }

	// === vector operators ===
	/** @brief dot product */
	float	dot(const PxVector3 &v) const { return m_vec[0]*v.m_vec[0] + m_vec[1]*v.m_vec[1] + m_vec[2]*v.m_vec[2]; }
	/** @brief length squared of the vector */
	float	lengthSquared(void) const { return m_vec[0]*m_vec[0] + m_vec[1]*m_vec[1] + m_vec[2]*m_vec[2]; }
	/** @brief length of the vector */
	float	length(void) const { return sqrt(lengthSquared()); }
	/** @brief cross product */
	PxVector3 cross(const PxVector3 &v) const { return PxVector3(m_vec[1]*v.m_vec[2] - m_vec[2]*v.m_vec[1], m_vec[2]*v.m_vec[0] - m_vec[0]*v.m_vec[2], m_vec[0]*v.m_vec[1] - m_vec[1]*v.m_vec[0]); }
	/** @brief normalizes the vector */
	PxVector3 &normalize(void) { const float l = 1.f / length(); for (int i=0; i < 3; i++) { m_vec[i] *= l; } return *this; }

protected:
	float m_vec[3];
};

/**
 * @brief Pixhawk 3D vector class in double precision, can be cast to a local OpenCV CvMat.
 *
 */
class PxVector3Double
{
public:
	/** @brief standard constructor */
	PxVector3Double(void) {}
	/** @brief copy constructor */
	PxVector3Double(const PxVector3Double &v) { for (int i=0; i < 3; i++) { m_vec[i] = v.m_vec[i]; } }
	/** @brief x,y,z constructor */
	PxVector3Double(const double _x, const double _y, const double _z) { m_vec[0] = _x; m_vec[1] = _y; m_vec[2] = _z; }
	/** @brief broadcast constructor */
	PxVector3Double(const double _f) { for (int i=0; i < 3; i++) { m_vec[i] = _f; } }

private:
	/** @brief private constructor (not used here, for SSE compatibility) */
	PxVector3Double(const double (&_vec)[3]) { for (int i=0; i < 3; i++) { m_vec[i] = _vec[i]; } }

public:
	/** @brief assignment operator */
	void operator= (const PxVector3Double &r) { for (int i=0; i < 3; i++) { m_vec[i] = r.m_vec[i]; } }
	/** @brief const element access */
	double operator[] (const int i) const { return m_vec[i]; }
	/** @brief element access */
	double &operator[] (const int i) { return m_vec[i]; }

	// === arithmetic operators ===
	/** @brief element-wise negation */
	friend PxVector3Double operator- (const PxVector3Double &v) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = -v.m_vec[i]; } return ret; }
	friend PxVector3Double operator+ (const PxVector3Double &l, const PxVector3Double &r) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] + r.m_vec[i]; } return ret; }
	friend PxVector3Double operator- (const PxVector3Double &l, const PxVector3Double &r) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] - r.m_vec[i]; } return ret; }
	friend PxVector3Double operator* (const PxVector3Double &l, const PxVector3Double &r) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * r.m_vec[i]; } return ret; }
	friend PxVector3Double operator/ (const PxVector3Double &l, const PxVector3Double &r) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] / r.m_vec[i]; } return ret; }

	friend void operator+= (PxVector3Double &l, const PxVector3Double &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] + r.m_vec[i]; } }
	friend void operator-= (PxVector3Double &l, const PxVector3Double &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] - r.m_vec[i]; } }
	friend void operator*= (PxVector3Double &l, const PxVector3Double &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * r.m_vec[i]; } }
	friend void operator/= (PxVector3Double &l, const PxVector3Double &r) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] / r.m_vec[i]; } }

	friend PxVector3Double operator+ (const PxVector3Double &l, double f) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] + f; } return ret; }
	friend PxVector3Double operator- (const PxVector3Double &l, double f) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] - f; } return ret; }
	friend PxVector3Double operator* (const PxVector3Double &l, double f) { PxVector3Double ret; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * f; } return ret; }
	friend PxVector3Double operator/ (const PxVector3Double &l, double f) { PxVector3Double ret; double inv = 1.f/f; for (int i=0; i < 3; i++) { ret.m_vec[i] = l.m_vec[i] * inv; } return ret; }

	friend void operator+= (PxVector3Double &l, double f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] + f; } }
	friend void operator-= (PxVector3Double &l, double f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] - f; } }
	friend void operator*= (PxVector3Double &l, double f) { for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * f; } }
	friend void operator/= (PxVector3Double &l, double f) { double inv = 1.f/f; for (int i=0; i < 3; i++) { l.m_vec[i] = l.m_vec[i] * inv; } }

	// === vector operators ===
	/** @brief dot product */
	double	dot(const PxVector3Double &v) const { return m_vec[0]*v.m_vec[0] + m_vec[1]*v.m_vec[1] + m_vec[2]*v.m_vec[2]; }
	/** @brief length squared of the vector */
	double	lengthSquared(void) const { return m_vec[0]*m_vec[0] + m_vec[1]*m_vec[1] + m_vec[2]*m_vec[2]; }
	/** @brief length of the vector */
	double	length(void) const { return sqrt(lengthSquared()); }
	/** @brief cross product */
	PxVector3Double cross(const PxVector3Double &v) const { return PxVector3Double(m_vec[1]*v.m_vec[2] - m_vec[2]*v.m_vec[1], m_vec[2]*v.m_vec[0] - m_vec[0]*v.m_vec[2], m_vec[0]*v.m_vec[1] - m_vec[1]*v.m_vec[0]); }
	/** @brief normalizes the vector */
	PxVector3Double &normalize(void) { const double l = 1.f / length(); for (int i=0; i < 3; i++) { m_vec[i] *= l; } return *this; }

protected:
	double m_vec[3];
};

/*@}*/

#endif //_PX_MATRIX_H_

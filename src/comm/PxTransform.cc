#include "PxTransform.h"

#include <cmath>
#include <iostream>
#include <sstream>

PxTransform::PxTransform()
{

}

PxTransform::PxTransform(const PxTransform& t)
{
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			matrix[r][c] = t.matrix[r][c];
		}
	}
}

PxTransform::PxTransform(const double* data)
{
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			matrix[r][c] = data[r * 4 + c];
		}
	}
}

void
PxTransform::identity(void)
{
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			matrix[r][c] = ((r == c) ? 1 : 0);
		}
	}
}

void
PxTransform::rotateX(double theta)
{
	double ctheta = cos(theta);
	double stheta = sin(theta);

	PxTransform temp;
	temp.identity();

	temp.matrix[1][1] = ctheta;
	temp.matrix[1][2] = -stheta;
	temp.matrix[2][1] = stheta;
	temp.matrix[2][2] = ctheta;

	leftMultiply(temp);
}

void
PxTransform::rotateY(double theta)
{
	double ctheta = cos(theta);
	double stheta = sin(theta);

	PxTransform temp;
	temp.identity();

	temp.matrix[0][0] = ctheta;
	temp.matrix[0][2] = stheta;
	temp.matrix[2][0] = -stheta;
	temp.matrix[2][2] = ctheta;

	leftMultiply(temp);
}

void
PxTransform::rotateZ(double theta)
{
	double ctheta = cos(theta);
	double stheta = sin(theta);

	PxTransform temp;
	temp.identity();

	temp.matrix[0][0] = ctheta;
	temp.matrix[0][1] = -stheta;
	temp.matrix[1][0] = stheta;
	temp.matrix[1][1] = ctheta;

	leftMultiply(temp);
}

void
PxTransform::setRotation(double x, double y, double z)
{
	rotateX(x); rotateY(y); rotateZ(z);
}

void
PxTransform::getRotation(double &x, double &y, double &z)
{
	x = atan2(matrix[2][1], matrix[2][2]);
	y = asin(-matrix[2][0]);
	z = atan2(matrix[1][0], matrix[0][0]);
}

void
PxTransform::translate(double x, double y, double z)
{
	matrix[0][3] += x;
	matrix[1][3] += y;
	matrix[2][3] += z;
}

void
PxTransform::getTranslation(double &x, double &y, double &z)
{
	x = matrix[0][3];
	y = matrix[1][3];
	z = matrix[2][3];
}

void
PxTransform::leftMultiply(const PxTransform &t)
{
	PxTransform result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.matrix[i][j] = 0;
			for (int k = 0; k < 4; ++k)
			{
				result.matrix[i][j] += t.matrix[i][k] * matrix[k][j];
			}
		}
	}

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			matrix[i][j] = result.matrix[i][j];
		}
	}
}

void
PxTransform::transformPoint(double &x, double &y, double &z)
{
	double v1[3] = {x, y, z};

	double v2[3] = {0, 0, 0};
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			v2[r] += matrix[r][c] * v1[c];
		}
		v2[r] += matrix[r][3];
	}
	x = v2[0];
	y = v2[1];
	z = v2[2];
}

void
PxTransform::copy(PxTransform &dest)
{
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			dest.matrix[r][c] = matrix[r][c];
		}
	}
}

std::string
PxTransform::toString(void)
{
	std::ostringstream oss(std::ostringstream::out);

	oss << "[";
	oss.setf(std::ios::fixed, std::ios::floatfield);
	oss.precision(3);
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			oss << " " << matrix[r][c];
		}
		oss << ";";
	}
	oss << "]";

	return oss.str();
}

cv::Mat
PxTransform::toCvMat(void)
{
	cv::Mat m = cv::Mat::eye(4, 4, CV_64F);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m.at<double>(i,j) = matrix[i][j];
		}
	}

	return m;
}

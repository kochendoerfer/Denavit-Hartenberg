#pragma once

#pragma once
#include"opencv2\core\core.hpp"
#include <math.h>


//normalizes angles according to constrictions
void normalizeAngles(cv::Vec3d &vec)
{
	//x, alpha, phi
	if (-CV_PI <= vec[0])
		vec[0] += 2 * CV_PI;
	if (CV_PI < vec[0])
		vec[0] -= 2 * CV_PI;
	//y, beta, theta
	if (-CV_PI / 2 <= vec[1])
		vec[1] += CV_PI;
	if (CV_PI / 2 < vec[1])
		vec[1] -= CV_PI;
	//z, gamma, psi
	if (-CV_PI <= vec[2])
		vec[2] += 2 * CV_PI;
	if (CV_PI <= vec[2])
		vec[2] -= 2 * CV_PI;
}
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
	if (R.cols == R.rows && R.cols == 3)
	{
		cv::Mat Rt;
		transpose(R, Rt);
		cv::Mat shouldBeIdentity = Rt * R;
		cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
		return  norm(I, shouldBeIdentity) < 1e-6;
	}
	return false;
}
// Calculates rotation matrix to euler angles
cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If
	double x, y, z;
	if (!singular)

	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RxRyRz(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r02 = R.at<double>(0, 2);
	if (r02 < 1)
	{
		if (r02 > -1)
		{
			y = asin(-R.at<double>(0, 2));
			x = atan2(-R.at<double>(1, 2), R.at<double>(2, 2));
			z = atan2(-R.at<double>(0, 1), R.at<double>(0, 0));
		}
		else  //r20 = -1 first singularity
		{
			y = -CV_PI / 2;
			x = -atan2(R.at<double>(1, 0), R.at<double>(1, 1));
			z = 0;
		}
	}
	else  //r20 = +1 second singularity
	{
		y = +CV_PI / 2;
		x = atan2(R.at<double>(1, 0), R.at<double>(1, 1));
		z = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RxRzRy(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r01 = R.at<double>(0, 1);
	if (r01 < 1)
	{
		if (r01 > -1)
		{
			z = asin(-R.at<double>(0, 1));
			x = atan2(R.at<double>(2, 1), R.at<double>(1, 1));
			y = atan2(R.at<double>(0, 2), R.at<double>(0, 0));
		}
		else  //r20 = -1 first singularity
		{
			z = +CV_PI / 2;
			x = -atan2(-R.at<double>(2, 0), R.at<double>(2, 2));
			y = 0;
		}
	}
	else  //r20 = +1 second singularity
	{
		z = -CV_PI / 2;
		x = atan2(-R.at<double>(2, 0), R.at<double>(2, 2));
		y = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RyRxRz(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r12 = R.at<double>(1, 2);
	if (r12 < 1)
	{
		if (r12 > -1)
		{
			x = asin(-R.at<double>(1, 2));
			y = atan2(R.at<double>(0, 2), R.at<double>(2, 2));
			z = atan2(R.at<double>(1, 0), R.at<double>(1, 1));
		}
		else  //r20 = -1 first singularity
		{
			x = +CV_PI / 2;
			y = -atan2(-R.at<double>(0, 1), R.at<double>(0, 0));
			z = 0;
		}
	}
	else  //r20 = +1 second singularity
	{
		x = -CV_PI / 2;
		y = atan2(-R.at<double>(0, 1), R.at<double>(0, 0));
		z = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RyRzRx(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r10 = R.at<double>(1, 0);
	if (r10 < 1)
	{
		if (r10 > -1)
		{
			z = asin(R.at<double>(1, 0));
			y = atan2(-R.at<double>(2, 0), R.at<double>(0, 0));
			x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		}
		else  //r20 = -1 first singularity
		{
			z = -CV_PI / 2;
			y = -atan2(-R.at<double>(2, 1), R.at<double>(2, 2));
			x = 0;
		}
	}
	else  //r20 = +1 second singularity
	{
		z = +CV_PI / 2;
		y = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		x = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RzRxRy(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r21 = R.at<double>(2, 1);
	if (r21 < 1)
	{
		if (r21 > -1)
		{
			x = asin(R.at<double>(2, 1));
			z = atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
			y = atan2(-R.at<double>(2, 0), R.at<double>(2, 2));
		}
		else  //r21 = -1 first singularity
		{
			x = -CV_PI / 2;
			z = -atan2(R.at<double>(0, 2), R.at<double>(0, 0));
			y = 0;
		}
	}
	else  //r21 = +1 second singularity
	{
		x = +CV_PI / 2;
		z = atan2(R.at<double>(0, 2), R.at<double>(0, 0));
		y = 0;
	}
	cv::Vec3d retVal(z, y, x);
	//normalizeAngles(retVal);
	return retVal;
}

cv::Vec3d RzRyRx(cv::Mat &R)
{
	assert(isRotationMatrix(R));
	double x, y, z;
	double r20 = R.at<double>(2, 0);
	if (r20 < 1)
	{
		if (r20 > -1)
		{
			y = asin(-R.at<double>(2, 0));
			z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
			x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		}
		else  //r20 = -1 first singularity
		{
			y = +CV_PI / 2;
			z = -atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
			x = 0;
		}
	}
	else  //r20 = +1 second singularity
	{
		y = -CV_PI / 2;
		z = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		x = 0;
	}

	cv::Vec3d retVal(z, y, x);
	normalizeAngles(retVal);
	return retVal;
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
	cv::Mat euler(3, 1, CV_64F);

	double m00 = rotationMatrix.at<double>(0, 0);
	double m02 = rotationMatrix.at<double>(0, 2);
	double m10 = rotationMatrix.at<double>(1, 0);
	double m11 = rotationMatrix.at<double>(1, 1);
	double m12 = rotationMatrix.at<double>(1, 2);
	double m20 = rotationMatrix.at<double>(2, 0);
	double m22 = rotationMatrix.at<double>(2, 2);

	double bank, attitude, heading;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		bank = 0;
		attitude = CV_PI / 2;
		heading = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		bank = 0;
		attitude = -CV_PI / 2;
		heading = atan2(m02, m22);
	}
	else
	{
		bank = atan2(-m12, m11);
		attitude = asin(m10);
		heading = atan2(-m20, m00);
	}

	euler.at<double>(0) = bank;
	euler.at<double>(1) = heading;
	euler.at<double>(2) = attitude;

	return euler;
}
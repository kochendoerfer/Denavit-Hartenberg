#pragma once
#include <iostream>
#include"opencv2\core\core.hpp"
#include <math.h>


cv::Mat getTransform(double * theta, double * alpha, double * a, double * d, int n)
{
	//convert to radians
	for (int i = 0; i < n; i++)
	{
		theta[i] *= CV_PI / 180;
		alpha[i] *= CV_PI / 180;
	}
	double angle = 180 * CV_PI / 180;
	double dataCorrect[16] = { cos(angle), -sin(angle), 0, 0,
						sin(angle), cos(angle), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1 };
	cv::Mat MC(4, 4, CV_64F, dataCorrect);
	cv::Mat Mres;
	MC.copyTo(Mres);
	for (int i = 1; i <= n; i++)
	{
		double data[16] = { cos(theta[n-i]), -sin(theta[n-i]) * cos(alpha[n-i]), sin(theta[n-i]) * sin(alpha[n-i]), a[n-i] * cos(theta[n-i]),
							sin(theta[n-i]), cos(theta[n-i]) * cos(alpha[n-i]), -cos(theta[n-i]) * sin(alpha[n-i]), a[n-i] * sin(theta[n-i]),
							0, sin(alpha[n-i]), cos(alpha[n-i]), d[n-i],
							0, 0, 0 , 1 };
		cv::Mat M(4, 4, CV_64F, data);
		Mres = M * Mres;
	}
	return  Mres;
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
	cv::Mat euler(3, 1, CV_64F);

	double sy = sqrt(pow(rotationMatrix.at<double>(0, 0), 2) + pow(rotationMatrix.at<double>(1, 0), 2));
	bool singular = sy < 0.1e-12;
	euler.at<double>(0) = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
	euler.at<double>(1) = atan2(-rotationMatrix.at<double>(2, 0), sy);
	euler.at<double>(2) = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
	if (singular)
	{
		euler.at<double>(0) = atan2(-rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(1, 1));
		euler.at<double>(1) = atan2(-rotationMatrix.at<double>(2, 0), sy);
		euler.at<double>(2) = 0;
	}
	double tmp = euler.at<double>(0);
	euler.at<double>(0) = euler.at<double>(2);
	euler.at<double>(2) = tmp;
	return euler;
}

int main(int argc, char* argv[])
{
	double orgData[4] = { 0, 0, 0, 1 };
	cv::Mat origin = cv::Mat(4, 1, CV_64F);
	origin.data = (uchar*)&orgData;
	//angle by which joints are turned
	double theta[6] = { -1 * 73.05,
							-81.29,
							(73.51 - 90),
							-70.09,
							44.35,
							-89.42 };
	//lengths of turning joint
	double d[6] = {675, 0, 0, -670, 0, -158};
	//lengths of arm on joint
	double a[6] = {260, 680, -35, 0, 0, 0};
	//angle between z-axis of adjacent joints
	double alpha[6] = { 270,
						0,
						90,
						270,
						90,
						180 };

	cv::Mat Mges = getTransform(theta, alpha, a, d, 6);
	std::cout << "XYZ: " << Mges * origin << std::endl << std::endl << std::endl;
	Mges.setTo(0, abs(Mges) < 0.00001);
	//std::cout << "result: " << std::endl << res << std::endl << std::endl << std::endl;

	cv::Rect srcRect(cv::Point(0, 0), cv::Point(3,3));
	cv::Mat rotOnly;
	Mges(srcRect).copyTo(rotOnly);
	cv::Mat euler = rot2euler(rotOnly);
	euler = euler * 180 / CV_PI;
	std::cout << "ABC: " << euler << std::endl << std::endl;
	int i = 0;
	std::cin >> i;
	return 0;
}
#pragma once
#include <iostream>
#include"opencv2\core\core.hpp"
#include <math.h>


//cv::Mat getTransform(double * theta, double * alpha, double * a, double * d)
//{
//	double data0[16] = { cos(theta[0]), -sin(theta[0]) * cos(alpha[0]), sin(theta[0]) * sin(alpha[0]), a[0] * cos(theta[0]),
//						sin(theta[0]), cos(theta[0]) * cos(alpha[0]), -cos(theta[0]) * sin(alpha[0]), a[0] * sin(theta[0]),
//						0, sin(alpha[0]), cos(alpha[0]), d[0],
//						0, 0, 0 , 1 };
//	cv::Mat M1(4, 4, CV_64F, data0);
//
//	//for transformation T1 -> T2
//	double data2[16] = { cos(theta[1]), -sin(theta[1]) * cos(alpha[1]), sin(theta[1]) * sin(alpha[1]), a[1] * cos(theta[1]),
//						sin(theta[1]), cos(theta[1]) * cos(alpha[1]), -cos(theta[1]) * sin(alpha[1]), a[1] * sin(theta[1]),
//						0, sin(alpha[1]), cos(alpha[1]), d[1],
//						0, 0, 0 , 1 };
//	cv::Mat M2(4, 4, CV_64F, data2);
//
//	//for transformation T2 -> T3
//	double data3[16] = { cos(theta[2]), -sin(theta[2]) * cos(alpha[2]), sin(theta[2]) * sin(alpha[2]), a[2] * cos(theta[2]),
//						sin(theta[2]), cos(theta[2]) * cos(alpha[2]), -cos(theta[2]) * sin(alpha[2]), a[2] * sin(theta[2]),
//						0, sin(alpha[2]), cos(alpha[2]), d[2],
//						0, 0, 0 , 1 };
//	cv::Mat M3(4, 4, CV_64F, data3);
//
//	//for transformation T3 -> T4
//	double data4[16] = { cos(theta[3]), -sin(theta[3]) * cos(alpha[3]), sin(theta[3]) * sin(alpha[3]), a[3] * cos(theta[3]),
//						sin(theta[3]), cos(theta[3]) * cos(alpha[3]), -cos(theta[3]) * sin(alpha[3]), a[3] * sin(theta[3]),
//						0, sin(alpha[3]), cos(alpha[3]), d[3],
//						0, 0, 0 , 1 };
//	cv::Mat M4(4, 4, CV_64F, data4);
//
//	//for transformation T4 -> T5
//	double data5[16] = { cos(theta[4]), -sin(theta[4]) * cos(alpha[4]), sin(theta[4]) * sin(alpha[4]), a[4] * cos(theta[4]),
//						sin(theta[4]), cos(theta[4]) * cos(alpha[4]), -cos(theta[4]) * sin(alpha[4]), a[4] * sin(theta[4]),
//						0, sin(alpha[4]), cos(alpha[4]), d[4],
//						0, 0, 0 , 1 };
//	cv::Mat M5(4, 4, CV_64F, data5);
//
//	//for transformation T5 -> T6
//	double data6[16] = { cos(theta[5]), -sin(theta[5]) * cos(alpha[5]), sin(theta[5]) * sin(alpha[5]), a[5] * cos(theta[5]),
//						sin(theta[5]), cos(theta[5]) * cos(alpha[5]), -cos(theta[5]) * sin(alpha[5]), a[5] * sin(theta[5]),
//						0, sin(alpha[5]), cos(alpha[5]), d[5],
//						0, 0, 0 , 1 };
//	cv::Mat M6(4, 4, CV_64F, data6);
//
//	double angle = 180 * CV_PI / 180;
//	//test transformation
//	double dataCorrect[16] = { cos(angle), -sin(angle), 0, 0,
//						sin(angle), cos(angle), 0, 0,
//						0, 0, 1, 0,
//						0, 0, 0, 1 };
//	cv::Mat MC(4, 4, CV_64F, dataCorrect);
//	cv::Mat Mges = M1 * M2 * M3 * M4 * M5 * M6 * MC;
//	return  Mges;
//}

cv::Mat getTransform(double * theta, double * alpha, double * a, double * d, int n)
{
	//convert to radians
	for (int i = 0; i < n; i++)
	{
		theta[i] *= CV_PI / 180;
		alpha[i] *= CV_PI / 180;
	}

	std::vector<cv::Mat> transforms(n);
	for (int i = 0; i < n; i++)
	{
		double data[16] = { cos(theta[i]), -sin(theta[i]) * cos(alpha[i]), sin(theta[i]) * sin(alpha[i]), a[i] * cos(theta[i]),
							sin(theta[i]), cos(theta[i]) * cos(alpha[i]), -cos(theta[i]) * sin(alpha[i]), a[i] * sin(theta[i]),
							0, sin(alpha[i]), cos(alpha[i]), d[i],
							0, 0, 0 , 1 };
		cv::Mat M(4, 4, CV_64F, data);
		M.copyTo(transforms.at(i));
	}

	double angle = 180 * CV_PI / 180;
	double dataCorrect[16] = { cos(angle), -sin(angle), 0, 0,
						sin(angle), cos(angle), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1 };
	cv::Mat MC(4, 4, CV_64F, dataCorrect);
	cv::Mat Mres = transforms.at(0) * transforms.at(1) * transforms.at(2) * transforms.at(3) * transforms.at(4) * transforms.at(5) * MC;
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
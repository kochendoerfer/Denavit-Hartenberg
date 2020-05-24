#pragma once
#include <iostream>
#include "AngleVariants.h"

cv::Mat getTransform(double * theta, double * alpha, double * a, double * d)
{
	double data0[16] = { cos(theta[0]), -sin(theta[0]) * cos(alpha[0]), sin(theta[0]) * sin(alpha[0]), a[0] * cos(theta[0]),
						sin(theta[0]), cos(theta[0]) * cos(alpha[0]), -cos(theta[0]) * sin(alpha[0]), a[0] * sin(theta[0]),
						0, sin(alpha[0]), cos(alpha[0]), d[0],
						0, 0, 0 , 1 };
	cv::Mat M1(4, 4, CV_64F, data0);

	//for transformation T1 -> T2
	double data2[16] = { cos(theta[1]), -sin(theta[1]) * cos(alpha[1]), sin(theta[1]) * sin(alpha[1]), a[1] * cos(theta[1]),
						sin(theta[1]), cos(theta[1]) * cos(alpha[1]), -cos(theta[1]) * sin(alpha[1]), a[1] * sin(theta[1]),
						0, sin(alpha[1]), cos(alpha[1]), d[1],
						0, 0, 0 , 1 };
	cv::Mat M2(4, 4, CV_64F, data2);

	//for transformation T2 -> T3
	double data3[16] = { cos(theta[2]), -sin(theta[2]) * cos(alpha[2]), sin(theta[2]) * sin(alpha[2]), a[2] * cos(theta[2]),
						sin(theta[2]), cos(theta[2]) * cos(alpha[2]), -cos(theta[2]) * sin(alpha[2]), a[2] * sin(theta[2]),
						0, sin(alpha[2]), cos(alpha[2]), d[2],
						0, 0, 0 , 1 };
	cv::Mat M3(4, 4, CV_64F, data3);

	//for transformation T3 -> T4
	double data4[16] = { cos(theta[3]), -sin(theta[3]) * cos(alpha[3]), sin(theta[3]) * sin(alpha[3]), a[3] * cos(theta[3]),
						sin(theta[3]), cos(theta[3]) * cos(alpha[3]), -cos(theta[3]) * sin(alpha[3]), a[3] * sin(theta[3]),
						0, sin(alpha[3]), cos(alpha[3]), d[3],
						0, 0, 0 , 1 };
	cv::Mat M4(4, 4, CV_64F, data4);
	//for transformation T4 -> T5
	double data5[16] = { cos(theta[4]), -sin(theta[4]) * cos(alpha[4]), sin(theta[4]) * sin(alpha[4]), a[4] * cos(theta[4]),
						sin(theta[4]), cos(theta[4]) * cos(alpha[4]), -cos(theta[4]) * sin(alpha[4]), a[4] * sin(theta[4]),
						0, sin(alpha[4]), cos(alpha[4]), d[4],
						0, 0, 0 , 1 };
	cv::Mat M5(4, 4, CV_64F, data5);

	//for transformation T5 -> T6
	double data6[16] = { cos(theta[5]), -sin(theta[5]) * cos(alpha[5]), sin(theta[5]) * sin(alpha[5]), a[5] * cos(theta[5]),
						sin(theta[5]), cos(theta[5]) * cos(alpha[5]), -cos(theta[5]) * sin(alpha[5]), a[5] * sin(theta[5]),
						0, sin(alpha[5]), cos(alpha[5]), d[5],
						0, 0, 0 , 1 };
	cv::Mat M6(4, 4, CV_64F, data6);
	return  M1 * M2 * M3 * M4 * M5 * M6;
}

int main(int argc, char* argv[])
{
	//from origin
	double orgData[4] = { 0, 0, 0, 1 };
	cv::Mat origin = cv::Mat(4, 1, CV_64F);
	origin.data = (uchar*)&orgData;

	//theta is the angle fo rotational axes
	double theta[6] = {-1 * 0 * CV_PI / 180 ,
							0 * CV_PI / 180,
							(0 - 90) * CV_PI / 180,
							0 * CV_PI / 180,
							0 * CV_PI / 180,
							0 * CV_PI / 180 };

	//variables that dont change for the system
	//lengths of turning joint
	double d[6] = {675, 0, 0, -670, 0, -158};
	//lengths of arm on joint
	double a[6] = {260, 680, -35, 0, 0, 0};

	//angles for transformations of z-axes
	double alpha[6] = { 270 * CV_PI / 180 ,
						0 * CV_PI / 180,
						90 * CV_PI / 180,
						270 * CV_PI / 180,
						90 * CV_PI / 180,
						180 * CV_PI / 180 };
	

	//test transformation
	double dataCorrect[16] = { cos(180), -sin(180), 0, 0,
						sin(180), cos(180), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1 };
	cv::Mat MC(4, 4, CV_64F, dataCorrect);

	cv::Mat Mges = getTransform(theta, alpha, a, d);
	Mges = MC * Mges;
	cv::Mat res = (Mges * origin);
	//Mges.setTo(0, abs(Mges) < 0.00001);
	std::cout << "result: " << std::endl << res << std::endl << std::endl << std::endl;

	cv::Rect srcRect(cv::Point(0, 0), cv::Point(3,3));
	cv::Mat rotOnly;
	Mges(srcRect).copyTo(rotOnly);
	cv::Vec3d angles1, angles2, angles3, angles4, angles5, angles6;
	if (isRotationMatrix(rotOnly))
	{
		angles1 = RxRyRz(rotOnly);
		angles1 = angles1 * 180 / CV_PI;
		std::cout << "Angles RxRyRz: " << angles1 << std::endl << std::endl;
		angles2 = RxRzRy(rotOnly);
		angles2 = angles2 * 180 / CV_PI;
		std::cout << "Angles RxRzRy: " << angles2 << std::endl << std::endl;
		angles3 = RyRxRz(rotOnly);
		angles3 = angles3 * 180 / CV_PI;
		std::cout << "Angles RyRxRz: " << angles3 << std::endl << std::endl;
		angles4 = RyRzRx(rotOnly);
		angles4 = angles4 * 180 / CV_PI;
		std::cout << "Angles RyRzRx: " << angles4 << std::endl << std::endl;
		angles5 = RzRxRy(rotOnly);
		angles5 = angles5 * 180 / CV_PI;
		std::cout << "Angles RzRxRy: " << angles5 << std::endl << std::endl;
		angles6 = RzRyRx(rotOnly);
		angles6 = angles6 * 180 / CV_PI;
		std::cout << "Angles RzRyRx: " << angles6 << std::endl << std::endl;
	}

	int i = 0;
	std::cin >> i;
	return 0;
}
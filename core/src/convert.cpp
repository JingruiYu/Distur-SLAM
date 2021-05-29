/*************************************************************************
	> File Name: src/convert.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时12分32秒
 ************************************************************************/

#include "convert.h"

double convert::normalize_angle(double theta)
{
	if (theta >= -M_PI && theta < M_PI)
		return theta;

	double multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}

SE2 convert::toSE2(const cv::Mat &cvT)
{
	double yaw = std::atan2(cvT.at<float>(1,0), cvT.at<float>(0,0));
    double theta = normalize_angle(yaw);
    double x = cvT.at<float>(0,2);
    double y = cvT.at<float>(1,2);

	return SE2(x,y,theta);
}

Eigen::Matrix4d convert::toMatrix4d(const SE2 &se2T)
{
	double c = cos(se2T.theta);
    double s = sin(se2T.theta);

    Eigen::Matrix4d mat;
    mat <<  c,-s, 0, se2T.x,
            s, c, 0, se2T.y,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return mat;
}

Eigen::Matrix4d convert::toMatrix4d(const cv::Mat &cvT)
{
    Eigen::Matrix4d mat;
    mat <<  cvT.at<float>(0,0),cvT.at<float>(0,1), 0, cvT.at<float>(0,2),
            cvT.at<float>(1,0),cvT.at<float>(1,1), 0, cvT.at<float>(1,2),
            0, 0, 1, 0,
            0, 0, 0, 1;
			
    return mat;
}
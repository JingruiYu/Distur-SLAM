/*************************************************************************
	> File Name: include/SE2.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时24分50秒
 ************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class SE2
{
public:
	SE2(){}
	SE2(double _x, double _y, double _theta);
	SE2(const cv::Mat &cvT);
	~SE2();

	Eigen::Matrix4d toMatrix4d();
public:
	double x, y, theta;
};



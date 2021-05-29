/*************************************************************************
	> File Name: include/convert.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时12分19秒
 ************************************************************************/

#pragma once

#include "SE2.h"
#include "Frame.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class convert
{
public:
	static double normalize_angle(double theta);
	static SE2 toSE2(const cv::Mat &cvT);

	static Eigen::Matrix4d toMatrix4d(const SE2 &se2T);
	static Eigen::Matrix4d toMatrix4d(const cv::Mat &cvT);
};
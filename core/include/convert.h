/*************************************************************************
	> File Name: include/convert.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时12分19秒
 ************************************************************************/

#pragma once

#include "SE2.h"
#include "Frame.h"
#include "config.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class convert
{
public:
	static double normalize_angle(double theta);

	static SE2 toSE2(const cv::Mat &cvT);
	static SE2 toSE2(const Eigen::Matrix3d &cvT);

	static cv::Mat tocvMat(const SE2 &se2T);
	static cv::Mat tocvRMat(float theta);
	static cv::Mat tocvMat(cv::Point2f &p);

	static Eigen::Matrix3d toMatrix3d(const SE2 &se2T);
	static Eigen::Matrix4d toMatrix4d(const SE2 &se2T);
	static Eigen::Matrix4d toMatrix4d(const cv::Mat &cvT);

	static void mat2vector(const cv::Mat &img, std::vector<float> &vimg);

	static cv::Point2f BirdviewPT2XY(const cv::Point2f& pt);
	static cv::Point2f XY2BirdviewPT(const cv::Point2f& pt);
};
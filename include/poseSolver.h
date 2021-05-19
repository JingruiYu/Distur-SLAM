/*************************************************************************
	> File Name: include/poseSolver.h
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时57分29秒
 ************************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

class poseSolver
{
public:
	poseSolver();
	~poseSolver();

	static cv::Point3f birdPixel2Camera(cv::Point2f pt);
	static cv::Point2f birdCamera2Pixel(cv::Point3f pt);

	static cv::Mat PnP3Dwith2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2);

public:
	static const float bRow;
	static const float bCol;
	static const float cor;
	static const float p2m;
	static const float m2p;
	static const float rear;
};


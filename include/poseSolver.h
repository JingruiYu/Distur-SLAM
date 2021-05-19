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

	static cv::Mat PnP3Dwith2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2);

public:

};


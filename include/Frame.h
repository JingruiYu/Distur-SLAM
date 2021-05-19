/*************************************************************************
	> File Name: include/Frame.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 15时53分48秒
 ************************************************************************/

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class Frame
{
public:
	Frame(int _idx, cv::Mat &_img, double _timestamp);
	~Frame();

	void setTwc(cv::Mat &Twc);
public:
	cv::Mat img;
	std::vector<cv::Mat> img_pyr;
	cv::Mat Twc;

	int idx;
	double timestamp;
};



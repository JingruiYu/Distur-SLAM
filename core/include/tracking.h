/*************************************************************************
	> File Name: include/tracking.h
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 16时14分03秒
 ************************************************************************/

#pragma once

#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <iostream>


class tracking
{
public:
	tracking(/* args */);
	~tracking();

	static std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > LK(Frame* refFrame, Frame* curFrame, bool doubleCheck);
	static std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > FeatureORB(Frame* refFrame, Frame* curFrame);
};


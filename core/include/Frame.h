/*************************************************************************
	> File Name: include/Frame.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 15时53分48秒
 ************************************************************************/

#pragma once

#include "SE2.h"
#include "convert.h"
#include <opencv2/opencv.hpp>
#include <iostream>

class Frame
{
public:
	Frame(int _idx, cv::Mat &_img, double _timestamp, cv::Vec3d _gtPose);
	~Frame();

	bool extractFastPoint();
	void setTwc(cv::Mat &Twc);
	
public:
	cv::Mat img;
	cv::Mat img_gray;
	std::vector<cv::Mat> img_pyr;
	
	SE2 se2Twc;
	cv::Mat Twc;
	SE2 mGtPose;

	struct bpoint
	{
	public:
		float ou,ov;
		float u,v;
		float s,r;
		float p2m,m2p;
		float X,Y;
		float pa,pb;

		bpoint(float _ou, float _ov, float _rows, float _cols) : ou(_ou), ov(_ov) 
		{
			v = _rows * 0.5 - ov;
			u = _cols * 0.5 - ou;
			s = 1.0;
			r = 1.393;
			p2m = 0.03984;
			m2p = 25.1;
			updateXY();

			pa = 1.0;
			pb = 0.0;
		}

		void updateXY()
		{
			X = s * p2m * v + r;
			Y = s * p2m * u;
		}
	};

	std::vector<cv::Point2f> vPoint2fs;
	std::vector<bpoint> vbPoints;

	int idx;
	double timestamp;

	float rows, cols;
};



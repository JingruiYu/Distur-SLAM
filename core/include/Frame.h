/*************************************************************************
	> File Name: include/Frame.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 15时53分48秒
 ************************************************************************/

#pragma once

#include "SE2.h"
#include "convert.h"
#include "FeatureLine.h"
#include "LineExtractor.h"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <iostream>

class Frame
{
public:
	Frame(int _idx, cv::Mat &_img, cv::Mat &_img_mask, double _timestamp, cv::Vec3d _gtPose);
	~Frame();

	void setTwc(cv::Mat &Twc);

	bool extractFastPoint();
	bool setMappoints(std::vector<cv::Point2f>& _vPoints);
	
	bool GetMajorLine(birdview::Line& line) const;
    void SetMajorLine(const birdview::Line& line);
	void setKeyLines(std::vector<KeyLine>& _vKeyLines, std::vector<bool>& _status);
	std::vector<cv::Point2f> getMiddlePtFromLine();
	std::vector<cv::Point2f> getEndPtFromLine();

public:
	cv::Mat img;
	cv::Mat img_gray;
	cv::Mat img_mask;
	std::vector<cv::Mat> img_pyr;
	
	SE2 se2Twc;
	cv::Mat Twc;
	SE2 mGtPose;

	birdview::Line mMajorLine;  // in XY coordinate
    bool mbIsMajorLineSet;

	struct bpoint
	{
	public:
		float ou,ov;
		float u,v;
		float s,r;
		float p2m,m2p;
		float X,Y;
		float pa,pb;

		bpoint(){}
		bpoint(float _ou, float _ov) : ou(_ou), ov(_ov) 
		{
			v = config::birdviewRows * 0.5 - ov;
			u = config::birdviewCols * 0.5 - ou;
			s = 1.0;
			r = config::rear_axle_to_center;
			p2m = config::pixel2meter;
			m2p = config::meter2pixel;
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

	std::vector<KeyLine> vKeyLines;

	int idx;
	double timestamp;
};



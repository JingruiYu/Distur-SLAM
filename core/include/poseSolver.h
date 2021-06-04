/*************************************************************************
	> File Name: include/poseSolver.h
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时57分29秒
 ************************************************************************/

#pragma once

#include "Frame.h"
#include <opencv2/opencv.hpp>

class poseSolver
{
public:
	poseSolver();
	~poseSolver();

	static cv::Mat ICP2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2);
	static int FindRtICP2D(const std::vector<cv::Point2f> &vKeys1, const std::vector<cv::Point2f> &vKeys2, const std::vector<cv::DMatch> &vMatches,
                    		std::vector<bool> &vbMatchesInliers, cv::Mat &R, cv::Mat &t, float sigma = 1.0, int mMaxIterations=200);
	static bool ComputeRtICP2D(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2, cv::Mat &R, cv::Mat &t);
	static int CheckRtICP2D(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::Point2f> &vP2D1, const std::vector<cv::Point2f> &vP2D2,
                			const std::vector<cv::DMatch> &vMatches12, std::vector<bool> &vbMatchesInliers, float sigma);

	static cv::Mat FindtICP2D(const std::vector<cv::Point2f> &vKeysXY1, const std::vector<cv::Point2f> &vKeysXY2, Frame* lastFrame, Frame* curFrame, float cur_theta);

public:
	static const float bRow;
	static const float bCol;
	static const float cor;
	static const float p2m;
	static const float m2p;
	static const float rear;
};


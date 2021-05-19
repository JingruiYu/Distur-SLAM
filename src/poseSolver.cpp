/*************************************************************************
	> File Name: src/poseSolver.cpp
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时58分06秒
 ************************************************************************/

#include "poseSolver.h"


poseSolver::poseSolver(/* args */)
{
}

poseSolver::~poseSolver()
{
}

cv::Mat poseSolver::PnP3Dwith2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2)
{
	cv::Mat Twc = cv::Mat::eye(4,4,CV_32FC1);

	return Twc;
}
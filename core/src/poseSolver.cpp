/*************************************************************************
	> File Name: src/poseSolver.cpp
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时58分06秒
 ************************************************************************/

#include "poseSolver.h"

const float poseSolver::bRow = 384.0;
const float poseSolver::bCol = 384.0;
const float poseSolver::cor = 1;
const float poseSolver::p2m = 0.03984*cor;
const float poseSolver::m2p = 25.1/cor;
const float poseSolver::rear = 1.393;

poseSolver::poseSolver(/* args */)
{
}

poseSolver::~poseSolver()
{
}

cv::Point3f poseSolver::birdPixel2Camera(cv::Point2f pt)
{
	cv::Point3f p;
    p.x = (bRow/2-pt.y)*p2m+rear;
    p.y = (bCol/2-pt.x)*p2m;
    p.z = 0;

    return p;
}

cv::Point2f poseSolver::birdCamera2Pixel(cv::Point3f p)
{
	cv::Point2f pt;
    pt.x = bCol/2-p.y*m2p;
    pt.y = bRow/2-(p.x-rear)*m2p;
    return pt;
}

cv::Mat poseSolver::PnP3Dwith2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2)
{
	
	cv::Mat Twc = cv::Mat::eye(4,4,CV_32FC1);

	return Twc;
}
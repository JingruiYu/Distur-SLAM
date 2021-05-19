/*************************************************************************
	> File Name: src/disSLAM.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "disSLAM.h"

disSLAM::disSLAM(/* args */)
{
    mpMap = new map();
    std::cout << "hello disSLAM " << std::endl;
}

disSLAM::~disSLAM()
{
}

void disSLAM::TrackwithOF(int _idx, cv::Mat &_img, double _timestamp)
{
    curFrame = new Frame(_idx, _img, _timestamp);
    if (!lastFrame)
    {
        cv::Mat Twc = cv::Mat::eye(4,4,CV_32FC1);
        curFrame->setTwc(Twc);
        mpMap->addFrame(curFrame);
        lastFrame = curFrame;
        return;
    }

    // std::cout << "cur_pyr: " << curFrame->img_pyr[0].size() << std::endl;
    // std::cout <<  "last_pyr: " << lastFrame->img_pyr[0].size() << std::endl;
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > kpts1_kpts2 = tracking::LK(lastFrame,curFrame,false);

    cv::Mat Twc = poseSolver::PnP3Dwith2D(kpts1_kpts2);

    curFrame->setTwc(Twc);
    mpMap->addFrame(curFrame);

    lastFrame = curFrame;
    return;
}
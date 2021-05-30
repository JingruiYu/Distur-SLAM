/*************************************************************************
	> File Name: src/disSLAM.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "disSLAM.h"
#include "keyFrame.h"
#include "optimizer.h"

disSLAM::disSLAM(/* args */)
{
    mpMap = new map();

    view::viewerConfig vfg;
    mpViewer = new view("viewer",vfg,mpMap);
    std::cout << "hello disSLAM " << std::endl;
    mpViewer->createWindow();
    // viewer_thread = std::thread(&view::run,mpViewer);
    // mpViewer->run();
}

disSLAM::~disSLAM()
{
}

void disSLAM::TrackwithOF(int _idx, cv::Mat &_img, double _timestamp, cv::Vec3d _gtPose)
{
    curFrame = new Frame(_idx, _img, _timestamp, _gtPose);
    if (!lastFrame)
    {
        cv::Mat Twc = cv::Mat::eye(3,3,CV_32FC1);
        curFrame->setTwc(Twc);
        mpMap->addFrame(curFrame);
        lastFrame = curFrame;
        return;
    }

    // std::cout << "cur_pyr: " << curFrame->img_pyr[0].size() << std::endl;
    // std::cout <<  "last_pyr: " << lastFrame->img_pyr[0].size() << std::endl;
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > kpts1_kpts2 = tracking::LK(lastFrame,curFrame,false);

    cv::Mat img_show = _img.clone();
    std::vector<cv::Point2f> curKeyPt = kpts1_kpts2.second;
    curFrame->setMappoints(curKeyPt);
    for(auto kp:curKeyPt)
    {
        cv::circle(img_show, kp, 5, cv::Scalar(0, 240, 0), 1);
    }
    // cv::imshow("corners", img_show);
    // cv::waitKey(30);

    cv::Mat Tc1c2 = poseSolver::ICP2D(kpts1_kpts2);
    optimizer::FrameDirectOptimization(lastFrame, curFrame, Tc1c2);
    cv::Mat Twc = lastFrame->Twc * Tc1c2;
    curFrame->setTwc(Twc);

    // keyFrame* curKF = new keyFrame(*curFrame);
    // std::cout << "curKF: " << curKF->mnId << std::endl;
    // std::cout << "curKF: " << curKF->idx << ", curFrame: " << curFrame->idx << std::endl;

    mpMap->addkeyFrame(curFrame);
    mpViewer->runcore(_idx);

    lastFrame = curFrame;
    return;
}
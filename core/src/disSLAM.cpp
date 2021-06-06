/*************************************************************************
	> File Name: src/disSLAM.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "disSLAM.h"
#include "keyFrame.h"
#include "optimizer.h"
#include "lineport.h"
// #include "config.h"


disSLAM::disSLAM(/* args */)
{
    mpMap = new map();

    view::viewerConfig vfg;
    mpViewer = new view("viewer",vfg,mpMap);
    std::cout << "hello disSLAM " << std::endl;

#ifndef macdebugwithoutviewer
    mpViewer->createWindow();
    // viewer_thread = std::thread(&view::run,mpViewer);
    // mpViewer->run();
#endif

    resFile.open("testres.txt");
}

disSLAM::~disSLAM()
{
}

void disSLAM::TrackwithOF(int _idx, cv::Mat &_img, cv::Mat &_img_mask, double _timestamp, cv::Vec3d _gtPose)
{
    curFrame = new Frame(_idx, _img, _img_mask, _timestamp, _gtPose);
    birdview::Line local_line;
    if (config::useLineForRotation)
    {
        lineport::CalculateMajorLine(curFrame,local_line);
    }
    
    if (!lastKF)
    {
        cv::Mat Twc = cv::Mat::eye(3,3,CV_32FC1);
        curFrame->setTwc(Twc);

        if (config::useLineForRotation)
        {
            birdview::Line major_line;
            curFrame->GetMajorLine(major_line);
            mpMap->SetMajorLine(major_line);
        }

        mpMap->addFrame(curFrame);
        lastFrame = curFrame;
        lastKF = new keyFrame(curFrame);
        return;
    }

    // std::cout << "curFrame Id: " << curFrame->idx << ", lastKF frame Id: " << lastKF->mpF->idx << ", KF Id: " << lastKF->mnId << std::endl;

    cv::Mat Twc2;
    if (config::useLineForRotation) // lines
    {
        float cur_theta = mpMap->getRotationViaLine(local_line);
        // Twc2 = poseSolver::FindtICP2D(kpts1_kpts2.first,kpts1_kpts2.second,lastKF->mpF,curFrame,cur_theta);

        Twc2 = poseSolver::FindtByLinePt(lastKF->mpF, curFrame, cur_theta);
    }
    else // feature points
    {
        std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > kpts1_kpts2;
        kpts1_kpts2 = tracking::LK(lastKF->mpF,curFrame,true);
        // kpts1_kpts2 = tracking::FeatureORB(lastKF->mpF,curFrame);
        curFrame->setMappoints(kpts1_kpts2.second);
        mpViewer->showKeyPts(_img,kpts1_kpts2.second);

        cv::Mat Tc1c2 = poseSolver::ICP2D(kpts1_kpts2);
        // optimizer::FrameDirectOptimization(lastKF->mpF, curFrame, Tc1c2);
        // checkT(Tc1c2);
        Twc2 = lastKF->mpF->Twc * Tc1c2;
    }
    curFrame->setTwc(Twc2);

    if (false)
    {
        cv::Mat gtTwc1 = convert::tocvMat(lastKF->mpF->mGtPose);
        cv::Mat gtTcc = gtTwc1.inv() * convert::tocvMat(curFrame->mGtPose);
        
        cv::Mat elTcc = lastKF->mpF->Twc.inv() * Twc2;

        SE2 gtse = convert::toSE2(gtTcc);
        SE2 lise = convert::toSE2(elTcc);

        resFile << _idx << std::endl;
        resFile << "gtse: " << gtse.x << " - " << gtse.y << " - " << gtse.theta << std::endl;
        resFile << "lise: " << lise.x << " - " << lise.y << " - " << lise.theta << std::endl << std::endl;
    }
    // keyFrame* curKF = new keyFrame(*curFrame);
    // std::cout << "curKF: " << curKF->mnId << std::endl;
    // std::cout << "curKF: " << curKF->idx << ", curFrame: " << curFrame->idx << std::endl;
    // mpMap->addkeyFrame(curFrame);
    
    // viewer
#ifndef macdebugwithoutviewer
    mpViewer->runcore(_idx);
#endif

    lastFrame = curFrame;

    if (cKF == 0)
    {
        cKF = 0;
        lastKF = new keyFrame(curFrame);
        mpMap->addkeyFrame(lastKF);
    }
    else
    {
        cKF++;
    }
    
    return;
}

void disSLAM::checkT(cv::Mat &_Tc1c2)
{
    SE2 Tc1c2 = convert::toSE2(_Tc1c2);
    checkT(Tc1c2);
    _Tc1c2 = convert::tocvMat(Tc1c2);
}

void disSLAM::checkT(SE2 &_Tc1c2)
{
    double delta_cam_pos = std::hypot(_Tc1c2.x,_Tc1c2.y);
    double delta_cam_ori = std::abs(_Tc1c2.theta);

    Eigen::Matrix3d curTwc = convert::toMatrix3d(curFrame->mGtPose);
    Eigen::Matrix3d refTwc = convert::toMatrix3d(lastFrame->mGtPose);
    Eigen::Matrix3d Tcrcc = refTwc.inverse() * curTwc;
    SE2 gTcc = convert::toSE2(Tcrcc);

    double delta_odo_pos = std::hypot(gTcc.x,gTcc.y);
    double delta_odo_ori = std::abs(gTcc.theta);

    // std::cout << "delta_cam_pos: " << delta_cam_pos << ", delta_cam_ori: " << delta_cam_ori << std::endl;
    // std::cout << "delta_odo_pos: " << delta_odo_pos << ", delta_odo_ori: " << delta_odo_ori << std::endl;

    double pos_len = delta_odo_pos > delta_cam_pos ? delta_odo_pos : delta_cam_pos;
    double ori_len = delta_odo_ori > delta_cam_ori ? delta_odo_ori : delta_cam_ori;
    
    double pos_dyn = std::abs(delta_odo_pos - delta_cam_pos) / pos_len;
    double ori_dyn = std::abs(delta_odo_ori - delta_cam_ori) / ori_len;
    
    if (pos_dyn > 0.5)
    {
        _Tc1c2.x = gTcc.x;
        _Tc1c2.y = gTcc.y;
    }

    if (ori_dyn > 0.5)
    {
        _Tc1c2.theta = gTcc.theta;
    }
    
    if (delta_cam_pos > 1.5 || delta_cam_ori > 0.6)
    {
        std::cout << " delta_cam_pos: " << delta_cam_pos << ", delta_cam_ori: " << delta_cam_ori << std::endl;
    }
    
    
    // std::cout << "lastFrame: " << lastFrame->idx << ", " << lastFrame->mGtPose.x << ", " << lastFrame->mGtPose.y << ", " << lastFrame->mGtPose.theta << std::endl;
    // std::cout << "curFrame: " << curFrame->idx << ", " << curFrame->mGtPose.x << ", " << curFrame->mGtPose.y << ", " << curFrame->mGtPose.theta << std::endl;
    
}
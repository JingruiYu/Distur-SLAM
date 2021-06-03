/*************************************************************************
	> File Name: src/disSLAM.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "disSLAM.h"
#include "keyFrame.h"
#include "optimizer.h"
#include "line.h"

#define macdebugwithoutviewer

disSLAM::disSLAM(/* args */)
{
    mpMap = new map();

    view::viewerConfig vfg;
    mpViewer = new view("viewer",vfg,mpMap);
    std::cout << "hello disSLAM " << std::endl;

#ifndef macdebugwithoutviewer
    mpViewer->createWindow();
#endif

    // viewer_thread = std::thread(&view::run,mpViewer);
    // mpViewer->run();

    resFile.open("testres.txt");
}

disSLAM::~disSLAM()
{
}

void disSLAM::TrackwithOF(int _idx, cv::Mat &_img, cv::Mat &_img_mask, double _timestamp, cv::Vec3d _gtPose)
{
    curFrame = new Frame(_idx, _img, _img_mask, _timestamp, _gtPose);
    lineport::CalculateMajorLine(curFrame)

    if (!lastFrame)
    {
        cv::Mat Twc = cv::Mat::eye(3,3,CV_32FC1);
        curFrame->setTwc(Twc);

        birdview::Line major_line;
        if(curFrame->GetMajorLine(major_line))
            mpMap->SetMajorLine(major_line);

        mpMap->addFrame(curFrame);
        lastFrame = curFrame;
        return;
    }


    if (true)
    {
        birdview::Line global_line, local_line;
        mpMap->GetMajorLine(global_line);
        curFrame->GetMajorLine(local_line);

        cv::Point2f gDir = global_line.eP - global_line.sP;
        cv::Point2f lDir = local_line.eP - local_line.sP;

        float flag = 1.0;
        if (lDir.y > 0)
        {
            flag = -1.0;
        }
        
        float cos_theta = std::fabs(gDir.dot(lDir)) / (cv::norm(gDir) * cv::norm(lDir));
        float cur_theta = flag * std::acos(cos_theta);
        float cur_angle =  cur_theta / 3.14 * 180;
        float gt_angle = curFrame->mGtPose.theta / 3.14 * 180;

        // std::cout << "global_line: " << global_line.sP << " - " << global_line.eP << std::endl;
        // std::cout << "local_line: " << local_line.sP << " - " << local_line.eP << std::endl;
        // std::cout << "gDir: " << gDir << std::endl;
        // std::cout << "lDir: " << lDir << std::endl;
        // std::cout << "theta: " << theta << std::endl;

        resFile << curFrame->mGtPose.theta << " vs " << cur_theta << " , " << gt_angle << " vs " << cur_angle << " ---- " << " local : sP: " << local_line.sP << " - " << local_line.eP << " , global: " << global_line.sP << " - " << global_line.eP << std::endl;
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
    // cv::imshow("corners", curFrame->img_mask);
    // cv::waitKey(30);

    
    cv::Mat Tc1c2 = poseSolver::ICP2D(kpts1_kpts2);
    // optimizer::FrameDirectOptimization(lastFrame, curFrame, Tc1c2);
    checkT(Tc1c2);
    cv::Mat Twc = lastFrame->Twc * Tc1c2;
    curFrame->setTwc(Twc);

    // keyFrame* curKF = new keyFrame(*curFrame);
    // std::cout << "curKF: " << curKF->mnId << std::endl;
    // std::cout << "curKF: " << curKF->idx << ", curFrame: " << curFrame->idx << std::endl;

    mpMap->addkeyFrame(curFrame);

    // viewer

#ifndef macdebugwithoutviewer
    mpViewer->runcore(_idx);
#endif

    lastFrame = curFrame;
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
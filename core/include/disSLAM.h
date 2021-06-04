
/*************************************************************************
	> File Name: include/disSLAM.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分40秒
 ************************************************************************/

#pragma once

#include "map.h"
#include "Frame.h"
#include "tracking.h"
#include "poseSolver.h"
#include "view.h"
#include "config.h"

#include <opencv2/opencv.hpp>
#include <thread>
#include <fstream>
#include <iostream>

class disSLAM
{
public:
    disSLAM(/* args */);
    ~disSLAM();

    void TrackwithOF(int _idx, cv::Mat &_img, cv::Mat &_img_mask, double _timestamp, cv::Vec3d gtPose);

    void checkT(cv::Mat &Tc1c2);
    void checkT(SE2 &Tc1c2);
    
public:
    Frame *curFrame  = nullptr;
    Frame *lastFrame = nullptr;
    view *mpViewer = nullptr;
    map *mpMap = nullptr;

    std::ofstream resFile;

    std::thread viewer_thread;
};


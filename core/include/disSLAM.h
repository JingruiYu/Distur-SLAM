
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

#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>

class disSLAM
{
public:
    disSLAM(/* args */);
    ~disSLAM();

    void TrackwithOF(int _idx, cv::Mat &_img, double _timestamp, cv::Vec3d gtPose);

public:
    Frame *curFrame  = nullptr;
    Frame *lastFrame = nullptr;
    view *mpViewer = nullptr;
    map *mpMap = nullptr;

    std::thread viewer_thread;
};


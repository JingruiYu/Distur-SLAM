
/*************************************************************************
	> File Name: include/disSLAM.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分40秒
 ************************************************************************/

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "map.h"
#include "Frame.h"
#include "tracking.h"
#include "poseSolver.h"

class disSLAM
{
public:
    disSLAM(/* args */);
    ~disSLAM();

    void TrackwithOF(int _idx, cv::Mat &_img, double _timestamp);

public:
    Frame *curFrame  = nullptr;
    Frame *lastFrame = nullptr;

    map *mpMap = nullptr;
};


/*************************************************************************
	> File Name: include/optimizer.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月30日 星期三 18时24分33秒
 ************************************************************************/

#pragma once

#include "keyFrame.h"
#include "LineExtractor.h"
#include "KeyLineGeometry.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/cubic_interpolation.h>

class optimizer
{
public:
	static void FrameDirectOptimization(Frame* pF1, Frame* pF2, cv::Mat &Tc1c2);
	static void OptimizeMajorLine(const std::vector<KeyLine>& vKeyLines, const std::vector<bool>& vIsParallel, cv::Point3f& le);

};


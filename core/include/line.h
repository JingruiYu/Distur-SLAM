#pragma once

#include "LineExtractor.h"
#include "Frame.h"

class lineport
{
public:
    static bool CalculateMajorLine(const Frame* pF, cv::Point3f& line);
public:
    
};



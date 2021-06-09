/*************************************************************************
	> File Name: src/map.cpp
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时24分41秒
 ************************************************************************/

#include "map.h"
#include <algorithm>

map::map()
{
}

map::~map()
{
}

void map::addFrame(Frame* pF)
{
	vFrame.push_back(pF);
}

void map::addkeyFrame(Frame* pF)
{
	keyFrame* pKF = new keyFrame(pF);

	vkeyFrame.push_back(pKF);
}

void map::addkeyFrame(keyFrame* pKF)
{
	vkeyFrame.push_back(pKF);
}

std::vector<keyFrame*> map::getKeyFrameAll()
{
	return std::vector<keyFrame*>(vkeyFrame.begin(),vkeyFrame.end());
}

std::vector<keyFrame*> map::getLocalKeyFrame(int num)
{
    std::vector<keyFrame*> vlocalKF;

    for (std::vector<keyFrame*>::reverse_iterator it = vkeyFrame.rbegin(); it != vkeyFrame.rbegin()+num && it != vkeyFrame.rend(); it++ )
    {
        keyFrame* pKF = *it;
        vlocalKF.push_back(pKF);
    }
    
    reverse(vlocalKF.begin(),vlocalKF.end());
    
    return vlocalKF;
}

void map::SetMajorLine(const birdview::Line &major_line)
{
    mMajorLine = major_line;
    mbIsMajorLineSet = true;
}

bool map::GetMajorLine(birdview::Line& major_line)
{
    if(!mbIsMajorLineSet)
        return false;
    major_line = mMajorLine;
    return true;
}

float map::getRotationViaLine(birdview::Line &local_line)
{
	birdview::Line global_line = mMajorLine;

	cv::Point2f gDir = global_line.eP - global_line.sP;
    cv::Point2f lDir = local_line.eP - local_line.sP;

    float flag = 1.0;
    if (lDir.y > 0)
    {
        flag = -1.0;
    }
    
    float cos_theta = std::fabs(gDir.dot(lDir)) / (cv::norm(gDir) * cv::norm(lDir));
    float cur_theta = flag * std::acos(cos_theta);
    // float cur_angle =  cur_theta / 3.14 * 180;

	return cur_theta;
}
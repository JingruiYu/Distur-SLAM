/*************************************************************************
	> File Name: src/map.cpp
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时24分41秒
 ************************************************************************/

#include "map.h"

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
	keyFrame* pKF = new keyFrame(*pF);

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
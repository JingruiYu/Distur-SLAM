/*************************************************************************
	> File Name: include/map.h
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时24分33秒
 ************************************************************************/

#pragma once

#include "keyFrame.h"
#include "Frame.h"

class map
{
public:
	map();
	~map();

	void addFrame(Frame* pF);
	void addkeyFrame(Frame* pF);
	void addkeyFrame(keyFrame* pKF);

	std::vector<keyFrame*> getKeyFrameAll();
	
public:
	std::vector<Frame*> vFrame;
	std::vector<keyFrame*> vkeyFrame;
};


/*************************************************************************
	> File Name: include/map.h
	> Author: 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时24分33秒
 ************************************************************************/

#pragma once

#include "Frame.h"

class map
{
public:
	map();
	~map();
	void addFrame(Frame* pF);

public:
	std::vector<Frame*> vFrame;
};


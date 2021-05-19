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
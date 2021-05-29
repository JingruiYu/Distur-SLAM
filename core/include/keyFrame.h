/*************************************************************************
	> File Name: include/keyFrame.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 13时13分46秒
 ************************************************************************/

#pragma once

#include "Frame.h"

class keyFrame : public Frame
{
public:
	keyFrame (const Frame & pF);
	~keyFrame ();

public:
	int mnId;
	static int mnNextId;
	
};


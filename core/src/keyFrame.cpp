/*************************************************************************
	> File Name: src/keyFrame.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 13时13分57秒
 ************************************************************************/

#include "keyFrame.h"

int keyFrame::mnNextId = 0;

keyFrame::keyFrame (Frame* pF) : mpF(pF)
{
	mnId = mnNextId++; 
}

keyFrame::~keyFrame ()
{
}

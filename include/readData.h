/*************************************************************************
	> File Name: include/readData.h
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分40秒
 ************************************************************************/

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>


class readData
{
public:
	readData();
	~readData();

	static bool LoadSAICData(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<std::string> &vstrBirdviewFilenames, 
                std::vector<std::string> &vstrBirdviewMaskFilenames, std::vector<std::string> &vstrBirdviewContourFilenames, 
                std::vector<std::string> &vstrBirdviewContourICPFilenames, std::vector<cv::Vec3d> &vgtPose, std::vector<double> &vTimestamps);
};
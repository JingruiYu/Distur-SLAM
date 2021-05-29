/*************************************************************************
	> File Name: src/readData.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "readData.h"

readData::readData()
{
}

readData::~readData()
{
}

bool readData::LoadSAICData(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<std::string> &vstrBirdviewFilenames, 
                std::vector<std::string> &vstrBirdviewMaskFilenames, std::vector<std::string> &vstrBirdviewContourFilenames,
                std::vector<std::string> &vstrBirdviewContourICPFilenames, std::vector<cv::Vec3d> &vgtPose, std::vector<double> &vTimestamps)
{
    std::ifstream f;
    f.open(strFile.c_str());

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            double x,y,theta;
            std::string image;
            ss >> t;
            vTimestamps.push_back(t);
            ss>>x>>y>>theta;
            vgtPose.push_back(cv::Vec3d(x,y,theta));
            ss >> image;
            vstrImageFilenames.push_back("image/"+image+".jpg");
            vstrBirdviewFilenames.push_back("birdview/"+image+".jpg");
            vstrBirdviewMaskFilenames.push_back("mask/"+image+".jpg");
            vstrBirdviewContourFilenames.push_back("contourICPWrite/"+image+".bmp");
            vstrBirdviewContourICPFilenames.push_back("contourICP/"+image+".jpg");
        }
    }

    return true;
}
/*************************************************************************
	> File Name: test/main.cpp
	> Author: yujr 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时22分31秒
 ************************************************************************/

#include "disSLAM.h"
#include "readData.h"

int main(int argc, char const *argv[])
{
    std::vector<std::string> vImg, vBird, vBirdMask, vBirdContour, vBirdContourICP;
	std::vector<double> vTimestamps;
	std::vector<cv::Vec3d> vgtPose;
	std::string strFile = std::string(argv[1]) + "/groundtruth.txt";

	readData::LoadSAICData(strFile,vImg,vBird,vBirdMask,vBirdContour,vBirdContourICP,vgtPose,vTimestamps);

	disSLAM* mSLAM = new disSLAM();

	for (int i = 0; i < vImg.size(); i++)
	{
		cv::Mat bird = cv::imread(std::string(argv[1])+"/"+vBird[i], CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[i];

		mSLAM->TrackwithOF(i,bird,tframe);
	}
	
	
    return 0;
}

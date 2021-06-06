/*************************************************************************
	> File Name: test/main.cpp
	> Author: yujr 
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时22分31秒
 ************************************************************************/

#include "disSLAM.h"
#include "readData.h"

const double pixel2meter = 0.03984;//*1.737;
const double meter2pixel = 25.1;///1.737;
const double rear_axle_to_center = 1.393;
const double vehicle_length = 4.63;
const double vehicle_width = 1.901;

void applyMaskBirdview(const cv::Mat& src, cv::Mat& dst, const cv::Mat& mask)
{
  dst = src.clone();
  for (int i = 0; i < src.rows; ++i)
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b pixel = mask.at<cv::Vec3b>(i, j);
      if (pixel[0] < 20 && pixel[1] < 20 && pixel[2] < 20)
        dst.at<cv::Vec3b>(i, j) = 0;
    }
}

void ConvertMaskBirdview(const cv::Mat& src, cv::Mat& dst)
{
    if(src.empty())
        return;

    cv::Mat dst_out = cv::Mat(src.rows,src.cols,CV_8UC1);
    for (int i = 0; i < src.rows; ++i)
        for (int j = 0; j < src.cols; ++j)
        {
            cv::Vec3b pixel = src.at<cv::Vec3b>(i, j);
            if (pixel[1] < 20)
                dst_out.at<uchar>(i, j) = 0;
            else
                dst_out.at<uchar>(i, j) = 250;
        }

    dst_out = dst_out > 50;
    int erosion_size = 5;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(dst_out, dst_out, element);

    // preprocess mask, ignore footprint
    int birdviewCols=src.cols;
    int birdviewRows=src.rows;
    double boundary = 15.0;
    double x = birdviewCols / 2 - (vehicle_width / 2 / pixel2meter) - boundary;
    double y = birdviewRows / 2 - (vehicle_length / 2 / pixel2meter) - boundary;
    double width = vehicle_width / pixel2meter + 2 * boundary;
    double height = vehicle_length / pixel2meter + 2 * boundary;
    cv::rectangle(dst_out, cv::Rect(x, y, width, height), cv::Scalar(0),-1);

    dst = dst_out.clone();
}

int main(int argc, char const *argv[])
{
	cv::Mat BirdMask = cv::imread(std::string(argv[1])+"/"+"view_mask.jpg",cv::IMREAD_UNCHANGED);

    std::vector<std::string> vImg, vBird, vBirdMask, vBirdContour, vBirdContourICP;
	std::vector<double> vTimestamps;
	std::vector<cv::Vec3d> vgtPose;
	std::string strFile = std::string(argv[1]) + "/groundtruth.txt";

	readData::LoadSAICData(strFile,vImg,vBird,vBirdMask,vBirdContour,vBirdContourICP,vgtPose,vTimestamps);

	disSLAM* mSLAM = new disSLAM();

	for (size_t i = 0; i < vImg.size(); i++)
	{
		cv::Mat bird = cv::imread(std::string(argv[1])+"/"+vBird[i], cv::IMREAD_UNCHANGED);
		cv::Mat mask = cv::imread(std::string(argv[1])+"/"+vBirdMask[i], cv::IMREAD_UNCHANGED);
		double tframe = vTimestamps[i];
		cv::Vec3d gtPose = vgtPose[i];
		
		cv::Mat mask_all;
		applyMaskBirdview(mask,mask_all,BirdMask);
		cv::Mat birdviewmask = mask_all.clone();
		ConvertMaskBirdview(birdviewmask,birdviewmask);

		mSLAM->TrackwithOF(i,bird,birdviewmask,tframe,gtPose);		
	}
	
	
    return 0;
}

/*************************************************************************
	> File Name: src/tracking.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 16时14分16秒
 ************************************************************************/

#include "tracking.h"

tracking::tracking(/* args */)
{
}

tracking::~tracking()
{
}

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > tracking::LK(Frame* refFrame, Frame* curFrame, bool doubleCheck)
{
	const std::vector<cv::Mat> &im1 = refFrame->img_pyr;
    const std::vector<cv::Mat> &im2 = curFrame->img_pyr;
	int rows = curFrame->rows;
    int cols = curFrame->cols;
	// std::cout << "curFrame->rows : " << curFrame->rows << ", rows: " << rows << std::endl;

	if (!curFrame->extractFastPoint())
	{
		return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
	}

	std::vector<cv::Point2f> kpts1 = curFrame->vPoint2fs;
	std::vector<uchar> status1;
	std::vector<float> errs1;
	std::vector<cv::Point2f> kpts2;
	cv::calcOpticalFlowPyrLK(im1,im2,kpts1,kpts2,status1,errs1,cv::Size(21,21),3);

	if (doubleCheck)
	{
		std::vector<uchar> status2;
		std::vector<float> errs2;
		std::vector<cv::Point2f> kpts11;
		cv::calcOpticalFlowPyrLK(im2,im1,kpts2,kpts11,status2,errs2,cv::Size(21,21),3);

		for (size_t i = 0; i < kpts1.size(); i++)
		{
			cv::Point2f pt1 = kpts1[i];
			cv::Point2f pt2 = kpts2[i];
			cv::Point2f pt11 = kpts11[i];

			if (!status2[i] || cv::norm(pt11-pt1) > 3.0)
			{
				status1[i] = 0;
			}
		}		
	}

	std::vector<cv::Point2f> vPs;
	std::vector<cv::Point2f> vQs;
	for (int i = 0; i < kpts1.size(); i++)
	{
		cv::Point2f pt1 = kpts1[i];
		cv::Point2f pt2 = kpts2[i];

		if (status1[i] && 
			pt2.x >= 0 && pt2.x < cols && pt2.y >= 0 && pt2.y < rows)
		{
			vPs.push_back(pt1);
			vQs.push_back(pt2);
		}
	}

	if (vPs.size() < 10)
	{
		std::cout << "in LK, the final vPs is less than 4 " << std::endl;
        return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
	}
	
	return std::make_pair(vPs, vQs);
}
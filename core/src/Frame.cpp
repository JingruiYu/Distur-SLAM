/*************************************************************************
	> File Name: src/Frame.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 09时30分56秒
 ************************************************************************/

#include "Frame.h"

Frame::Frame(int _idx, cv::Mat &_img, cv::Mat &_img_mask, double _timestamp, cv::Vec3d _gtPose)
{
    idx = _idx;
    timestamp = _timestamp;

    mGtPose = SE2(_gtPose[0], _gtPose[1], _gtPose[2]);

    img_mask = _img_mask.clone();

    rows = _img.rows;
    cols = _img.cols;

    cv::cvtColor(_img, img_gray, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(20.0, cv::Size(8, 8));
    clahe->apply(img_gray, img);
    cv::buildOpticalFlowPyramid(img, img_pyr, cv::Size(21, 21), 3);

    if (_idx % 7 == 0)
    {
        std::cout << "Frame " << _idx << " is init ... " << std::endl;
    }
}

Frame::~Frame()
{
}

bool Frame::extractFastPoint()
{
    if (img_gray.empty())
    {
        std::cout << "the image for fast point extract is empty. " << std::endl;
        return false;
    }
    
    cv::Mat img_mask_gray;
    cv::cvtColor(img_mask, img_mask_gray, cv::COLOR_RGB2GRAY);

    cv::goodFeaturesToTrack(img_gray,vPoint2fs,100,0.01,5.0,img_mask_gray);

    std::cout << "vPoint2fs: " << vPoint2fs.size() << std::endl;
    
    if (vPoint2fs.size() < 20)
    {
        std::cout << "the extracted fast points is little. " << std::endl;
        return false;
    }

    for (auto p:vPoint2fs)
    {
        bpoint bp(p.x,p.y,rows,cols);
        vbPoints.push_back(bp);
    }

    return true;
}

void Frame::setTwc(cv::Mat &_Twc)
{
    Twc = _Twc.clone();
    se2Twc = convert::toSE2(Twc);
    
    // SE2 tmp = SE2(Twc);
    // std::cout << "orignial Twc is: " << std::endl << Twc << std::endl;
    // std::cout << "se2Twc.x: " << se2Twc.x << " se2Twc.y: " << se2Twc.y << " se2Twc.theta: " << se2Twc.theta << std::endl;
    // std::cout << "tmp.x: " << tmp.x << " tmp.y: " << tmp.y << " tmp.theta: " << tmp.theta << std::endl;
    
    // Eigen::Matrix4d eT = convert::toMatrix4d(se2Twc);
    // Eigen::Matrix4d dT = convert::toMatrix4d(Twc);
    // Eigen::Matrix4d mT = tmp.toMatrix4d();
    // std::cout << "eigen mat is: " << std::endl << eT << std::endl;
    // std::cout << "eigen mat is: " << std::endl << dT << std::endl;
    // std::cout << "eigen mat is: " << std::endl << mT << std::endl;
}

bool Frame::setMappoints(std::vector<cv::Point2f>& _vPoints)
{
    vPoint2fs.clear();
    vbPoints.clear();

    for (auto p:_vPoints)
    {
        bpoint bp(p.x,p.y,rows,cols);
        vbPoints.push_back(bp);
        vPoint2fs.push_back(p);
    }

    return true;
}

bool Frame::GetMajorLine(birdview::Line& line) const
{
    if(!mbIsMajorLineSet)
    {
        return false;
    }
    line = mMajorLine;
    return true;
}

void Frame::SetMajorLine(const birdview::Line& line)
{
    mMajorLine = line;
    mbIsMajorLineSet = true;
}
#include "Frame.h"

Frame::Frame(int _idx, cv::Mat &_img, double _timestamp)
{
    idx = _idx;
    timestamp = _timestamp;

    cv::Mat img_gray;
    cv::cvtColor(_img, img_gray, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(20.0, cv::Size(8, 8));
    clahe->apply(img_gray, img);
    cv::buildOpticalFlowPyramid(img, img_pyr, cv::Size(21, 21), 3);

    std::cout << "Frame " << _idx << " is init ... " << std::endl;
}

Frame::~Frame()
{
}

void Frame::setTwc(cv::Mat &_Twc)
{
    Twc = _Twc.clone();
}
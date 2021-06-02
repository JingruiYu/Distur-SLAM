/*************************************************************************
	> File Name: src/convert.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时12分32秒
 ************************************************************************/

#include "convert.h"

double convert::normalize_angle(double theta)
{
	if (theta >= -M_PI && theta < M_PI)
		return theta;

	double multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}

SE2 convert::toSE2(const cv::Mat &cvT)
{
	double yaw = std::atan2(cvT.at<float>(1,0), cvT.at<float>(0,0));
    double theta = normalize_angle(yaw);
    double x = cvT.at<float>(0,2);
    double y = cvT.at<float>(1,2);

	return SE2(x,y,theta);
}

SE2 convert::toSE2(const Eigen::Matrix3d &cvT)
{
	double yaw = std::atan2(cvT(1,0), cvT(0,0));
    double theta = normalize_angle(yaw);
    double x = cvT(0,2);
    double y = cvT(1,2);

	return SE2(x,y,theta);
}

cv::Mat convert::tocvMat(const SE2 &se2T)
{
    cv::Mat m = cv::Mat::eye(3,3,CV_32F);

    double c = cos(se2T.theta);
    double s = sin(se2T.theta);

    m.at<float>(0,0) = c;
    m.at<float>(0,1) = -s;
    m.at<float>(1,0) = s;
    m.at<float>(1,1) = c;
    m.at<float>(0,2) = se2T.x;
    m.at<float>(1,2) = se2T.y;

    return m;
}

Eigen::Matrix3d convert::toMatrix3d(const SE2 &se2T)
{
	double c = cos(se2T.theta);
    double s = sin(se2T.theta);

    Eigen::Matrix3d mat;
    mat <<  c,-s, se2T.x,
            s, c, se2T.y,
            0, 0, 1;

    return mat;
}

Eigen::Matrix4d convert::toMatrix4d(const SE2 &se2T)
{
	double c = cos(se2T.theta);
    double s = sin(se2T.theta);

    Eigen::Matrix4d mat;
    mat <<  c,-s, 0, se2T.x,
            s, c, 0, se2T.y,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return mat;
}

Eigen::Matrix4d convert::toMatrix4d(const cv::Mat &cvT)
{
    Eigen::Matrix4d mat;
    mat <<  cvT.at<float>(0,0),cvT.at<float>(0,1), 0, cvT.at<float>(0,2),
            cvT.at<float>(1,0),cvT.at<float>(1,1), 0, cvT.at<float>(1,2),
            0, 0, 1, 0,
            0, 0, 0, 1;
			
    return mat;
}

void convert::mat2vector(const cv::Mat &img, std::vector<float> &vimg)
{
    int rows = img.rows;
    int cols = img.cols;

    for (size_t c = 0; c < cols; c++)
    {
        for (size_t r = 0; r < rows; r++)
        {
            vimg.push_back(img.at<u_char>(r,c));
        }
    }
}

cv::Point2f convert::BirdviewPT2XY(const cv::Point2f &kp)
{
    int birdviewRows = 384;
    int birdviewCols = 384;
    float pixel2meter = 0.03984;
    float meter2pixel = 25.1;
    float rear_axle_to_center = 1.393;

    cv::Point2f p;
    p.x = (birdviewRows/2.0-kp.y)*pixel2meter+rear_axle_to_center;
    p.y = (birdviewCols/2.0-kp.x)*pixel2meter;
    // p.z = -0.32115;

    return p;
}
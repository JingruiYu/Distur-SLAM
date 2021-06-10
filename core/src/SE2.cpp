/*************************************************************************
	> File Name: src/SE2.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月29日 星期六 10时25分00秒
 ************************************************************************/
#include "SE2.h"
#include "convert.h"

double SE2::normalize_angle(double theta)
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

SE2::SE2(double _x, double _y, double _theta): x(_x), y(_y), theta(_theta)
{
}

SE2::SE2(const cv::Mat &cvT)
{
	double yaw = std::atan2(cvT.at<float>(1,0), cvT.at<float>(0,0));
    theta = convert::normalize_angle(yaw);
    x = cvT.at<float>(0,2);
    y = cvT.at<float>(1,2);
}

SE2::~SE2()
{
}

Eigen::Matrix4d SE2::toMatrix4d()
{
	double c = cos(theta);
    double s = sin(theta);

    Eigen::Matrix4d mat;
    mat <<  c,-s, 0, x,
            s, c, 0, y,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return mat;
}

SE2 SE2::operator+(const SE2& that) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    double _x = x + that.x*c - that.y*s;
    double _y = y + that.x*s + that.y*c;
    double _theta = normalize_angle(theta + that.theta);
    return SE2(_x, _y, _theta);
}

// that.inv() + *this
SE2 SE2::operator-(const SE2& that) const
{
    double dx = x - that.x;
    double dy = y - that.y;
    double dth = normalize_angle(theta - that.theta);

    double c = std::cos(that.theta);
    double s = std::sin(that.theta);
    return SE2(c*dx+s*dy, -s*dx+c*dy, dth);
}

// current -> world
cv::Point2f SE2::operator*(const cv::Point2f &pt) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);

    double xw = c * pt.x - s * pt.y + x;
    double yw = s * pt.x + c * pt.y + y;

    return cv::Point2f(xw, yw);
}
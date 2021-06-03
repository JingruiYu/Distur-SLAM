#include "line.h"
#include "KeyLineGeometry.h"
#include "optimizer.h"
#include "FeatureLine.h"

#include <opencv2/opencv.hpp>

static bool isRef = false;
static cv::Point2f mLastDir;

bool lineport::CalculateMajorLine(Frame* pF)
{
    const float th = cos(M_PI / 4);
    const float th2 = cos(M_PI / 10);
    const float thDist = 0.05;
    const float cx = 0.5 * config::birdviewCols;
    const float cy = 0.5 * config::birdviewRows;

    cv::Mat imageRaw = pF->img.clone();
    cv::Mat imageMask = pF->img_mask.clone();
    cv::Point2f image_center(cx, cy);
    
    // // extract lsd lines
    LineExtractorPtr mpLineExtractor = std::make_shared<LineExtractor>();
    std::vector<KeyLine> vKeyLines;
    mpLineExtractor->extractLines(imageRaw, vKeyLines, imageMask);
    // std::cout << "vKeyLines: " << vKeyLines.size() << std::endl;

    // find major direction from lsd lines
    std::vector<bool> status;
    KeyLine major_line;
    birdview::KeyLineGeometry::FindMajorDirection(vKeyLines, status, major_line);
    birdview::KeyLineGeometry::reduceVector(vKeyLines, status);

    // optimize direction
    cv::Point3f _mline = birdview::KeyLineGeometry::GetKeyLineCoeff(major_line);
    status = std::vector<bool>(vKeyLines.size(), false);
    cv::Point3f infinity(_mline.y, - _mline.x, 0.0);
    for(int i = 0; i < vKeyLines.size(); i++)
    {
        float dist = birdview::KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]).dot(infinity);
        if(fabs(dist) < thDist)
        {
            status[i] = true;
        }
    }
    optimizer::OptimizeMajorLine(vKeyLines,status,_mline);

    cv::Point2f dir(_mline.y, - _mline.x);
    if(!isRef)
    {
        const cv::Point2f minus_y(0.0, -1.0);
        float cos_theta = fabs(dir.dot(minus_y)) / (cv::norm(dir) * cv::norm(minus_y));
        if(cos_theta < th)
        {
            dir = cv::Point2f(dir.y, - dir.x);
        }
        if(dir.y > 0)
        {
            dir = - dir;
        }

        mLastDir = dir / cv::norm(dir);
        isRef = true;
    }
    else
    {
        float cos_theta = fabs(dir.dot(mLastDir)) / (cv::norm(dir) * cv::norm(mLastDir));
        if(cos_theta < th2)
        {
            dir = cv::Point2f(dir.y, - dir.x);
        }
        if(dir.dot(mLastDir) < 0)
        {
            dir = - dir;
        }

        mLastDir = dir / cv::norm(dir);
    }
    dir = dir / cv::norm(dir);
    birdview::Line l(image_center, image_center + cy * dir);

    // convert to XY coordinate
    birdview::Line MajorLine = birdview::Line(convert::BirdviewPT2XY(l.sP), convert::BirdviewPT2XY(l.eP));
    pF->SetMajorLine(MajorLine);
    // pFrame->SetMajorLine(MajorLine);

    // visualization
    cv::Mat keylineImg;
    mpLineExtractor->drawKeylines(imageRaw, vKeyLines, keylineImg);

    cv::Point3f line2 = cv::Point3f(_mline.y, -_mline.x, 1.0);
    birdview::KeyLineGeometry::DrawLineDirection(keylineImg, _mline);
    birdview::KeyLineGeometry::DrawLineDirection(keylineImg, line2);
    cv::arrowedLine(keylineImg,l.sP,l.eP,cv::Scalar(0,0,255),4);

    cv::imshow("KeyLines", keylineImg);
    cv::waitKey(30);

    // std::cout << "line::CalculateMajorLine ... " << std::endl;

    return true;
}
#pragma once

#include "map.h"

#include <pangolin/pangolin.h>
#include <iostream>

class view
{
public:
    struct viewerConfig
    {
        viewerConfig(): windowWidth(1000), windowHeight(600),
                        ViewPointX(0.0), ViewPointY(-0.7), 
                        ViewPointZ(-1.8), ViewPointF(500),
                        KeyFrameSize(0.05), KeyFrameLineWidth(1),
                        GraphLineWidth(0.9)
        {}
        int windowWidth, windowHeight;
        double ViewPointX, ViewPointY, ViewPointZ, ViewPointF;
        double KeyFrameSize, KeyFrameLineWidth, GraphLineWidth;
    };
    
    view(const std::string& _windowName, const viewerConfig& _vfg, map* _mpMap);
    ~view();

    void run();
    void runcore(int num);
    void createWindow();

    void getCameraMatriux(pangolin::OpenGlMatrix &M);
    void DrawTest(int num);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawKeyFrame();
    
    void showKeyPts(cv::Mat& img_show, std::vector<cv::Point2f> &curKeyPt);
    static void showLines(cv::Mat& img_show);
public:

    map* mpMap;

    std::string mWindowName;
    viewerConfig mvfg;

    pangolin::OpenGlRenderState* ms_cam;
    pangolin::View* md_cam;
};


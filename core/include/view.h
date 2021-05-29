#pragma once

#include <pangolin/pangolin.h>
#include <iostream>

class view
{
public:
    struct viewerConfig
    {
        viewerConfig(): windowWidth(1000), windowHeight(600),
        ViewPointX(0.0), ViewPointY(-0.7), ViewPointZ(-1.8), ViewPointF(500)
        {}
        int windowWidth, windowHeight;
        double ViewPointX, ViewPointY, ViewPointZ, ViewPointF;
    };
    
    view(const std::string& _windowName, const viewerConfig& _vfg);
    ~view();

    void run();
    void runcore(int num);
    void createWindow();

    void getCameraMatriux(pangolin::OpenGlMatrix &M);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawKeyFrames(int num);
    
public:
    std::string mWindowName;
    viewerConfig mvfg;

    pangolin::OpenGlRenderState* ms_cam;
    pangolin::View* md_cam;
};


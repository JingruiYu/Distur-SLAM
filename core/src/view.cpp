#include "view.h"

view::view(const std::string& _windowName, const viewerConfig& _vfg, map* _mpMap) :
           mpMap(_mpMap), mWindowName(_windowName), mvfg(_vfg), ms_cam(nullptr), md_cam(nullptr)
{
}

view::~view()
{
}

void view::run()
{
    while ( !pangolin::ShouldQuit() )
    {
        runcore(5);
    }
}

void view::runcore(int num)
{

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    Twc(0,3) = num*0.1;
    md_cam->Activate(*ms_cam );
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    // getCameraMatriux(Twc);
    // ms_cam->Follow(Twc);
    // DrawCurrentCamera(Twc);
    // DrawTest(num);
    DrawKeyFrame();

    pangolin::FinishFrame();
}

void view::createWindow()
{
    pangolin::CreateWindowAndBind(mWindowName,mvfg.windowWidth, mvfg.windowHeight);
    // pangolin::CreateWindowAndBind( "visualize geometry", 1000, 600 );
    
    glEnable(GL_DEPTH_TEST);

    ms_cam = new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(mvfg.windowWidth, mvfg.windowHeight,
        mvfg.ViewPointF, mvfg.ViewPointF, mvfg.windowWidth/2, mvfg.windowHeight/2, 0.1, 1000),
        pangolin::ModelViewLookAt(mvfg.ViewPointX, mvfg.ViewPointY, mvfg.ViewPointZ, 
                                0,0,0,pangolin::AxisY));
    
    md_cam = &pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, - double(mvfg.windowWidth)/double(mvfg.windowHeight))
            .SetHandler(new pangolin::Handler3D(*ms_cam));
    
    glClearColor(1.0f,1.0f,1.0f,1.0f);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
}


void view::getCameraMatriux(pangolin::OpenGlMatrix &M)
{
    M.SetIdentity();
}

void view::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float w = 0.08;
    const float h = w*0.75;
    const float z = w*0.6;
    const float mCameraLineWidth = 3;

    glPushMatrix();

    glMultMatrixd(Twc.m);

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void view::DrawTest(int num)
{
    const float w = 0.05;
    const float h = w*0.75;
    const float z = w*0.6;
    float mKeyFrameLineWidth = 1;

    for(int i=0; i<num; i++)
    {
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        Twc(0,3) = i*0.1;
        
        glPushMatrix();

        glMultMatrixd(Twc.m);

        glLineWidth(mKeyFrameLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void view::DrawKeyFrame()
{
    std::vector<keyFrame*> vkeyFrameAll = mpMap->getKeyFrameAll();
    // std::cout << "vkeyFrameAll.size() : " << vkeyFrameAll.size() << std::endl;

    const float &w = mvfg.KeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    for(auto pKF:vkeyFrameAll)
    {
        // std::cout << "pKF: " << pKF->idx << std::endl;
        // Eigen::Matrix4d Twc = convert::toMatrix4d(pKF->Twc);
        Eigen::Matrix4d Twc = pKF->mGtPose.toMatrix4d();

        glPushMatrix();
        glMultMatrixd(Twc.data());

        glLineWidth(mvfg.KeyFrameLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);

        glVertex3f(0,0,0); glVertex3f(w,h,z);
        glVertex3f(0,0,0); glVertex3f(w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,h,z);

        glVertex3f(w,h,z); glVertex3f(w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(w,h,z);
        glVertex3f(-w,-h,z); glVertex3f(w,-h,z);

        glEnd();

        glPopMatrix();
    }

    // Draw graph
    glLineWidth(mvfg.GraphLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    for(size_t i = 1; i < vkeyFrameAll.size(); i++)
    {
        Eigen::Vector3d begin = convert::toMatrix4d(vkeyFrameAll[i-1]->Twc).topRightCorner(3, 1);
        Eigen::Vector3d end = convert::toMatrix4d(vkeyFrameAll[i]->Twc).topRightCorner(3, 1);
        glVertex3f(begin[0], begin[1], begin[2]);
        glVertex3f(end[0], end[1], end[2]);
    }
    glEnd();
}
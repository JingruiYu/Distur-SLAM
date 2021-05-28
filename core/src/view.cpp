#include "view.h"

view::view(const std::string& _windowName, const viewerConfig& _vfg) :
            mWindowName(_windowName), mvfg(_vfg), ms_cam(nullptr), md_cam(nullptr)
{
}

view::~view()
{
}

void view::run()
{
    pangolin::CreateWindowAndBind ( "visualize geometry", 1000, 600 );
    glEnable ( GL_DEPTH_TEST );
    pangolin::OpenGlRenderState s_cam (
        pangolin::ProjectionMatrix ( 1000, 600, 420, 420, 500, 300, 0.1, 1000 ),
        pangolin::ModelViewLookAt ( 3,3,3,0,0,0,pangolin::AxisY )
    );
    
    const int UI_WIDTH = 500;
    
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f/600.0f).SetHandler(new pangolin::Handler3D(s_cam));
    
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while ( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        
        getCameraMatriux(Twc);

        DrawKeyFrames();

        s_cam.Follow(Twc);

        d_cam.Activate( s_cam );
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        DrawCurrentCamera(Twc);

        pangolin::FinishFrame();
    }
}

void view::createWindow()
{
    // pangolin::CreateWindowAndBind(mWindowName,mvfg.windowWidth, mvfg.windowHeight);
    // pangolin::CreateWindowAndBind( "visualize geometry", 1000, 600 );
    
    // glEnable(GL_DEPTH_TEST);

    // s_cam = new pangolin::OpenGlRenderState(
    //     pangolin::ProjectionMatrix(mvfg.windowWidth, mvfg.windowHeight,
    //     mvfg.ViewPointF, mvfg.ViewPointF, mvfg.windowWidth/2, mvfg.windowHeight/2, 0.1, 1000),
    //     pangolin::ModelViewLookAt(mvfg.ViewPointX, mvfg.ViewPointY, mvfg.ViewPointZ, 
    //                             0,0,0,pangolin::AxisY));
    
    // d_cam = &pangolin::CreateDisplay()
    //         .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, - double(mvfg.windowWidth)/double(mvfg.windowHeight))
    //         .SetHandler(new pangolin::Handler3D(*s_cam));
    
    // glClearColor(1.0f,1.0f,1.0f,1.0f);

    // pangolin::OpenGlMatrix Twc;
    // Twc.SetIdentity();
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

void view::DrawKeyFrames()
{
    const float w = 0.05;
    const float h = w*0.75;
    const float z = w*0.6;
    float mKeyFrameLineWidth = 1;

    for(int i=0; i<5; i++)
    {
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        Twc(0,3) = i;
        
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
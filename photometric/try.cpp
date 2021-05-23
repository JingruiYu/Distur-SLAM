#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/core/block_solver.h"
#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/core/robust_kernel.h"
#include "/home/yujr/mono_brid_semantic/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace std;
using namespace g2o;

/********************************************
 * 本节演示了RGBD上的稀疏直接法 
 ********************************************/

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

inline Eigen::Vector3d project2Dto3D ( int x, int y, int d, float fx, float fy, float cx, float cy, float scale )
{
    float zz = float ( d ) /scale;
    float xx = zz* ( x-cx ) /fx;
    float yy = zz* ( y-cy ) /fy;
    return Eigen::Vector3d ( xx, yy, zz );
}

inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}

// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参； 输出：相机位姿
// 返回：true为成功，false失败
bool poseEstimationDirect ( const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& intrinsics, Eigen::Isometry3d& Tcw );


// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect ( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image )
        : x_world_ ( point ), fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), image_ ( image )
    {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v  =static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d x_local = v->estimate().map ( x_world_ );
        float x = x_local[0]*fx_/x_local[2] + cx_;
        float y = x_local[1]*fy_/x_local[2] + cy_;
        // check x,y is in the image
        if ( x-4<0 || ( x+4 ) >image_->cols || ( y-4 ) <0 || ( y+4 ) >image_->rows )
        {
            _error ( 0,0 ) = 0.0;
            this->setLevel ( 1 );
        }
        else
        {
            _error ( 0,0 ) = getPixelValue ( x,y ) - _measurement;
        }
    }

    // plus in manifold
    virtual void linearizeOplus( )
    {
        if ( level() == 1 )
        {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d xyz_trans = vtx->estimate().map ( x_world_ );   // q in book

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        float u = x*fx_*invz + cx_;
        float v = y*fy_*invz + cy_;

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx_;
        jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
        jacobian_uv_ksai ( 0,2 ) = - y*invz *fx_;
        jacobian_uv_ksai ( 0,3 ) = invz *fx_;
        jacobian_uv_ksai ( 0,4 ) = 0;
        jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx_;

        jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 ) *fy_;
        jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy_;
        jacobian_uv_ksai ( 1,2 ) = x*invz *fy_;
        jacobian_uv_ksai ( 1,3 ) = 0;
        jacobian_uv_ksai ( 1,4 ) = invz *fy_;
        jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
        jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

        _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;
    }

    // dummy read and write functions because we don't care...
    virtual bool read ( std::istream& in ) {}
    virtual bool write ( std::ostream& out ) const {}

protected:
    // get a gray scale value from reference image (bilinear interpolated)
    inline float getPixelValue ( float x, float y )
    {
        uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
        float xx = x - floor ( x );
        float yy = y - floor ( y );
        return float (
                   ( 1-xx ) * ( 1-yy ) * data[0] +
                   xx* ( 1-yy ) * data[1] +
                   ( 1-xx ) *yy*data[ image_->step ] +
                   xx*yy*data[image_->step+1]
               );
    }
public:
    Eigen::Vector3d x_world_;   // 3D point in world frame
    float cx_=0, cy_=0, fx_=0, fy_=0; // Camera intrinsics
    cv::Mat* image_=nullptr;    // reference image
};

void fit(std::vector<float> & data_x, std::vector<float> & data_y, float &a, float &b)
{
    float A = 0.0;
    float B = 0.0;
    float C = 0.0;
    float D = 0.0;
    float E = 0.0;
    float F = 0.0;
    int n = data_x.size();
    float data_n = n;

    std::cout << "n: " << n << " , data_n: "  << data_n << std::endl;

    for (size_t i = 0; i < data_x.size(); i++)
    {
        A += data_x[i] * data_x[i];
        B += data_x[i];
        C += data_x[i] * data_y[i];
        D += data_y[i];
    }
    
    float temp = 0;
    if (temp = (data_n*A - B*B))
    {
        a = (data_n*C - B*D) / temp;
        b = (A*D - B*C) / temp;
    }
    else
    {
        a = 1;
        b = 0;
    }

    std::cout << "a: " << a << ", b: " << b << std::endl;
}


int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    srand ( ( unsigned int ) time ( 0 ) );
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";

    ifstream fin ( associate_file );

    string rgb_file, depth_file, time_rgb, time_depth;
    vector<Measurement> measurements1,measurements2;
    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K<<fx,0.f,cx,0.f,fy,cy,0.f,0.f,1.0f;

    // get gt pose
    Eigen::Matrix4d gTwc1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d gTwc2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d gTwc3 = Eigen::Matrix4d::Identity();

    Eigen::Quaterniond q1(-0.3909,0.8851,0.2362,-0.0898); 
    Eigen::Matrix3d Rwc1 = q1.toRotationMatrix();
    Eigen::Matrix<double,3,1> t1(1.3112,0.8507,1.5186);
    gTwc1.topLeftCorner(3, 3) = Rwc1;
    gTwc1.topRightCorner(3, 1) = t1;
    Eigen::Matrix4d gTc1w = gTwc1.inverse();
    
    Eigen::Quaterniond q2(-0.3998,0.8893,0.2037,-0.0885);
    Eigen::Matrix3d Rwc2 = q2.toRotationMatrix();
    Eigen::Matrix<double,3,1> t2(1.3370,0.8274,1.5265);
    gTwc2.topLeftCorner(3, 3) = Rwc2;
    gTwc2.topRightCorner(3, 1) = t2;
    Eigen::Matrix4d gTc2w = gTwc2.inverse();

    Eigen::Quaterniond q3(-0.4180,0.8865,0.1763,-0.0915);
    Eigen::Matrix3d Rwc3 = q3.toRotationMatrix();
    Eigen::Matrix<double,3,1> t3(1.3627,0.7957,1.5391);
    gTwc3.topLeftCorner(3, 3) = Rwc3;
    gTwc3.topRightCorner(3, 1) = t3;
    Eigen::Matrix4d gTc3w = gTwc3.inverse();

    std::cout << "gTwc1: " << std::endl << gTwc1 << std::endl;
    std::cout << "gTwc2: " << std::endl << gTwc2 << std::endl;
    std::cout << "gTwc3: " << std::endl << gTwc3 << std::endl;

    // read mat1, mat2, mat3
    cv::Mat color1, color2, color3;
    cv::Mat depth1, depth2, depth3;
    cv::Mat gray1, gray2, gray3;
    fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
    color1 = cv::imread ( path_to_dataset+"/"+rgb_file );
    depth1 = cv::imread ( path_to_dataset+"/"+depth_file, -1 );
    fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
    color2 = cv::imread ( path_to_dataset+"/"+rgb_file );
    depth2 = cv::imread ( path_to_dataset+"/"+depth_file, -1 );
    fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
    color3 = cv::imread ( path_to_dataset+"/"+rgb_file );
    depth3 = cv::imread ( path_to_dataset+"/"+depth_file, -1 );

    cv::cvtColor ( color1, gray1, cv::COLOR_BGR2GRAY );
    cv::cvtColor ( color2, gray2, cv::COLOR_BGR2GRAY );
    cv::cvtColor ( color3, gray3, cv::COLOR_BGR2GRAY );

    // cv::imshow("1_c",color1);
    // cv::imshow("2_c",color2);
    // cv::imshow("3_c",color3);
    // cv::imshow("1_d",depth1);
    // cv::imshow("2_d",depth2);
    // cv::imshow("3_d",depth3);
    // cv::waitKey(0);


    // get kpts from mat1
    vector<cv::KeyPoint> keypoints1;
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
    detector->detect ( color1, keypoints1 );
    for ( auto kp:keypoints1 )
    {
        // 去掉邻近边缘处的点
        if ( kp.pt.x < 20 || kp.pt.y < 20 || ( kp.pt.x+20 ) > color1.cols || ( kp.pt.y+20 ) > color1.rows )
            continue;
        ushort d = depth1.ptr<ushort> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ];
        if ( d==0 )
            continue;
        Eigen::Vector3d pc13d = project2Dto3D ( kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale );
        Eigen::Vector3d pw = Rwc1 * pc13d + t1;
        float grayscale = float ( gray1.ptr<uchar> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ] );
        measurements1.push_back ( Measurement ( pw, grayscale ) );
    }
    std::cout << "measurements1: "  << measurements1.size() << std::endl;

    // optimize mat1 with mat2, get T
    // Eigen::Isometry3d mTc2w = Eigen::Isometry3d::Identity();
    // mTc2w.rotate(gTc2w.topLeftCorner(3, 3));
    // mTc2w.pretranslate(gTc2w.topRightCorner(3, 1));
    Eigen::Isometry3d mTc2w(gTc2w);
    // std::cout << "mTc2w 1 : " << std::endl << mTc2w.matrix() << std::endl;
    // std::cout << "gTc2w 1 : " << std::endl << gTc2w.matrix() << std::endl;

    poseEstimationDirect ( measurements1, &gray2, K, mTc2w );
    std::cout << "mTc2w=" << std::endl << mTc2w.matrix().inverse() <<endl;

    // cv::Mat img_show ( color1.rows*2, color1.cols, CV_8UC3 );
    // color1.copyTo ( img_show ( cv::Rect ( 0,0,color1.cols, color1.rows ) ) );
    // color2.copyTo ( img_show ( cv::Rect ( 0,color1.rows,color1.cols, color1.rows ) ) );
    // for ( Measurement m:measurements1 )
    // {
    //     if ( rand() > RAND_MAX/5 )
    //         continue;
    //     Eigen::Vector3d pw = m.pos_world;
    //     Eigen::Vector3d p = gTc1w.topLeftCorner(3, 3)*m.pos_world + gTc1w.topRightCorner(3, 1);
    //     Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );
    //     Eigen::Vector3d p2 = mTc2w*m.pos_world;
    //     Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy );
    //     if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color2.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color2.rows )
    //         continue;

    //     float b = 255*float ( rand() ) /RAND_MAX;
    //     float g = 255*float ( rand() ) /RAND_MAX;
    //     float r = 255*float ( rand() ) /RAND_MAX;
    //     cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 2 );
    //     cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color1.rows ), 8, cv::Scalar ( b,g,r ), 2 );
    //     cv::line ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color1.rows ), cv::Scalar ( b,g,r ), 1 );
    // }
    // cv::imshow ( "result", img_show );
    // cv::waitKey ( 0 );

    // change mat2
    std::vector<float> data_x, data_y;
    for (Measurement m:measurements1)
    {
        Eigen::Vector3d p = gTc1w.topLeftCorner(3, 3)*m.pos_world + gTc1w.topRightCorner(3, 1);
        Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );
        Eigen::Vector3d p2 = gTc2w.topLeftCorner(3, 3)*m.pos_world + gTc2w.topRightCorner(3, 1);
        Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy ); 

        // std::cout << "pixel_now: " << pixel_now << std::endl;

        if (cvRound ( pixel_now[1] ) >= 480)
        {
            continue;
        }
        
        float grayscale1 = float ( gray1.ptr<uchar> ( cvRound ( pixel_prev[1] ) ) [ cvRound ( pixel_prev[0] ) ] );
        float grayscale2 = float ( gray2.ptr<uchar> ( cvRound ( pixel_now[1] ) ) [ cvRound ( pixel_now[0] ) ] );

        data_y.push_back(grayscale1);
        data_x.push_back(grayscale2);
    }
    
    float pck, pco;
    fit(data_x,data_y,pck,pco);
    std::cout << "pck:" << pck << ", pco: " << pco << std::endl;

    // std::cout << "gray2.rows: " << gray2.rows << std::endl;
    // std::cout << "gray2.cols: " << gray2.cols << std::endl;
    
    // cv::imshow("before",gray2);

    for (size_t r = 0; r < gray2.rows; r++)
    {
        for (size_t c = 0; c < gray2.cols; c++)
        {
            float tmp = 0.9 * gray2.at<uchar>(r,c) + 81.4421;
            uchar gtp = tmp;

            if(tmp > 255)
                gtp = 255;
            

            gray2.at<uchar>(r,c) = gtp;
        }
    }


    // cal Tw2 again

    Eigen::Isometry3d mTc2wNew(gTc2w);
    poseEstimationDirect ( measurements1, &gray2, K, mTc2wNew );
    std::cout << "mTc2wNew=" << std::endl << mTc2wNew.matrix().inverse() <<endl;

    
    // cv::imshow("after",gray2);
    // cv::waitKey(0);

    // get kpts from mat2
    // optimze mat2 with mat3, get T
    vector<cv::KeyPoint> keypoints2;
    cv::Ptr<cv::FastFeatureDetector> detector2 = cv::FastFeatureDetector::create();
    detector2->detect ( color2, keypoints2 );
    for ( auto kp:keypoints2 )
    {
        // 去掉邻近边缘处的点
        if ( kp.pt.x < 20 || kp.pt.y < 20 || ( kp.pt.x+20 ) > color2.cols || ( kp.pt.y+20 ) > color2.rows )
            continue;
        ushort d = depth2.ptr<ushort> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ];
        if ( d==0 )
            continue;
        Eigen::Vector3d pc23d = project2Dto3D ( kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale );
        Eigen::Vector3d pw = Rwc2 * pc23d + t2;
        float grayscale = float ( gray2.ptr<uchar> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ] );
        measurements2.push_back ( Measurement ( pw, grayscale ) );
    }
    std::cout << "measurements2: "  << measurements2.size() << std::endl;

    // optimize mat1 with mat2, get T
    // Eigen::Isometry3d mTc2w = Eigen::Isometry3d::Identity();
    // mTc2w.rotate(gTc2w.topLeftCorner(3, 3));
    // mTc2w.pretranslate(gTc2w.topRightCorner(3, 1));
    Eigen::Isometry3d mTc3w(gTc3w);
    // std::cout << "mTc2w 1 : " << std::endl << mTc2w.matrix() << std::endl;
    // std::cout << "gTc2w 1 : " << std::endl << gTc2w.matrix() << std::endl;

    poseEstimationDirect ( measurements2, &gray3, K, mTc3w );
    std::cout << "mTc3w=" << std::endl << mTc3w.matrix().inverse() <<endl;

    cv::Mat img_show ( color2.rows*2, color2.cols, CV_8UC3 );
    color2.copyTo ( img_show ( cv::Rect ( 0,0,color2.cols, color2.rows ) ) );
    color3.copyTo ( img_show ( cv::Rect ( 0,color2.rows,color2.cols, color2.rows ) ) );
    for ( Measurement m:measurements1 )
    {
        if ( rand() > RAND_MAX/5 )
            continue;
        Eigen::Vector3d pw = m.pos_world;
        Eigen::Vector3d p = gTc2w.topLeftCorner(3, 3)*m.pos_world + gTc2w.topRightCorner(3, 1);
        Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );
        Eigen::Vector3d p2 = mTc3w*m.pos_world;
        Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy );
        if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color2.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color2.rows )
            continue;

        float b = 255*float ( rand() ) /RAND_MAX;
        float g = 255*float ( rand() ) /RAND_MAX;
        float r = 255*float ( rand() ) /RAND_MAX;
        cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 2 );
        cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color2.rows ), 8, cv::Scalar ( b,g,r ), 2 );
        cv::line ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color2.rows ), cv::Scalar ( b,g,r ), 1 );
    }
    cv::imshow ( "result", img_show );
    cv::waitKey ( 0 );


    return 0;
}

bool poseEstimationDirect ( const vector< Measurement >& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( false );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // 添加边
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
            m.pos_world,
            K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), gray
        );
        edge->setVertex ( 0, pose );
        edge->setMeasurement ( m.grayscale );
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        edge->setId ( id++ );
        optimizer.addEdge ( edge );
    }
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    Tcw = pose->estimate();
}


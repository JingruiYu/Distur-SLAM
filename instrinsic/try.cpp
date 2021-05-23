#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

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

struct K_Fit_COST
{
    K_Fit_COST(Eigen::Vector2d _uv, Eigen::Vector3d _pc) : uv(_uv), pc(_pc) {}

    template <typename T>
    bool operator() (
        const T* const vks,
        T* residual ) const
    {
        residual[0] = T(uv(0,0)) - (vks[0] * (T(pc(0,0)) / T(pc(2,0))) + vks[2]);
        residual[1] = T(uv(1,0)) - (vks[1] * (T(pc(1,0)) / T(pc(2,0))) + vks[3]);
        
        return true;
    }

    Eigen::Vector2d uv;
    Eigen::Vector3d pc;
};

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
    Eigen::Matrix<double,3,1> twc1(1.3112,0.8507,1.5186);
    gTwc1.topLeftCorner(3, 3) = Rwc1;
    gTwc1.topRightCorner(3, 1) = twc1;
    Eigen::Matrix4d gTc1w = gTwc1.inverse();
    
    Eigen::Quaterniond q2(-0.3998,0.8893,0.2037,-0.0885);
    Eigen::Matrix3d Rwc2 = q2.toRotationMatrix();
    Eigen::Matrix<double,3,1> twc2(1.3370,0.8274,1.5265);
    gTwc2.topLeftCorner(3, 3) = Rwc2;
    gTwc2.topRightCorner(3, 1) = twc2;
    Eigen::Matrix4d gTc2w = gTwc2.inverse();
    Eigen::Matrix3d Rc2w = gTc2w.topLeftCorner(3, 3);
    Eigen::Matrix<double,3,1> tc2w = gTc2w.topRightCorner(3, 1);

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


    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( color1,keypoints_1 );
    detector->detect ( color2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( color1, keypoints_1, descriptors_1 );
    descriptor->compute ( color2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> matches,vgoodMatches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    ceres::Problem problem;
    double vks[4] = {500.0,500.0,320.0,250.0};
    for(int i=0; i < matches.size(); i++)
    {
        cv::DMatch imatch = matches[i];
        if (imatch.distance < 20)
        {
            vgoodMatches.push_back(imatch);

            cv::Point2f pt1 = keypoints_1[imatch.queryIdx].pt;
            cv::Point2f pt2 = keypoints_2[imatch.trainIdx].pt;

            ushort d = depth1.ptr<ushort> ( cvRound ( pt1.y ) ) [ cvRound ( pt1.x ) ];
            if ( d==0 )
                continue;
            Eigen::Vector3d pc1 = project2Dto3D ( pt1.x, pt1.y, d, fx, fy, cx, cy, depth_scale );
            Eigen::Vector3d pw = Rwc1 * pc1 + twc1;
            Eigen::Vector3d pc2 = Rc2w * pw + tc2w;
            Eigen::Vector2d uv2(pt2.x, pt2.y);
            Eigen::Vector2d uv2t = project3Dto2D(pc2(0,0),pc2(1,0),pc2(2,0),fx,fy,cx,cy);

            // std::cout << "pt2 : " <<  pt2 << ", uv2: " << uv2 << std::endl;

            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<K_Fit_COST,2,4>(
                    new K_Fit_COST(uv2t,pc2)
                ),
                nullptr,
                vks
            );
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    for(auto k:vks)
    {
        std::cout << k << std::endl;
    }

    // cv::Mat img_match;
    // cv::drawMatches(color1,keypoints_1,color2,keypoints_2,vgoodMatches,img_match);
    // cv::imshow("matches",img_match);
    // cv::waitKey(0);

    return 0;
}


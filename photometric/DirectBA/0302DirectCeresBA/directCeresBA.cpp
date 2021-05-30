#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/cubic_interpolation.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace Eigen;

typedef vector<vector<float>> vvf;

// global variables
string pose_file = "./poses.txt";
string points_file = "./points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

struct GetPixelGrayValue {
    GetPixelGrayValue(const float pixel_gray_val_in[16],
                      const int rows,
                      const int cols,
                      const std::vector<float>& vec_pixel_gray_values)
    {
        for (int i = 0; i < 16; i++)
            pixel_gray_val_in_[i] = pixel_gray_val_in[i];
        rows_ = rows;
        cols_ = cols;
        
        grid2d.reset(new ceres::Grid2D<float>(
            &vec_pixel_gray_values[0], 0, rows_, 0, cols_));
        get_pixel_gray_val.reset(
            new ceres::BiCubicInterpolator<ceres::Grid2D<float> >(*grid2d));
    }
    
    template <typename T>
    bool operator()(const T* const so3t,            // 模型参数，6维
                    const T* const xyz,             // 模型参数，3维
                    T* residual ) const             // 残差，16维
    {
        // 计算变换后的u和v
        T u_out, v_out, pt[3], r[3];
        r[0] = so3t[0];
        r[1] = so3t[1];
        r[2] = so3t[2];
        ceres::AngleAxisRotatePoint(r, xyz, pt);
        pt[0] += so3t[3];
        pt[1] += so3t[4];
        pt[2] += so3t[5];
        u_out = pt[0] * T(fx) / pt[2] + T(cx);
        v_out = pt[1] * T(fy) / pt[2] + T(cy);
                
        for (int i = 0; i < 16; i++)
        {
            int m = i / 4;
            int n = i % 4;
            T u, v, pixel_gray_val_out;
            u = u_out + T(m - 2);
            v = v_out + T(n - 2);
            get_pixel_gray_val->Evaluate(u, v, &pixel_gray_val_out);
            residual[i] = T(pixel_gray_val_in_[i]) - pixel_gray_val_out;
        }
        
        return true;
    }

    float pixel_gray_val_in_[16];
    int rows_,cols_;
    std::unique_ptr<ceres::Grid2D<float> > grid2d;
    std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<float> > > get_pixel_gray_val;
};


int main(int argc, char **argv) {
    
    // read poses and points
    vector<double> poses, points;
    
    ifstream fin(pose_file);

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7]{};
        for (auto &d: data) fin >> d;
        double q[4]={data[6],data[3],data[4],data[5]};
        double so3[3]{};
        ceres::QuaternionToAngleAxis(q,so3);
        for (int i = 0; i < 3; ++i)
        {
            poses.push_back(so3[i]);
        }
        for (int i = 0; i < 3; ++i)
        {
            poses.push_back(data[i]);
        }
        
        if (!fin.good()) break;
    }
    fin.close();

    vvf patch_gray_values;
    fin.open(points_file);
    while (!fin.eof()) {
        double xyz[3]{};
        
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        
        if (xyz[0] == 0) break;
        
        for (int i = 0; i < 3; i++) points.push_back(xyz[i]);
        
        vector<float> vec_gray_values;

        for (int i = 0; i < 16; i++)
        {
            float gray_val;
            fin >> gray_val;
            vec_gray_values.push_back(gray_val);
        }
        
        patch_gray_values.push_back(vec_gray_values);
        
        if (fin.good() == false) break;
    }
    fin.close();
    
    cout << "poses: " << poses.size()/6 << ", points: " << points.size()/3 << endl;
    
    // read images
    vvf multi_img_gray_values;
    vector<int> multi_img_rows_cols;
    boost::format fmt("./%d.png");
    
    for (int i = 0; i < 7; i++)
    {
        cv::Mat img = cv::imread((fmt % i).str(), 0);
        vector<float> img_gray_values;
        int rows,cols;
        rows = img.rows;
        cols = img.cols;
        multi_img_rows_cols.push_back(rows);
        multi_img_rows_cols.push_back(cols);
        
        for (uint u = 0; u < cols; ++u)
            for (uint v = 0; v < rows; ++v)
                img_gray_values.push_back(img.at<uchar>(v, u));
        
        multi_img_gray_values.push_back(img_gray_values);
    }
    
    int poses_num, parameters_poses_num, points_num, parameters_points_num, parameters_num;
    parameters_poses_num = poses.size();
    poses_num = parameters_poses_num / 6;
    parameters_points_num = points.size();
    points_num = parameters_points_num / 3;
    parameters_num = parameters_poses_num + parameters_points_num;
    double *first_poses_pos = new double[parameters_num];
    double *parameters, *first_point_pos;
    parameters = first_poses_pos;
    first_point_pos = first_poses_pos + parameters_poses_num;
    
    for (int i = 0; i < parameters_poses_num; i++)
        parameters[i] = poses[i];
    
    for (int i = 0; i < parameters_points_num; i++)
        parameters[parameters_poses_num + i] = points[i];
        
    ceres::Problem problem;
    for (int ip = 0; ip < poses_num; ip++)
    {
        for (int jp = 0; jp < points_num; jp++)
        {
            double *pose_position = first_poses_pos + ip * 6;
            double *point_position = first_point_pos + jp * 3;
            float gray_values[16]{};
            for ( int i = 0; i < 16; i++ )
                gray_values[i] = patch_gray_values[jp][i];
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<GetPixelGrayValue, 16, 6, 3> (
                    new GetPixelGrayValue ( gray_values,
                                            multi_img_rows_cols[ip * 2],
                                            multi_img_rows_cols[ip * 2 + 1],
                                            multi_img_gray_values[ip]
                    )
                ),
                new ceres::HuberLoss(1.0),
                pose_position,
                point_position
            );
        }
    }
    
    // 配置求解器
    std::cout << "Solving ceres directBA ... " << endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    
    delete [] parameters;
    return 0;
}

#pragma once

#include "Frame.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/jet.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/ceres.h>
#include <memory>
#include <iostream>

int birdviewRows = 384;
int birdviewCols = 384;
float pixel2meter = 0.03984;
float meter2pixel = 25.1;
float rear_axle_to_center = 1.393;

template <typename T>
Eigen::Matrix<T, 2, 2> to2DRotationMatrix(T yaw_radians)
{
    const T cos_yaw = ceres::cos(yaw_radians);
    const T sin_yaw = ceres::sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

template <typename T>
Eigen::Matrix<T,2,1> pixel2XY(T u, T v)
{
    Eigen::Matrix<T, 2, 1> pc;
    pc[0] = (T(birdviewRows/2.0)-v) * T(pixel2meter) + T(rear_axle_to_center);
    pc[1] = (T(birdviewCols/2.0)-u) * T(pixel2meter);

    return pc;
}

template <typename T>
Eigen::Matrix<T,2,1> XY2pixel(T x, T y)
{
    Eigen::Matrix<T, 2, 1> pu;
    pu[0] = T(birdviewCols/2.0) - y * T(meter2pixel);
    pu[1] = T(birdviewRows/2.0) - (x- T(rear_axle_to_center)) * T(meter2pixel);

    return pu;
}

struct directError
{
    directError(const std::vector<float>& vimg1, 
                const std::vector<float>& vimg2,
                const cv::Point2f& p1,
                const cv::Point2f& p2,
                const int rows,
                const int cols)
    {
        p1_ = p1;
        p2_ = p2;

        rows_ = rows;
        cols_ = cols;

        img1_grid2d.reset(new ceres::Grid2D<float>(
            &vimg1[0], 0, rows_, 0, cols_));
        img1_get_pixel_gray_val.reset(
            new ceres::BiCubicInterpolator<ceres::Grid2D<float> >(*img1_grid2d));

        img2_grid2d.reset(new ceres::Grid2D<float>(
            &vimg2[0], 0, rows_, 0, cols_));
        img2_get_pixel_gray_val.reset(
            new ceres::BiCubicInterpolator<ceres::Grid2D<float> >(*img2_grid2d));
    }

    template<typename T>
    bool operator()(const T* const pose_x, const T* pose_y, const T* pose_theta, T* residuals) const
    {
        Eigen::Matrix<T,2,1> pc2 = pixel2XY(T(p2_.x), T(p2_.y));
        Eigen::Matrix<T,2,1> t_bw(*pose_x, *pose_y);
        Eigen::Matrix<T,2,1> pc1 = to2DRotationMatrix(*pose_theta) * pc2 + t_bw;
        Eigen::Matrix<T,2,1> pu1 = XY2pixel(pc1[0],pc1[1]);
        T gray1, gray2;
        img1_get_pixel_gray_val->Evaluate(pu1[0],pu1[1],&gray1);
        img2_get_pixel_gray_val->Evaluate(T(p2_.x),T(p2_.y),&gray2);

        residuals[0] = gray2 - gray1;

        return true;
    }


    cv::Point2f p1_, p2_;
    int rows_,cols_;
    std::unique_ptr<ceres::Grid2D<float> > img1_grid2d;
    std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<float> > > img1_get_pixel_gray_val;

    std::unique_ptr<ceres::Grid2D<float> > img2_grid2d;
    std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<float> > > img2_get_pixel_gray_val;

};

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T& angle_radians)
{
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return angle_radians -
           two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization
{
public:
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
    {
        *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
        return true;
    }
};
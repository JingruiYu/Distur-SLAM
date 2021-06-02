/*************************************************************************
	> File Name: include/optimizer.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月30日 星期三 18时24分33秒
 ************************************************************************/

#include "optimizer.h"
#include "convert.h"
#include "costFunction.h"

void optimizer::FrameDirectOptimization(Frame* pF1, Frame* pF2, cv::Mat &Tc1c2)
{
	// prepare image
	std::vector<float> vimg1, vimg2;
	convert::mat2vector(pF1->img_gray,vimg1);
	convert::mat2vector(pF2->img_gray,vimg2);

	std::vector<cv::Point2f> vmp1 = pF1->vPoint2fs;
	std::vector<cv::Point2f> vmp2 = pF2->vPoint2fs;

	SE2 Tcc = convert::toSE2(Tc1c2);
	SE2 Tcc_cp = convert::toSE2(Tc1c2);

	int row = pF1->img_gray.rows;
	int col = pF2->img_gray.cols;

	ceres::Problem problem;
	ceres::LocalParameterization* angle_local_parameter = new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>;
	for (size_t i = 0; i < vmp1.size(); i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<directError,1,1,1,1>(
				new directError(vimg1,vimg2,vmp1[i],vmp2[i],row,col)
			),
			new ceres::HuberLoss(1.0),
			&Tcc.x, &Tcc.y, &Tcc.theta
		);
		problem.SetParameterization(&Tcc.theta, angle_local_parameter);
	}
	
	
	ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";

	bool isupdate = true;
	if(summary.IsSolutionUsable())
    {
		double ln = std::sqrt(Tcc.x*Tcc.x + Tcc.y*Tcc.y);
		double lo = std::sqrt(Tcc_cp.x*Tcc_cp.x + Tcc_cp.y*Tcc_cp.y);
		double dt = std::abs(Tcc_cp.theta - Tcc.theta);

		if (ln/lo > 3|| ln/lo < 0.2)
		{
			std::cout << "ln/lo: " << ln/lo << std::endl;
			isupdate = false;
		}

		if (dt > 0.5)
		{
			std::cout << "dt: " << dt << std::endl;
			isupdate = false;
		}
    }
	else
	{
		isupdate = false;
	}

	if (isupdate)
	{
		Tc1c2 = convert::tocvMat(Tcc);
	}
	
	
	// std::cout << "FrameDirectOptimization ... " << std::endl;
}


void optimizer::OptimizeMajorLine(const std::vector<KeyLine> &vKeyLines, const std::vector<bool> &vIsParallel,
                                  cv::Point3f &le)
{
    assert(vKeyLines.size() == vIsParallel.size());
    int N = vKeyLines.size();

    ceres::Problem problem;

    Eigen::Vector2d normal(le.x, le.y);
    for(int i = 0; i < N; i++)
    {
        cv::Point3f le_i = birdview::KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]);
        if(vIsParallel[i])
        {
            ceres::CostFunction* cost_function = ParallelLineCostFunctor::Create(le_i.x, le_i.y);
            problem.AddResidualBlock(cost_function, nullptr,normal.data());
        }
        else
        {
            ceres::CostFunction* cost_function = OrthogonalLineCostFunctor::Create(le_i.x, le_i.y);
            problem.AddResidualBlock(cost_function, nullptr,normal.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    if(!summary.IsSolutionUsable())
    {
        return;
    }

    le.x = normal[0];
    le.y = normal[1];
}

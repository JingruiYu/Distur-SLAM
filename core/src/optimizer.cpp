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

	int row = pF1->img_gray.rows;
	int col = pF2->img_gray.cols;

	ceres::Problem problem;
	for (size_t i = 0; i < vmp1.size(); i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<directError,1,1,1,1>(
				new directError(vimg1,vimg2,vmp1[i],vmp2[i],row,col)
			),
			new ceres::HuberLoss(1.0),
			&Tcc.x, &Tcc.y, &Tcc.theta
		);
	}
	
	ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";

	if(summary.IsSolutionUsable())
    {
		Tc1c2 = convert::tocvMat(Tcc);
    }
	std::cout << "FrameDirectOptimization ... " << std::endl;
}
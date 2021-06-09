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

void optimizer::poseGraphOptimize(std::vector<keyFrame*>& vlocalKF)
{
	ceres::Problem problem;
	ceres::LossFunction* loss_function = NULL;
  	ceres::LocalParameterization* angle_local_parameter = new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>;
	
	std::vector<SE2> vbeforeTwc;
	std::vector<SE2> vTwc;
	int coutIdx = 30;
	std::cout << "vlocalKF[0]: " << vlocalKF[0]->mnId << std::endl;
	for (size_t i = 0; i < vlocalKF.size()-1; i++)
	{
		keyFrame* akf = vlocalKF[i];
		keyFrame* bkf = vlocalKF[i+1];
		SE2 constraint = convert::getDetlaTcc(akf->mpF->mGtPose, bkf->mpF->mGtPose);
		SE2 Twca = akf->mpF->se2Twc;
		SE2 Twcb = bkf->mpF->se2Twc;
		SE2 tbcpy = Twcb;
		vbeforeTwc.push_back(tbcpy);

		if (vlocalKF[0]->mnId == coutIdx)
		{
			std::cout << "vlocalKF[i]: " << vlocalKF[i]->mnId << std::endl;
			std::cout << "a: " << Twca.x << " - " << Twca.y << " - " << Twca.theta << std::endl;
			std::cout << "c: " << constraint.x << " - " << constraint.y << " - " << constraint.theta << std::endl;
			std::cout << "b: " << Twcb.x << " - " << Twcb.y << " - " << Twcb.theta << std::endl;
		}

		const Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
		ceres::CostFunction* cost_function = PoseGraph2dError::Create(constraint.x, constraint.y, constraint.theta, sqrt_information);
		problem.AddResidualBlock(cost_function, loss_function,
								&Twca.x, &Twca.y, &Twca.theta,
								&Twcb.x, &Twcb.y, &Twcb.theta);

		problem.SetParameterization(&Twca.theta, angle_local_parameter);
		problem.SetParameterization(&Twcb.theta, angle_local_parameter);

		if (i == 0)
		{
			problem.SetParameterBlockConstant(&Twca.x);
			problem.SetParameterBlockConstant(&Twca.y);
			problem.SetParameterBlockConstant(&Twca.theta);
		}	

		vTwc.push_back(Twcb);
	}
	
	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options,&problem,&summary);

	// std::cout << summary.FullReport() << '\n';
	// std::cout << summary.BriefReport() << std::endl;

	// std::cout << "vTwc: " << vTwc.size() << " vs " << vlocalKF.size() << std::endl;

	// for (size_t i = 0; i < vTwc.size(); i++)
	// {
	// 	std::cout << "KF_" << vlocalKF[i+1]->mnId << ", its pose was " 
	// 			<< vlocalKF[i+1]->mpF->se2Twc.x << " - " << vlocalKF[i+1]->mpF->se2Twc.y
	// 			<< " - " << vlocalKF[i+1]->mpF->se2Twc.theta << std::endl;

	// 	vlocalKF[i+1]->mpF->setTwc(vTwc[i]);

	// 	std::cout << "KF_" << vlocalKF[i+1]->mnId << ", its pose now " 
	// 			<< vlocalKF[i+1]->mpF->se2Twc.x << " - " << vlocalKF[i+1]->mpF->se2Twc.y
	// 			<< " - " << vlocalKF[i+1]->mpF->se2Twc.theta << std::endl;
	// }
	
	if (vlocalKF[0]->mnId == coutIdx)
	{
		for (size_t i = 0; i < vTwc.size(); i++)
		{
			std::cout << std::endl << std::endl;
			std::cout << "before: " << vbeforeTwc[i].x << " - " << vbeforeTwc[i].y << " - " << vbeforeTwc[i].theta << std::endl;
			std::cout << "update: " << vTwc[i].x << " - " << vTwc[i].y << " - " << vTwc[i].theta << std::endl;
		}
	}
	
	if(!summary.IsSolutionUsable())
    {
		std::cout << "~~~~~~~~~~~~ wrong ~~~~~~~~~~~~" << std::endl;
        return;
    }
}

void optimizer::testposeGraphOptimize()
{
	ceres::Problem problem;
	ceres::LossFunction* loss_function = NULL;
  	ceres::LocalParameterization* angle_local_parameter = new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>;
	SE2 Twca = SE2(1.2, 45.2, 0.1);
	SE2 con = SE2(10.9, 50.3, 1.3);
	SE2 Twcb = SE2(8.4, 145.9, 1.1);
	SE2 targ = SE2(1.2+10.9, 45.2+50.3, 0.1+1.3);

	std::cout << "before: " << std::endl;
	std::cout << "Twca: " << Twca.x << " - " << Twca.y << " - " << Twca.theta << std::endl;
	std::cout << "con: " << con.x << " - " << con.y << " - " << con.theta << std::endl;
	std::cout << "Twcb: " << Twcb.x << " - " << Twcb.y << " - " << Twcb.theta << std::endl;

	const Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
	ceres::CostFunction* cost_function = PoseGraph2dError::Create(con.x, con.y, con.theta, sqrt_information);
	problem.AddResidualBlock(cost_function, loss_function,
							&Twca.x, &Twca.y, &Twca.theta,
							&Twcb.x, &Twcb.y, &Twcb.theta);

	problem.SetParameterBlockConstant(&Twca.x);
	problem.SetParameterBlockConstant(&Twca.y);
	problem.SetParameterBlockConstant(&Twca.theta);

	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options,&problem,&summary);

	std::cout << "after: " << std::endl;
	std::cout << "Twca: "<< Twca.x << " - " << Twca.y << " - " << Twca.theta << std::endl;
	std::cout << "con: "<< con.x << " - " << con.y << " - " << con.theta << std::endl;
	std::cout << "Twcb: "<< Twcb.x << " - " << Twcb.y << " - " << Twcb.theta << std::endl;
	std::cout << "targ: " << targ.x << " - " << targ.y << " - " << targ.theta << std::endl;
	std::cout << std::endl;

	return;
}
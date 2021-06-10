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
	if (vlocalKF.size() < 10)
	{
		return;
	}
	
	std::cout << " ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ optimize with begin of: " << vlocalKF[0]->mpF->idx << std::endl;
	std::cout << " ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ optimize with end of: " << vlocalKF[vlocalKF.size()-1]->mpF->idx << std::endl;

	ceres::Problem problem;
	ceres::LossFunction* loss_function = NULL;
  	ceres::LocalParameterization* angle_local_parameter = new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>;
	
	std::vector<SE2> vTab, vTc, vTarg;
	for (size_t i = 0; i < vlocalKF.size(); i++)
	{
		SE2 t = vlocalKF[i]->mpF->se2Twc;
		vTab.push_back(t);
	}
	
	for (size_t i = 0; i < vlocalKF.size()-1; i++)
	{
		keyFrame* akf = vlocalKF[i];
		keyFrame* bkf = vlocalKF[i+1];
		SE2 constraint = convert::getDetlaTcc(akf->mpF->mGtPose, bkf->mpF->mGtPose);
		// SE2 constraint = bkf->mpF->mGtPose - akf->mpF->mGtPose; //convert::getDetlaTcc(akf->mpF->mGtPose, bkf->mpF->mGtPose);
		vTc.push_back(constraint);
	}

	for (size_t i = 0; i < vTc.size(); i++)
	{
		SE2 targ = vTab[i] + vTc[i];
		vTarg.push_back(targ);
	}
	
	// std::cout << "before: " << std::endl;
	// for (size_t i = 0; i < vTab.size(); i++)
	// {
	// 	// std::cout << "i: " << i << vTab[i].x << ", " << vTab[i].y << ", " << vTab[i].theta << std::endl;
	// }

	// std::vector<SE2> verrBefore;
	// for (size_t i = 0; i < vTarg.size(); i++)
	// {
	// 	SE2 err = vTab[i+1] - vTarg[i];
	// 	err.x = std::abs(err.x);
	// 	err.y = std::abs(err.y);
	// 	err.theta = std::abs(err.theta);
	// 	verrBefore.push_back(err);
	// }
	

	Eigen::Matrix3d Sigma_vk = Eigen::Matrix3d::Identity();
	Sigma_vk(0,0) = 1.0 / (config::noiseX * config::noiseX);
	Sigma_vk(1,1) = 1.0 / (config::noiseY * config::noiseY);
	Sigma_vk(2,2) = 1.0 / (config::noiseYaw * config::noiseYaw);
	Eigen::Matrix3d sqrt_information = Sigma_vk.llt().matrixL();
	ceres::CostFunction* cost_function = PoseGraph2dError::Create(vTc[0].x, vTc[0].y, vTc[0].theta, sqrt_information);
	problem.AddResidualBlock(cost_function, loss_function,
							&vTab[0].x, &vTab[0].y, &vTab[0].theta,
							&vTab[1].x, &vTab[1].y, &vTab[1].theta);
	problem.SetParameterization(&vTab[0].theta, angle_local_parameter);
	problem.SetParameterization(&vTab[1].theta, angle_local_parameter);
	{
		problem.SetParameterBlockConstant(&vTab[0].x);
		problem.SetParameterBlockConstant(&vTab[0].y);
		problem.SetParameterBlockConstant(&vTab[0].theta);
	}

	for (size_t i = 1; i < vTc.size(); i++)
	{
		ceres::CostFunction* cost_function = PoseGraph2dError::Create(vTc[i].x, vTc[i].y, vTc[i].theta, sqrt_information);
		problem.AddResidualBlock(cost_function, loss_function,
								&vTab[i].x, &vTab[i].y, &vTab[i].theta,
								&vTab[i+1].x, &vTab[i+1].y, &vTab[i+1].theta);

		problem.SetParameterization(&vTab[i].theta, angle_local_parameter);
		problem.SetParameterization(&vTab[i+1].theta, angle_local_parameter);
	}
	
	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options,&problem,&summary);

	for (size_t i = 0; i < vlocalKF.size(); i++)
	{
		vlocalKF[i]->mpF->setTwc(vTab[i]);
	}

	// std::cout << "after: " << std::endl;
	// for (size_t i = 0; i < vTab.size(); i++)
	// {
	// 	std::cout << "i: " << i << vTab[i].x << ", " << vTab[i].y << ", " << vTab[i].theta << std::endl;
	// }

	// std::vector<SE2> verrAfter;
	// for (size_t i = 0; i < vTarg.size(); i++)
	// {
	// 	SE2 err = vTab[i+1] - vTarg[i];
	// 	err.x = std::abs(err.x);
	// 	err.y = std::abs(err.y);
	// 	err.theta = std::abs(err.theta);
	// 	verrAfter.push_back(err);

	// 	SE2 der = verrBefore[i] - err;
	// 	std::cout << "Err less? " << der.x << " - " << der.y << " - " << der.theta << std::endl;
	// }

	// std::cout << std::endl << std::endl;
	// if(!summary.IsSolutionUsable())
    // {
	// 	std::cout << "~~~~~~~~~~~~ wrong ~~~~~~~~~~~~" << std::endl;
    //     return;
    // }
}

void optimizer::testposeGraphOptimize()
{
	ceres::Problem problem;
	ceres::LossFunction* loss_function = NULL;
  	ceres::LocalParameterization* angle_local_parameter = new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>;
	
	std::vector<SE2> vTwa, vCon, vTwb, vTarg;
	SE2 Twca1 = SE2(3.88071, -11.824, -0.818864);
	SE2 con1 = SE2(-0.485378, -0.0211419, 0.077129);
	SE2 Twcb1 = SE2(3.58552, -11.8249, -0.70348);
	SE2 targ1 = Twca1 + con1;

	SE2 Twca2 = SE2(3.58552, -11.8249, -0.70348);
	SE2 con2 = SE2(-0.569105, -0.0113562, 0.040322);
	SE2 Twcb2 = SE2(3.3125, -11.3369, -0.654073);
	SE2 targ2 = Twca2 + con2;

	SE2 Twca3 = SE2(3.3125, -11.3369, -0.654073);
	SE2 con3 = SE2(-0.534153, -0.0123651, 0.052595);
	SE2 Twcb3 = SE2(3.00054, -11.2056, -0.604631);
	SE2 targ3 = Twca3 + con3;

	SE2 Twca4 = SE2(3.00054, -11.2056, -0.604631);
	SE2 con4 = SE2(-0.472063, -0.0160752, 0.075104);
	SE2 Twcb4 = SE2(2.44532, -10.9941, -0.538532);
	SE2 targ4 = Twca4 + con4;

	SE2 Twca5 = SE2(2.44532, -10.9941, -0.538532);
	SE2 con5 = SE2(-0.691511, -0.05322, 0.148659);
	SE2 Twcb5 = SE2(2.1246, -10.9069, -0.426423);
	SE2 targ5 = Twca5 + con5;

	SE2 Twca6 = SE2(2.1246, -10.9069, -0.426423);
	SE2 con6 = SE2(-0.924068, -0.0725101, 0.154207);
	SE2 Twcb6 = SE2(1.53758, -10.7538, -0.243267);
	SE2 targ6 = Twca6 + con6;

	SE2 Twca7 = SE2(1.53758, -10.7538, -0.243267);
	SE2 con7 = SE2(-0.442428, -0.0130867, 0.054696);
	SE2 Twcb7 = SE2(0.984783, -10.6869, -0.17613);
	SE2 targ7 = Twca7 + con7;

	SE2 Twca8 = SE2(0.984783, -10.6869, -0.17613);
	SE2 con8 = SE2(-0.319173, -0.00217936, 0.012947);
	SE2 Twcb8 = SE2(0.713422, -10.7212, -0.126577);
	SE2 targ8 = Twca8 + con8;

	SE2 Twca9 = SE2(0.713422, -10.7212, -0.126577);
	SE2 con9 = SE2(-0.344641, -0.000680231, 0.00739);
	SE2 Twcb9 = SE2(0.556632, -10.8736, -0.14155);
	SE2 targ9 = Twca9 + con9;

	vTwa.push_back(Twca1);
	vTwa.push_back(Twca2);
	vTwa.push_back(Twca3);
	vTwa.push_back(Twca4);
	vTwa.push_back(Twca5);
	vTwa.push_back(Twca6);
	vTwa.push_back(Twca7);
	vTwa.push_back(Twca8);
	vTwa.push_back(Twca9);

	vCon.push_back(con1);
	vCon.push_back(con2);
	vCon.push_back(con3);
	vCon.push_back(con4);
	vCon.push_back(con5);
	vCon.push_back(con6);
	vCon.push_back(con7);
	vCon.push_back(con8);
	vCon.push_back(con9);

	vTwb.push_back(Twcb1);
	vTwb.push_back(Twcb2);
	vTwb.push_back(Twcb3);
	vTwb.push_back(Twcb4);
	vTwb.push_back(Twcb5);
	vTwb.push_back(Twcb6);
	vTwb.push_back(Twcb7);
	vTwb.push_back(Twcb8);
	vTwb.push_back(Twcb9);

	vTarg.push_back(targ1);
	vTarg.push_back(targ2);
	vTarg.push_back(targ3);
	vTarg.push_back(targ4);
	vTarg.push_back(targ5);
	vTarg.push_back(targ6);
	vTarg.push_back(targ7);
	vTarg.push_back(targ8);
	vTarg.push_back(targ9);

	// std::cout << "before: " << std::endl;
	// for (size_t i = 0; i < vTwb.size(); i++)
	// {
	// 	std::cout << "i-a: " << vTwa[i].x << ", " << vTwa[i].y << ", " << vTwa[i].theta << std::endl;
	// 	std::cout << "i-b: " << vTwb[i].x << ", " << vTwb[i].y << ", " << vTwb[i].theta << std::endl;
	// }

	std::cout << "detla before: " << std::endl;
	for (size_t i = 0; i < vTwb.size(); i++)
	{
		// std::cout << "i-a: " << vTwa[i].x << ", " << vTwa[i].y << ", " << vTwa[i].theta << std::endl;
		std::cout << "i-" << i << " " << vTwb[i].x-vTarg[i].x << ", " << vTwb[i].y-vTarg[i].y << ", " << vTwb[i].theta-vTarg[i].theta << std::endl;
	}

	for (size_t i = 0; i < vTwa.size(); i++)
	{
		const Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
		ceres::CostFunction* cost_function = PoseGraph2dError::Create(vCon[i].x, vCon[i].y, vCon[i].theta, sqrt_information);
		problem.AddResidualBlock(cost_function, loss_function,
								&vTwa[i].x, &vTwa[i].y, &vTwa[i].theta,
								&vTwb[i].x, &vTwb[i].y, &vTwb[i].theta);

		problem.SetParameterization(&vTwa[i].theta, angle_local_parameter);
		problem.SetParameterization(&vTwb[i].theta, angle_local_parameter);
	}

	problem.SetParameterBlockConstant(&vTwa[0].x);
	problem.SetParameterBlockConstant(&vTwa[0].y);
	problem.SetParameterBlockConstant(&vTwa[0].theta);
	
	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options,&problem,&summary);

	// std::cout << "after: " << std::endl;
	// for (size_t i = 0; i < vTwb.size(); i++)
	// {
	// 	// std::cout << "i-a: " << vTwa[i].x << ", " << vTwa[i].y << ", " << vTwa[i].theta << std::endl;
	// 	std::cout << "i-b: " << vTwb[i].x << ", " << vTwb[i].y << ", " << vTwb[i].theta << std::endl;
	// 	std::cout << "i-t: " << vTarg[i].x << ", " << vTarg[i].y << ", " << vTarg[i].theta << std::endl;
	// }
	// std::cout << std::endl;

	std::cout << "detla after: " << std::endl;
	for (size_t i = 0; i < vTwb.size(); i++)
	{
		// std::cout << "i-a: " << vTwa[i].x << ", " << vTwa[i].y << ", " << vTwa[i].theta << std::endl;
		std::cout << "i-" << i << " " << vTwb[i].x-vTarg[i].x << ", " << vTwb[i].y-vTarg[i].y << ", " << vTwb[i].theta-vTarg[i].theta << std::endl;
	}

	return;
}
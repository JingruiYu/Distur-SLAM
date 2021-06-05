/*************************************************************************
	> File Name: src/poseSolver.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时58分06秒
 ************************************************************************/

#include "poseSolver.h"
#include "Frame.h"

const float poseSolver::bRow = 384.0;
const float poseSolver::bCol = 384.0;
const float poseSolver::cor = 1;
const float poseSolver::p2m = 0.03984*cor;
const float poseSolver::m2p = 25.1/cor;
const float poseSolver::rear = 1.393;

poseSolver::poseSolver(/* args */)
{
}

poseSolver::~poseSolver()
{
}


cv::Mat poseSolver::ICP2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2)
{
	cv::Mat Tc1c2 = cv::Mat::eye(3,3,CV_32F);
	// poseSolver::FindRtICP2D();
	std::vector<cv::Point2f> vKeys1 = kpts1_kpts2.first;
	std::vector<cv::Point2f> vKeys2 = kpts1_kpts2.second;
	std::vector<cv::DMatch> vMatches;
    std::vector<cv::Point2f> vBpoint1, vBpoint2;

	for (size_t i = 0; i < vKeys1.size(); i++)
	{
		cv::DMatch imatch;
		imatch.queryIdx = i;
		imatch.trainIdx = i;
		vMatches.push_back(imatch);

        cv::Point2f p1 = vKeys1[i];
        cv::Point2f p2 = vKeys2[i];

        // Frame::bpoint bp1(p1.x,p1.y,bRow,bCol);
        // Frame::bpoint bp2(p2.x,p2.y,bRow,bCol);

        // cv::Point2f xyp1(bp1.X,bp1.Y);
        // cv::Point2f xyp2(bp2.X,bp2.Y);

        cv::Point2f xyp11 = convert::BirdviewPT2XY(p1);
        cv::Point2f xyp22 = convert::BirdviewPT2XY(p2);

        vBpoint1.push_back(xyp11);
        vBpoint2.push_back(xyp22);
	}
	std::vector<bool> vbMatchesInliers(vMatches.size(),false);

	assert(vKeys1.size() == vbMatchesInliers.size());
	// std::cout << "vKeys1.size(): " << vKeys1.size() << " - " << "vKeys2.size(): " << vKeys2.size() << std::endl;
	// std::cout << "vMatches.size(): " << vMatches.size() << " - " << "vbMatchesInliers.size(): " << vbMatchesInliers.size() << std::endl;

	cv::Mat Rc1c2,tc1c2;
	poseSolver::FindRtICP2D(vBpoint1, vBpoint2, vMatches, vbMatchesInliers, Rc1c2, tc1c2, 0.03984, 200);
	
	Rc1c2.copyTo(Tc1c2.rowRange(0,2).colRange(0,2));
	tc1c2.copyTo(Tc1c2.rowRange(0,2).col(2));
	// std::cout << "R: " << std::endl << R << std::endl << " t: " << t.t() << std::endl << "Tcc:" << std::endl << Tcc << std::endl;

	return Tc1c2;
}

int poseSolver::FindRtICP2D(const std::vector<cv::Point2f> &vKeysXY1, const std::vector<cv::Point2f> &vKeysXY2, const std::vector<cv::DMatch> &vMatches,
                            std::vector<bool> &vbMatchesInliers, cv::Mat &R, cv::Mat &t, float sigma, int mMaxIterations)
{
    const int N = vMatches.size();
    // Indices for minimum set selection
    std::vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    std::vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 2 points for each RANSAC iteration
    std::vector<std::vector<size_t> > vSets(mMaxIterations,std::vector<size_t>(2,0));

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<2; j++)
        {
            int randi = rand() % (vAvailableIndices.size()-1); //Here need debug
            int idx = vAvailableIndices[randi];

            vSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }


    // Best Results variables
    int bestNumInliers=0;
    vbMatchesInliers = std::vector<bool>(N,false);

    // Iteration variables
    std::vector<cv::Point2f> vP1i(2);
    std::vector<cv::Point2f> vP2i(2);
    cv::Mat Ri,ti;
    std::vector<bool> vbCurrentInliers(N,false);
    int nNumInliers=0;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<2; j++)
        {
            int idx = vSets[it][j];

            vP1i[j] = vKeysXY1[vMatches[idx].queryIdx];
            vP2i[j] = vKeysXY2[vMatches[idx].trainIdx];
        }

		poseSolver::ComputeRtICP2D(vP1i,vP2i,Ri,ti);
        nNumInliers = poseSolver::CheckRtICP2D(Ri,ti,vKeysXY1,vKeysXY2,vMatches,vbCurrentInliers,sigma);

        if(nNumInliers > bestNumInliers)
        {
            R = Ri.clone();
            t = ti.clone();
            vbMatchesInliers = vbCurrentInliers;
            bestNumInliers = nNumInliers;
        }
    }

    return bestNumInliers;
}

bool poseSolver::ComputeRtICP2D(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2, cv::Mat &R, cv::Mat &t)
{
    int N = vP1.size();

    // centroid
    cv::Point2f p1=cv::Point2f(),p2=cv::Point2f();
    for(int k=0;k<N;k++)
    {
        p1+=vP1[k];
        p2+=vP2[k];
    }

	p1.x = p1.x / N;
	p1.y = p1.y / N;
	p2.x = p2.x / N;
	p2.y = p2.y / N;

    // minus centroid
    std::vector<cv::Point2f> vQ1(N), vQ2(N);
    for(int k=0;k<N;k++)
    {
        vQ1[k] = vP1[k]-p1;
        vQ2[k] = vP2[k]-p2;
    }

    cv::Mat W(2,2,CV_32F,cv::Scalar(0));
    for(int k=0;k<N;k++)
    {
        W+=cv::Mat(vQ1[k])*cv::Mat(vQ2[k]).t();
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(W,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    R = u*vt;
    if(cv::determinant(R)<0)
    {
        u.col(1) = -u.col(1);
        R = u*vt;
    }

    t = cv::Mat(p1)-R*cv::Mat(p2);

    return true;
}

int poseSolver::CheckRtICP2D(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::Point2f> &vP2D1, const std::vector<cv::Point2f> &vP2D2,
                const std::vector<cv::DMatch> &vMatches12, std::vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = vMatches12.size();

    vbMatchesInliers.resize(N,false);
    const float th = 5.991;
    const float invSigmaSquare = 1.0/(sigma*sigma);
    int nNumInliers = 0;

    for(int i=0;i<N;i++)
    {
        bool bIn = true;

        cv::Mat e = cv::Mat(vP2D1[vMatches12[i].queryIdx])-(R*cv::Mat(vP2D2[vMatches12[i].trainIdx])+t);
        float ex = e.at<float>(0), ey = e.at<float>(1);
        float squareDist = ex*ex+ey*ey;
        float chiSquare = squareDist*invSigmaSquare;

        if(chiSquare>th)
        {
            bIn = false;
        }
        
        if(bIn)
        {
            nNumInliers++;
            vbMatchesInliers[i] = true;
        }
        else
        {
            vbMatchesInliers[i] = false;
        }
    }
    // cout<<"Current Inliers = "<<nNumInliers<<"/"<<N<<endl; 
    return nNumInliers;
}

cv::Mat poseSolver::FindtICP2D(const std::vector<cv::Point2f> &vKeys1, const std::vector<cv::Point2f> &vKeys2, Frame* lastFrame, Frame* curFrame, float cur_theta)
{
    cv::Mat Twc1 = lastFrame->Twc.clone();
    cv::Mat Rwc1 = Twc1.rowRange(0,2).colRange(0,2);
    cv::Mat twc1 = Twc1.rowRange(0,2).col(2);

    cv::Mat Twc2 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat Rwc2 = convert::tocvRMat(cur_theta);
    cv::Mat twc2 = cv::Mat::zeros(2,1,CV_32F);
    cv::Mat Tc2w = cv::Mat::eye(3,3,CV_32F);
    cv::Mat Rc2w = Rwc2.inv();
    cv::Mat tc2w = cv::Mat::zeros(2,1,CV_32F);
    
    int N1 = vKeys1.size();
    int N2 = vKeys2.size();
    std::vector<cv::Mat> vPc1inc2, vPc2;
    for (int i = 0; i < N1; i++)
    {
        cv::Point2f p1 = vKeys1[i];
        cv::Point2f pc1 = convert::BirdviewPT2XY(p1);
        cv::Mat pc1m = convert::tocvMat(pc1);
        cv::Mat pw1 = Rwc1 * pc1m + twc1;
        cv::Mat p1inc2 = Rc2w * pw1;

        vPc1inc2.push_back(p1inc2);
    }

    for (int i = 0; i < N2; i++)
    {
        cv::Point2f p2 = vKeys2[i];
        cv::Point2f pc2 = convert::BirdviewPT2XY(p2);
        cv::Mat pc2m = convert::tocvMat(pc2);
        vPc2.push_back(pc2m);
    }
    
    cv::Mat vCentor1 = cv::Mat::zeros(2,1,CV_32F);
    cv::Mat vCentor2 = cv::Mat::zeros(2,1,CV_32F);

    for (int i = 0; i < N1; i++)
    {
        vCentor1.at<float>(0) += vPc1inc2[i].at<float>(0);
        vCentor1.at<float>(1) += vPc1inc2[i].at<float>(1);
    }

    for (int i = 0; i < N2; i++)
    {
        vCentor2.at<float>(0) += vPc2[i].at<float>(0);
        vCentor2.at<float>(1) += vPc2[i].at<float>(1);
    }
    
    vCentor1.at<float>(0) = vCentor1.at<float>(0) / N1;
    vCentor1.at<float>(1) = vCentor1.at<float>(1) / N1;
    vCentor2.at<float>(0) = vCentor2.at<float>(0) / N2;
    vCentor2.at<float>(1) = vCentor2.at<float>(1) / N2;

    float scale = 1;
    tc2w.at<float>(0) = scale*(vCentor2.at<float>(0) - vCentor1.at<float>(0));
    tc2w.at<float>(1) = scale*(vCentor2.at<float>(1) - vCentor1.at<float>(1));

    Rc2w.copyTo(Tc2w.rowRange(0,2).colRange(0,2));
    tc2w.copyTo(Tc2w.rowRange(0,2).col(2));
    Twc2 = Tc2w.inv();

    // std::cout << "Tc2w : "<< std::endl << Tc2w << std::endl;
    // std::cout << "Rc2w : "<< std::endl << Rc2w << std::endl;
    // std::cout << "tc2w : "<< std::endl << tc2w << std::endl;

    // std::cout << "Twc2 : "<< std::endl << Twc2 << std::endl;
    // std::cout << "Rwc2 : "<< std::endl << Rwc2 << std::endl;
    // std::cout << "twc2 : "<< std::endl << twc2 << std::endl;

    return Twc2;
}

cv::Mat poseSolver::FindtByLinePt(Frame* refFrame, Frame* curFrame, float cur_theta)
{
    std::vector<cv::Point2f> vKeys1 = refFrame->getEndPtFromLine();
    std::vector<cv::Point2f> vKeys2 = curFrame->getEndPtFromLine();

    if (vKeys1.size() < 5 || vKeys2.size() < 5)
    {
        std::cout << "poseSolver::FindtByLinePt getchar() " << std::endl;
        std::cin.get();
    }
    
    // std::cout << "vKeys1: " << vKeys1.size() << " - vKeys2: " << vKeys2.size() << std::endl;
    std::vector<cv::Point2f> vKeys2OF;
    std::vector<uchar> status1;
	std::vector<float> errs1;
    cv::calcOpticalFlowPyrLK(refFrame->img_gray,curFrame->img_gray,vKeys1,vKeys2OF,status1,errs1,cv::Size(21,21),3);

    cv::Mat Twc2 = poseSolver::FindtICP2D(vKeys1, vKeys2OF, refFrame, curFrame, cur_theta);

    return Twc2;
}
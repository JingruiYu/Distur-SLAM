/*************************************************************************
	> File Name: src/poseSolver.cpp
	> Author: yujr
	> Mail: 
	> Created Time: 2021年05月19日 星期三 18时58分06秒
 ************************************************************************/

#include "poseSolver.h"

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

cv::Point3f poseSolver::birdPixel2Camera(cv::Point2f pt)
{
	cv::Point3f p;
    p.x = (bRow/2-pt.y)*p2m+rear;
    p.y = (bCol/2-pt.x)*p2m;
    p.z = 0;

    return p;
}

cv::Point2f poseSolver::birdCamera2Pixel(cv::Point3f p)
{
	cv::Point2f pt;
    pt.x = bCol/2-p.y*m2p;
    pt.y = bRow/2-(p.x-rear)*m2p;
    return pt;
}

cv::Mat poseSolver::ICP2D(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > &kpts1_kpts2)
{
	cv::Mat Tc1c2 = cv::Mat::eye(3,3,CV_32F);
	// poseSolver::FindRtICP2D();
	std::vector<cv::Point2f> vKeys1 = kpts1_kpts2.first;
	std::vector<cv::Point2f> vKeys2 = kpts1_kpts2.second;
	std::vector<cv::DMatch> vMatches;
	for (size_t i = 0; i < vKeys1.size(); i++)
	{
		cv::DMatch imatch;
		imatch.queryIdx = i;
		imatch.trainIdx = i;
		vMatches.push_back(imatch);
	}
	std::vector<bool> vbMatchesInliers(vMatches.size(),false);

	assert(vKeys1.size() == vbMatchesInliers.size());
	// std::cout << "vKeys1.size(): " << vKeys1.size() << " - " << "vKeys2.size(): " << vKeys2.size() << std::endl;
	// std::cout << "vMatches.size(): " << vMatches.size() << " - " << "vbMatchesInliers.size(): " << vbMatchesInliers.size() << std::endl;

	cv::Mat Rc1c2,tc1c2;
	poseSolver::FindRtICP2D(vKeys1, vKeys2, vMatches, vbMatchesInliers, Rc1c2, tc1c2, 0.03984, 200);
	
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

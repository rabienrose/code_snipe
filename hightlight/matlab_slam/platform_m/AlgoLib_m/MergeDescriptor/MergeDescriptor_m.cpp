#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <cassert>
#include <opencvmex.hpp>
#include "MergeDescriptor_m.h"

int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    assert(nrhs >= 1);
    assert(nlhs >= 1);

    
    bool bCopy = false;
    cv::Mat tempDesc;
    ocvMxArrayToImage_uint8(prhs[0],tempDesc);
    tempDesc = tempDesc.t();
    int descCount = tempDesc.cols;
    int descWidth = tempDesc.rows;
    
    //std::cout<<tempDesc<<std::endl;
    const size_t N = descCount;
    

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = DescriptorDistance(tempDesc.col(i),tempDesc.col(j));
            //std::cout<<distij<<std::endl;
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        std::vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    
    cv::Mat mDescriptor = tempDesc.col(BestIdx);
    mDescriptor = mDescriptor.t();
    
    //std::cout<<mDescriptor<<std::endl;
    plhs[0] = ocvMxArrayFromImage_uint8(mDescriptor);       
}
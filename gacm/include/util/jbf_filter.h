/**
* This file is part of GAC-Mapping.
*
* Copyright (C) 2020-2022 JinHao He, Yilin Zhu / RAPID Lab, Sun Yat-Sen University 
* 
* For more information see <https://github.com/SYSU-RoboticsLab/GAC-Mapping>
*
* GAC-Mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* GAC-Mapping is distributed to support research and development of
* Ground-Aerial heterogeneous multi-agent system, but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GAC-Mapping. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>
#include <time.h>
#include <sstream>
#include <fstream>
#include"omp.h"
#include"ip_basic.h"

ushort empty_value = 0;

using namespace std;
#define JTFLOOP 10
#define JTFLOOPCLMF 1;

bool is_filled = false;
int jtf_loop_num = JTFLOOP;
int jtf_loop_num1 = JTFLOOPCLMF;
cv::Mat greymask;
int pixelcount = 0;
float gaussian_kernel[21][21];
float color_kernel[256];
float depth_kernel[65536];
int o_win_size = 0;
double o_sigma_s = 1;
double o_sigma_r = 1;
double o_sigma_d = 1;



double gaussian_winsize[4] = {3,3,3,3};
double gaussian_sigma[4] = {1,1,1,1};


int loop_num[5] = {220,75,1,1,1};
int jbf_winsize[5] = {7,7,7,7,7};
int gf_r[5] = {4,5,11,11,11};
double jbf_sigmas[5] = {3.5,3.5,3.5,3.5,3.5};
double jbf_sigmar[5] = {8, 8, 8, 8, 8};
double jbf_sigmad[5] = {2300, 2300, 2300, 2300, 2300};







// Guided Filter主函数
cv::Mat guidedFilter(cv::Mat I, cv::Mat p, int r, double eps) {  
  /* 
  % GUIDEDFILTER   O(1) time implementation of guided filter. 
  % 
  %   - guidance image: I (should be a gray-scale/single channel image) 
  %   - filtering input image: p (should be a gray-scale/single channel image) 
  %   - local window radius: r 
  %   - regularization parameter: eps 
  */  
   
  cv::Mat _I;  
  I.convertTo(_I, CV_64FC1);  
  I = _I;  
   
  cv::Mat _p;  
  p.convertTo(_p, CV_64FC1);  
  p = _p;  
   
  //[hei, wid] = size(I);  
  int hei = I.rows;  
  int wid = I.cols;  
   
  //N = boxfilter(ones(hei, wid), r); % the size of each local patch; N=(2r+1)^2 except for boundary pixels.  
  cv::Mat N;  
  cv::boxFilter(cv::Mat::ones(hei, wid, I.type()), N, CV_64FC1, cv::Size(r, r));  
   
  //mean_I = boxfilter(I, r) ./ N;  
  cv::Mat mean_I;  
  cv::boxFilter(I, mean_I, CV_64FC1, cv::Size(r, r));  
    
  //mean_p = boxfilter(p, r) ./ N;  
  cv::Mat mean_p;  
  cv::boxFilter(p, mean_p, CV_64FC1, cv::Size(r, r));  
   
  //mean_Ip = boxfilter(I.*p, r) ./ N;  
  cv::Mat mean_Ip;  
  cv::boxFilter(I.mul(p), mean_Ip, CV_64FC1, cv::Size(r, r));  
   
  //cov_Ip = mean_Ip - mean_I .* mean_p; % this is the covariance of (I, p) in each local patch.  
  cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);  
   
  //mean_II = boxfilter(I.*I, r) ./ N;  
  cv::Mat mean_II;  
  cv::boxFilter(I.mul(I), mean_II, CV_64FC1, cv::Size(r, r));  
   
  //var_I = mean_II - mean_I .* mean_I;  
  cv::Mat var_I = mean_II - mean_I.mul(mean_I);  
   
  //a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;     
  cv::Mat a = cov_Ip/(var_I + eps);  
   
  //b = mean_p - a .* mean_I; % Eqn. (6) in the paper;  
  cv::Mat b = mean_p - a.mul(mean_I);  
   
  //mean_a = boxfilter(a, r) ./ N;  
  cv::Mat mean_a;  
  cv::boxFilter(a, mean_a, CV_64FC1, cv::Size(r, r));  
  //mean_a = mean_a/N;  
   
  //mean_b = boxfilter(b, r) ./ N;  
  cv::Mat mean_b;  
  cv::boxFilter(b, mean_b, CV_64FC1, cv::Size(r, r));  
  //mean_b = mean_b/N;  
   
  //q = mean_a .* I + mean_b; % Eqn. (8) in the paper;  
  cv::Mat q = mean_a.mul(I) + mean_b;  
  cv::Mat q_;
  q.convertTo(q_, CV_16UC1);
  return q_;  
}  




// 归一化图像到0~1的32FC1类型
cv::Mat mat2gray(cv::Mat inMat) {
    //idea: to be scaled between 0 and 1, compute for each value: val := (val - minVal)/(maxVal-minVal)
    if (inMat.channels() != 1) std::cout << "mat2gray only works for single channel floating point matrices" << std::endl;
    // cout << "inmat channels " << inMat.channels() << endl;
    // here we assume floating point matrices (single/double precision both ok)
    double minVal, maxVal;

    //cv::Mat scaledMat(inMat.rows, inMat.cols, CV_32F, cv::Scalar(0));
    cv::Mat scaledMat;
    inMat.convertTo(scaledMat, CV_32F);
    cv::minMaxLoc(scaledMat, &minVal, &maxVal);
    // cout << " Max " << maxVal << " Min " << minVal << endl;
    scaledMat = scaledMat - cv::Mat(scaledMat.size(), scaledMat.type(), cv::Scalar(minVal));
    scaledMat = ((1.0)/(maxVal-minVal))*scaledMat;
    // cv::imshow("scalemat", scaledMat);
    return scaledMat;
}




// 高斯下采样
void origin_gaussian(const cv::Mat &src, cv::Mat &dst, int N, int win_size, double sigma) {
    cv::Mat tempblur;
    cv::Mat tempdist = src.clone();
    
    for (int i = 0; i < N; i++) {
        tempblur = tempdist.clone();
        //GaussianBlur(tempdist, tempblur, cv::Size(win_size,win_size), sigma, sigma); //高斯模糊
        // cv::imshow("test0", tempblur);
        //scaleIntervalSampling(tempblur,tempdist,0.5, 0.5);//等距采样
        // cv::imshow("test", tempdist);
        // cv::waitKey(0);
        //  cv::pyrDown(tempdist, dst,cv::Size(tempdist.cols/2, tempdist.rows/2));
        //  tempdist = dst.clone();
        cv::resize(tempblur, tempdist, cv::Size(tempblur.cols/2, tempblur.rows/2),0,0,cv::INTER_LINEAR);
    }
    dst = tempdist.clone();
}

// 将以landmark为圆心画圆，确保下采样能采到
void fill_depth(cv::Mat &src, int radius) {
    cv::Mat origin = src.clone();
    // #pragma omp parallel for num_threads(2)
    for (int i = 0; i < src.rows; i ++) {
        for (int j = 0; j < src.cols; j++) {
            if (origin.at<ushort>(i,j) != 0)
                cv::circle(src, cv::Point2d(j,i), radius, cv::Scalar(src.at<ushort>(i, j)), -1); // 非空值画半径为radius的圆
        }
    }
}

// 给深度建立mask
void generate_mask( const cv::Mat &src, cv::Mat &mask) {
    mask = cv::Mat(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < src.rows; i ++) {
        for (int j = 0; j < src.cols; j++) {
            if (src.at<ushort>(i,j) != 0)
                mask.at<uchar>(i, j) = 255;
        }
    }
}

// 恢复原始landmark深度
void keep_depth(cv::Mat src, cv::Mat& dst) {
    for (int i = 0; i < src.rows; i ++) {
        for (int j = 0; j < src.cols; j++) {
            if (src.at<ushort>(i,j) != 0 && src.at<ushort>(i,j) < dst.at<ushort>(i, j) + 0.1*256)
                dst.at<ushort>(i, j) = src.at<ushort>(i,j);
        }
    }
}





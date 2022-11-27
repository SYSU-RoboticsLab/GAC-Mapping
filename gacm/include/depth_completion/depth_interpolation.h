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

# ifndef DEPTH_INTERPOLATION_H
# define DEPTH_INTERPOLATION_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ip_basic_dc.h"
#include "joint_bilateral_filter.h"

using namespace std;
int N = 4; //层数

int loop_num[5] = {220,75,1,1,1};
int jbf_winsize[5] = {7,7,7,7,7};
int gf_radius[5] = {4,5,11,11,11};
double jbf_sigmas[5] = {3.5,3.5,3.5,3.5,3.5};
double jbf_sigmar[5] = {8, 8, 8, 8, 8};
double jbf_sigmad[5] = {2300, 2300, 2300, 2300, 2300};


// 强制像素采样保留所有有效点的下采样
void ForcePixelSampling(const cv::Mat &src, cv::Mat &dst, double xRatio, double yRatio) {
    int type = src.type();
    if (type != CV_64FC1) {
        src.convertTo(src, CV_64FC1);
    }
    int rows = static_cast<int>(src.rows * xRatio);
    int cols = static_cast<int>(src.cols * yRatio);
    cv::Mat dstMat(rows, cols, src.type(),cv::Scalar(0));
    int step = cols/100.0;
    // #pragma omp parallel for num_threads(2)
    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            double cur_val = src.at<double>(i,j);
            if (cur_val != 0) {
                int di = static_cast<int>(i * xRatio);
                int dj = static_cast<int>(j * yRatio);
                double dst_val = dstMat.at<double>(di,dj);
                if (dst_val == 0 || dst_val > cur_val) {
                    dstMat.at<double>(di,dj) = cur_val;
                }
            }
        }
    }
    dst = dstMat.clone();
    dst.convertTo(dst, type);
    src.convertTo(src, type);
}

// 高斯下采样
void GaussianSampling(const cv::Mat &src, cv::Mat &dst, int N, int win_size, double sigma) {
    cv::Mat tempblur;
    cv::Mat tempdist = src.clone();
    
    for (int i = 0; i < N; i++) {
        tempblur = tempdist.clone();
        cv::resize(tempblur, tempdist, cv::Size(tempblur.cols/2, tempblur.rows/2),0,0,cv::INTER_LINEAR);
    }
    dst = tempdist.clone();
}




// 恢复原始landmark深度
void KeepDepth(cv::Mat src, cv::Mat& dst) {
    for (int i = 0; i < src.rows; i ++) {
        for (int j = 0; j < src.cols; j++) {
            //if (src.at<ushort>(i,j) != 0 && src.at<ushort>(i,j) < dst.at<ushort>(i, j) + 0.1*256)
            //    dst.at<ushort>(i, j) = src.at<ushort>(i,j);
            if (src.at<double>(i,j) != 0 && src.at<double>(i,j) < dst.at<double>(i, j))
                dst.at<double>(i, j) = src.at<double>(i,j);
        }
    }
}



# endif

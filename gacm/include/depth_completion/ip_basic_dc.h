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

#ifndef IP_BASIC_H
#define IP_BASIC_H

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "eval/depth_util.h"

// Full kernels
const cv::Mat kFullKernel3 = cv::Mat::ones(3,3,CV_8U);
const cv::Mat kFullKernel5 = cv::Mat::ones(5,5,CV_8U);
const cv::Mat kFullKernel7 = cv::Mat::ones(7,7,CV_8U);
const cv::Mat kFullKernel9 = cv::Mat::ones(9,9,CV_8U);
const cv::Mat kFullKernel31 = cv::Mat::ones(31,31,CV_8U);

// 3*3 cross kernel
const cv::Mat kCrossKernel3 =  (cv::Mat_<uchar> (3,3) <<  0,1,0,
                                                     1,1,1,
                                                     0,1,0) ;

// 5*5 cross kernel
const cv::Mat kCrossKernel5 =  (cv::Mat_<uchar> (5,5) <<  0,0,1,0,0,
                                                     0,0,1,0,0,
                                                     1,1,1,1,1,
                                                     0,0,1,0,0,
                                                     0,0,1,0,0) ;

// 7*7 cross kernel
const cv::Mat kCrossKernel7 = (cv::Mat_<uchar>(7,7) <<  0,0,0,1,0,0,0,
                                                   0,0,0,1,0,0,0,
                                                   0,0,0,1,0,0,0,
                                                   1,1,1,1,1,1,1,
                                                   0,0,0,1,0,0,0,
                                                   0,0,0,1,0,0,0,
                                                   0,0,0,1,0,0,0) ;

// 5*5 diamond kernel
const cv::Mat kDiamondKernel5 =  (cv::Mat_<uchar> (5,5) <<  0,0,1,0,0,
                                                       0,1,1,1,0,
                                                       1,1,1,1,1,
                                                       0,1,1,1,0,
                                                       0,0,1,0,0) ;

// 7*7 diamond kernel
const cv::Mat kDiamondKernel7 = (cv::Mat_<uchar>(7,7) <<  0,0,0,1,0,0,0,
                                                     0,0,1,1,1,0,0,
                                                     0,1,1,1,1,1,0,
                                                     1,1,1,1,1,1,1,
                                                     0,1,1,1,1,1,0,
                                                     0,0,1,1,1,0,0,
                                                     0,0,0,1,0,0,0) ;

enum KernelType {
    KERNEL_TYPE_FULL = 0,
    KERNEL_TYPE_CROSS = 1,
    KERNEL_TYPE_DIAMOND = 2
};


// max(100) - origin 
void GetInvertDepth(cv::Mat src, cv::Mat& dst, const cv::Mat& mask) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }

    cv::Mat max = cv::Mat(src.rows, src.cols, CV_64F, cv::Scalar(100));
    cv::subtract(max, src, dst, mask);
    dst.convertTo(dst, type);
}

// generate masks 
void GenerateAllMask(cv::Mat src, cv::Mat& valid, cv::Mat& invalid, cv::Mat& far, cv::Mat& med, cv::Mat& near) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }

    valid = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    invalid = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    far = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    med = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    near = cv::Mat::zeros(src.rows, src.cols, CV_8U);

    for (int i =0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            double cur_val = src.at<double>(i,j);
            if (cur_val == 0 || cur_val >= 256*200) {
                invalid.at<uchar>(i,j) = 1;
            } else {
                valid.at<uchar>(i,j) = 1;
                if (cur_val < 15 * 256) {
                    near.at<uchar>(i,j) = 1;
                } else if (cur_val < 30*256) {
                    med.at<uchar>(i,j) = 1;
                } else if (cur_val < 100*256) {
                    far.at<uchar>(i,j) = 1;
                }
            }
        }
    }
}

void GenerateValidMask(cv::Mat src, cv::Mat& valid) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }

    valid = cv::Mat::zeros(src.rows, src.cols, CV_8U);

    for (int i =0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            double cur_val = src.at<double>(i,j);
            if (cur_val > 0) {
                valid.at<uchar>(i,j) = 1;
            }
        }
    }
}

void GenerateInvalidMask(cv::Mat src, cv::Mat& invalid) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }

    invalid = cv::Mat::zeros(src.rows, src.cols, CV_8U);

    for (int i =0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            double cur_val = src.at<double>(i,j);
            if (cur_val <  0.1) {
                invalid.at<uchar>(i,j) = 1;
            }
        }
    }
}

// fill the top invalid pixels with the first valid value
void ExtendTop(cv::Mat src, cv::Mat& dst) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }
    std::vector<int> top_indices;
    std::vector<double> top_values;
    for (int j = 0; j < src.cols; j++) {
        for (int i = 0; i < src.rows; i++) {
            double cur_val = src.at<double>(i,j);
            if (cur_val > 0.1) {
                top_indices.push_back(i);
                // std::cout << "Top " << i << std::endl;
                top_values.push_back(cur_val);
                break;
            }
        }
    }

    for (int j = 0; j < src.cols; j++) {
        if (0 < top_indices[j] &&  top_indices[j] < src.cols) {
            cv::Mat(top_indices[j]+1,1, CV_64F, cv::Scalar(top_values[j])).copyTo(src.col(j).rowRange(0,top_indices[j]+1));
        } else {
            continue;
        }
    }
    src.convertTo(dst,type);
}

void CustomDilate(cv::Mat src, cv::Mat& dst, const int& kernel_size, KernelType kernel_type) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }
    src = src/1000.0;
    cv::Mat mask;
    GenerateValidMask(src, mask);
    GetInvertDepth(src, src, mask);
    cv::Mat kernel;
    switch (kernel_size) {
        case 3 :
            if (kernel_type == KERNEL_TYPE_CROSS) 
                kernel = kCrossKernel3;
            break;
        case 5 : 
            if (kernel_type == KERNEL_TYPE_CROSS)
                kernel = kCrossKernel5;
            else if (kernel_type == KERNEL_TYPE_DIAMOND)
                kernel = kDiamondKernel5;
            break;
        case 7 :
            if (kernel_type == KERNEL_TYPE_CROSS)
                kernel = kCrossKernel7;
            else if (kernel_type == KERNEL_TYPE_DIAMOND)
                kernel = kDiamondKernel7;
            break;
    }

    cv::dilate(src, src, kernel);
    //DisplayFalseColors(src, "after dilate");

    //cv::morphologyEx(src,src, cv::MORPH_CLOSE, kFullKernel5);
    //DisplayFalseColors(src, "after mop close");

    //cv::Mat tempdilate;
    //cv::dilate(src, tempdilate, kFullKernel7);
    //GenerateInvalidMask(src,mask);
    //cv::Mat temphole;
    // don't change value of the valid pixels
    //tempdilate.copyTo(temphole, mask); 
    //DisplayFalseColors(temphole, "medium hole");
    //src = src + temphole;
    //DisplayFalseColors(src, "after add hole");

    //ExtendTop(src,src);
    //DisplayFalseColors(src, "after extend top");


    // cv::dilate(src, tempdilate, FULL_KERNEL_31);
    // generateInvalidMask(src,mask);
    // cv::Mat bighole;
    // tempdilate.copyTo(bighole, mask);
    // displayFalseColors(bighole, "big hole");
    // src = src + bighole;
    // displayFalseColors(src, "after add big hole");

    // cv::Mat src_;
    // src.convertTo(src_,CV_32F);
    // cv::Mat median;
    // cv::medianBlur(src_,median,5);
    // median.convertTo(median,CV_64F);
    // src = median.clone();

    // generateValidMask(src, mask);
    // cv::Mat blur;
    // cv::GaussianBlur(src,blur,cv::Size(5,5),0);
    // cv::Mat blurvalid;
    // blur.copyTo(blurvalid, mask);
    // src = blurvalid.clone();
    
    GenerateValidMask(src, mask);
    GetInvertDepth(src,dst, mask);
    dst = dst*1000.0;
    dst.convertTo(dst, type);
}


void SimpleDilate(cv::Mat src, cv::Mat& dst, const int& kernel_size, const int& kernel_type) {
    int type = src.type();
    if (type != CV_64F) {
        src.convertTo(src, CV_64F);
    }
    src = src/1000.0;
    cv::Mat mask;
    GenerateValidMask(src, mask);
    GetInvertDepth(src, src,  mask);
    // displayFalseColors(src, "inverse depth");
    cv::Mat kernel;
    switch (kernel_size) {
        case 3 :
            if (kernel_type == KERNEL_TYPE_CROSS) 
                kernel = kCrossKernel3;
            break;
        case 5 : 
            if (kernel_type == KERNEL_TYPE_CROSS)
                kernel = kCrossKernel5;
            else if (kernel_type == KERNEL_TYPE_DIAMOND)
                kernel = kDiamondKernel5;
            break;
        case 7 :
            if (kernel_type == KERNEL_TYPE_CROSS)
                kernel = kCrossKernel7;
            else if (kernel_type == KERNEL_TYPE_DIAMOND)
                kernel = kDiamondKernel7;
            break;
    }

    cv::dilate(src, src ,kernel);
    // displayFalseColors(src, "after dilate");

    cv::morphologyEx(src,src, cv::MORPH_CLOSE, kFullKernel5);
    // displayFalseColors(src, "after mop close");

    cv::Mat tempdilate;
    cv::dilate(src, tempdilate, kFullKernel7);
    GenerateInvalidMask(src,mask);
    cv::Mat temphole;
    tempdilate.copyTo(temphole, mask);


    src = src + temphole;

    
    GenerateValidMask(src, mask);
    GetInvertDepth(src,dst, mask);
    dst = dst*1000.0;
    dst.convertTo(dst, type);
    
}

#endif

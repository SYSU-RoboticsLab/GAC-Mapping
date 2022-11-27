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

# ifndef IPUTIL_H
# define IPUTIL_H

#include <opencv2/opencv.hpp>
#include <string>

#include "log_colormap.h"




//将深度图映射到彩色域中
void generateFalseColors (cv::Mat src, cv::Mat& dst) {

    // color map
    float map[8][4] = {{0,0,0,114},{0,0,1,185},{1,0,0,114},{1,0,1,174},
                       {0,1,0,114},{0,1,1,185},{1,1,0,114},{1,1,1,0}};
    float sum = 0;
    for (int32_t i=0; i<8; i++)
      sum += map[i][3];

    float weights[8]; // relative weights
    float cumsum[8];  // cumulative weights
    cumsum[0] = 0;
    for (int32_t i=0; i<7; i++) {
      weights[i]  = sum/map[i][3];
      cumsum[i+1] = cumsum[i] + map[i][3]/sum;
    }

    
    // create color png image
    cv::Mat image(src.rows,src.cols,CV_8UC3, cv::Scalar(0,0,0));

    double minv = 0.0, maxv = 0.0;  
    double* minp = &minv;  
    double* maxp = &maxv;  

    minMaxIdx(src,minp,maxp);  

    // cout << "Mat minv = " << minv << endl;  
    // cout << "Mat maxv = " << maxv << endl;  


    // for all pixels do
    for (int32_t u=0; u< src.rows; u++) {
      for (int32_t v=0; v< src.cols; v++) {

        // get normalized value
        double val = std::min(std::max(src.at<ushort>(u,v)/maxv,0.0),1.0);

        // find bin
        int32_t i;
        for (i=0; i<7; i++)
          if (val<cumsum[i+1])
            break;

        // compute red/green/blue values
        float   w = 1.0-(val-cumsum[i])*weights[i];
        uint8_t r = (uint8_t)((w*map[i][0]+(1.0-w)*map[i+1][0]) * 255.0);
        uint8_t g = (uint8_t)((w*map[i][1]+(1.0-w)*map[i+1][1]) * 255.0);
        uint8_t b = (uint8_t)((w*map[i][2]+(1.0-w)*map[i+1][2]) * 255.0);

        // set pixel
        image.at<cv::Vec3b>(u,v)[0] = b;
        image.at<cv::Vec3b>(u,v)[1] = g;
        image.at<cv::Vec3b>(u,v)[2] = r;
      }
    }

    dst = image.clone();
}

void generateFalseColors1 (cv::Mat src, cv::Mat& dst, double maxv) {

    // color map
    float map[8][4] = {{0,0,0,114},{0,0,1,185},{1,0,0,114},{1,0,1,174},
                       {0,1,0,114},{0,1,1,185},{1,1,0,114},{1,1,1,0}};
    float sum = 0;
    for (int32_t i=0; i<8; i++)
      sum += map[i][3];

    float weights[8]; // relative weights
    float cumsum[8];  // cumulative weights
    cumsum[0] = 0;
    for (int32_t i=0; i<7; i++) {
      weights[i]  = sum/map[i][3];
      cumsum[i+1] = cumsum[i] + map[i][3]/sum;
    }

    
    // create color png image
    cv::Mat image(src.rows,src.cols,CV_8UC3, cv::Scalar(0,0,0));

    // double minv = 0.0, maxv = 0.0;  
    // double* minp = &minv;  
    // double* maxp = &maxv;  

    // minMaxIdx(src,minp,maxp);  

    // cout << "Mat minv = " << minv << endl;  
    // cout << "Mat maxv = " << maxv << endl;  


    // for all pixels do
    for (int32_t u=0; u< src.rows; u++) {
      for (int32_t v=0; v< src.cols; v++) {

        // get normalized value
        double val = std::min(std::max(src.at<ushort>(u,v)/maxv,0.0),1.0);

        // find bin
        int32_t i;
        for (i=0; i<7; i++)
          if (val<cumsum[i+1])
            break;

        // compute red/green/blue values
        float   w = 1.0-(val-cumsum[i])*weights[i];
        uint8_t r = (uint8_t)((w*map[i][0]+(1.0-w)*map[i+1][0]) * 255.0);
        uint8_t g = (uint8_t)((w*map[i][1]+(1.0-w)*map[i+1][1]) * 255.0);
        uint8_t b = (uint8_t)((w*map[i][2]+(1.0-w)*map[i+1][2]) * 255.0);

        // set pixel
        image.at<cv::Vec3b>(u,v)[0] = b;
        image.at<cv::Vec3b>(u,v)[1] = g;
        image.at<cv::Vec3b>(u,v)[2] = r;
      }
    }

    dst = image.clone();
}

// int tmpcnt = 0;
void displayFalseColors(cv::Mat src, std::string name) {
    if (src.type() != CV_16UC1) {
      src.convertTo(src, CV_16UC1);
    }
    cv::Mat bgr;
    generateFalseColors(src, bgr);
    cv::imshow(name.c_str(), bgr);
    std::stringstream img_name;
    // img_name << "/home/ncslab_slam/Pictures/experiment/testdepth/" <<tmpcnt++<<".png";
    // cv::imwrite(img_name.str(), bgr);
}


# endif
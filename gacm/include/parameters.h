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

#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>

// for camera models
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <pcl/filters/voxel_grid.h>
// for pcl::fromRosMsg
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>

#include <dirent.h>

// camera parameters
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern std::string CAM_NAME;
extern Eigen::Matrix3d RCL;
extern Eigen::Vector3d TCL;
extern Eigen::Matrix4d T_LC;
extern Eigen::Matrix4d T_CL;


// image feature extractor parameters 
extern int MAX_CNT_LSD;
extern int NUM_OCTAVES_LSD;
extern float SCALE_LSD;
extern int WIDTH_OF_BAND_LBD;

extern int MAX_CNT_CORNER;
extern int MIN_DIST_CORNER;

extern double F_THRESHOLD;

extern int WINDOW_SIZE;

extern int DOWN_SAMPLE_NEED;

// laser scan registrator parameters
extern int N_SCANS;
extern int HORIZON_SCANS;
extern float MINIMUM_RANGE;
extern float ANGLE_RES_X;
extern float ANGLE_RES_Y;
extern float ANGLE_BOTTOM;
extern float CROP_NEAR;
extern float CROP_FAR;

extern float EDGE_THRESHOLD;
extern float SURF_THRESHOLD;
extern float NEAREST_FEATURE_DIST;
extern float OCCLUDE_THRESHOLD;

// odom parameters
extern float SCAN_PERIOD;
extern float DISTANCE_SQ_THRESHOLD;
extern float NEARBY_SCAN;
extern int DISTORTION;
extern float VISUAL_WEIGHT_MAX;

// submap parameters
extern int SUBMAP_LENGTH;
extern float KEEP_SQURE_DISTANCE;
// thumbnail para
extern float NDT_RESOLUTION_THUMBNAIL;
extern float NDT_RESOLUTION_MATCH;
extern int NEED_CHECK_DIRECTION;

// output para
extern int NEED_PUB_ODOM;
extern std::string GT_PATH;
extern double GT_START_TIME;
extern std::string OUT_ODOM_PATH;
extern std::string OUT_GT_PATH;
extern int AIR_GPS;

// cache para
extern std::string CACHE_PATH;

// config para
extern int CONFIG_ID;
extern std::string DATA_PATHS[5];

// debug print
extern bool DEBUG;

// display frame selection
extern bool display_frame_cam;

void erase_comment(std::string& str);
void printCopyright();
void readParameters(ros::NodeHandle &n);
// 显示进度，和python里的tqdm效果差不多
void status(int length, float percent); // 第一位是显示长度，第二位是进行的百分数，范围(0,1)
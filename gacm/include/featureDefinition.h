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

#ifndef FEATUREDEFINITION_H
#define FEATUREDEFINITION_H


#include <list>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// #include <manif/manif.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


struct ImagePoint {
     int index;
     int track_cnt;
     int u, v;
     
};

// DW:PCL 中自定义 点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT (ImagePoint,
                                   (int, index, index)
                                   (int, track_cnt, track_cnt)
                                   (int, u, u)
                                   (int, v, v))

struct CameraPoint {
     int index;
     int track_cnt;
     float x;
     float y;
     float z;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (CameraPoint,
                                   (int, index, index)
                                   (int, track_cnt, track_cnt)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)) 

struct DepthPoint {
     float u, v;
     float depth;
     int label;
     int ind;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (DepthPoint,
                                   (float, u, u)
                                   (float, v, v)
                                   (float, depth, depth)
                                   (int, label, label)
                                   (int, ind, ind))





class PointFeaturePerFrame {
     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          PointFeaturePerFrame(const Eigen::Matrix<double, 3, 1> &_point, unsigned char _depth_type, int _frame_id) {
               // point.x() = _point(0);
               // point.y() = _point(1);
               // point.z() = _point(2);
               inverse_depth = _point(0);
               uv.x() = _point(1);
               uv.y() = _point(2);
               depth_type = _depth_type;
               frame_id = _frame_id;
          }

          // Eigen::Vector3d point; // world Frame point cordinate
          double inverse_depth;
          Eigen::Vector2d uv; // observation on each frame
          unsigned char depth_type; // 0: without depth 1: from lidar 2: from triangulate?
          int frame_id;
};

// maintain the depth on the first track frame
class PointFeaturePerId {
     private:
          int start_frame;
          int depth_frame_index;
     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          const int feature_id;

          std::vector<PointFeaturePerFrame,Eigen::aligned_allocator<PointFeaturePerFrame>> feature_per_frame;

          int used_num;

          // Eigen::Vector3d estimated_point; // under world frame
          // double estimated_depth[1] = {-1};
          double inverse_depth[1] = {-1};
          int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
          unsigned char depth_type; // 0: without depth 1: from lidar 2: from triangulate?

          PointFeaturePerId(int _feature_id, int _start_frame)
               : feature_id(_feature_id), start_frame(_start_frame), depth_frame_index(-1), 
                    used_num(0), solve_flag(0), depth_type(0){
          }

          int startFrame() {
               return start_frame;
          }

          int endFrame() {
               // return start_frame + feature_per_frame.size() - 1;
               return feature_per_frame.back().frame_id;
          }

          int depthFrameIndex() {
               return depth_frame_index;
          }

          int depthFrame() {
               return feature_per_frame[depth_frame_index].frame_id;
          }

          int getFrameIdAt(int i) {
               return feature_per_frame[i].frame_id;
          }


          void setDepthType(const unsigned char depth_type_) {
               if (depth_type == 0 && depth_type_ == 1) { // first depth observation
                    depth_type = depth_type_;
                    depth_frame_index = feature_per_frame.size()-1;
                    // estimated_point = feature_per_frame.back().point;
                    inverse_depth[0] = feature_per_frame.back().inverse_depth;

               }
          }

          void useDepthAtIndex(const int& new_idnx) {
               depth_frame_index = new_idnx;
               // estimated_point = feature_per_frame[depth_frame_index].point;
               inverse_depth[0] = feature_per_frame[depth_frame_index].inverse_depth;
          }

          int nextDepthIndex() {
               // case 1: already the last frame
               if (depth_frame_index >= feature_per_frame.size()-1) {
                    return -1;
               }
               int depth_frame_index_tmp = depth_frame_index;
               // case 2:
               while (++depth_frame_index_tmp < feature_per_frame.size()) {
                    if (feature_per_frame[depth_frame_index_tmp].depth_type == 1) {
                         return depth_frame_index_tmp;
                    }
                    // depth_frame_index_tmp++;
               }
               return -1;
          }

          bool isOutlier() {
               // if (estimated_point.hasNaN()) return true;
               // TODO need check?
               // 深度大于100m 或者 小于1.5米
               if (inverse_depth[0] < 0.01 || inverse_depth[0] > 0.67) return true; 
               
               // int next_d_idnx = nextDepthIndex();
               // if (next_d_idnx > 0){
                    
                    // 如果优化后深度值与原深度值相差5米以上认为不可信
                    if (fabs(1.0/inverse_depth[0] - 1.0/feature_per_frame[depth_frame_index].inverse_depth) > 5) {
                         return true;
                    }
                    // if ((estimated_point - feature_per_frame[depth_frame_index].point).norm() > 10) {
                    //      return true;
                    // }
               // }

               return false; 
          }
          

          bool useNextDepth() {

               int new_idnx = nextDepthIndex();
               if (new_idnx<0) return false;
               
               useDepthAtIndex(new_idnx);
               return true;
          }

          
};



class PosePerFrame {
     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          int frame_id;
          // Eigen::Quaterniond q; // x y z w
          // Eigen::Vector3d t;// = {0, 0, 0};
          Eigen::Matrix<double,7,1> pose;
          Eigen::Matrix<double,7,1> rel;
          PosePerFrame() {
               frame_id = -1;
               pose << 0,0,0,1,   0,0,0;
               rel << 0,0,0,1,   0,0,0;
               // t << 0, 0, 0;
               // q = Eigen::Quaterniond::Identity();
          }
          PosePerFrame(int frame_id_, const Eigen::Matrix<double, 7, 1> &pose_, const Eigen::Matrix<double, 7, 1> &rel_) {
               frame_id = frame_id_;
               // t.x() = pose(0);
               // t.y() = pose(1);
               // t.z() = pose(2);

               // q.x() = pose(3);
               // q.y() = pose(4);
               // q.z() = pose(5);
               // q.w() = pose(6);

               pose = pose_;
               rel = rel_;

               // ROS_WARN_STREAM("Add pose to Sequence " << t.transpose() << "\n" << q.matrix());
          } 
};

class MappingDataPerFrame {
     public:
          int frame_id;
          double timestamp;
          cv::Mat rgb_image;
          cv::Mat depth_image;
          std::vector<int> point_feature_ids;
          MappingDataPerFrame(int frame_id_,double timestamp_, const cv::Mat&  rgb_image_, const cv::Mat& depth_image_) : frame_id(frame_id_), timestamp(timestamp_) {
               rgb_image = rgb_image_.clone();
               depth_image = depth_image_.clone();
          } 
};



#endif


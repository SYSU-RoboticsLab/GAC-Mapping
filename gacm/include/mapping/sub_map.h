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

#ifndef SUB_MAP_H
#define SUB_MAP_H

#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/features/boundary.h>
#include <pcl/visualization/pcl_plotter.h>
// #include <pcl/features/rops_estimation.h>
// #include <pcl/features/>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "ga_velodyne/common.h"
#include "lidarFactor.h"
#include "ga_posegraph/poseGraphStructure.h"
#include "parameters.h"

// #include "netvlad_tf_test/CompactImg.h"

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}
class SubMap {
    

    // private:
    public:

        int robot_id; // which session
        int submap_id;  // which submap
        bool is_drone;
        ros::NodeHandle nh;

        int frameCount = 0;
        int access_status = 0; // -1 cached, >=0 active(recently accessed by submap N)

        int laserCloudCenWidth = 52; // 52
        int laserCloudCenHeight = 52;
        int laserCloudCenDepth = 52;
        const int laserCloudWidth = 105; //105
        const int laserCloudHeight = 105;
        const int laserCloudDepth = 105;
        float halfCubeSize = 5.0; //5
        float cubeSize= 10.0; //10
        int halfSurroundCubeNum = 13; //13
        int halfCubeForMapping = 10;


        const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
        int laserCloudSurroundNum = 0;
        int laserCloudValidInd[15625];
        int laserCloudSurroundInd[15625];
        
        // 点云ptr初始化形式参考pose estimator
        // input: from odom
        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; //(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // (new pcl::PointCloud<PointType>());

        // ouput: all visualble cube points
        pcl::PointCloud<PointType>::Ptr laserCloudSurround;//(new pcl::PointCloud<PointType>());

        // surround points in map to build tree
        pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap; //(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap; //(new pcl::PointCloud<PointType>());
        
        //input & output: points in one frame. local --> global
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes; //(new pcl::PointCloud<PointType>());

        // 这里初始化要看一下
        // points in every cube
        std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray;// [laserCloudNum];
        std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray;//[laserCloudNum];

        //kd-tree
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;//(new pcl::KdTreeFLANN<PointType>());
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;//(new pcl::KdTreeFLANN<PointType>());

       


        double stampLatestOdom = 0;
        double timeLaserOdometry = 0;
        double timeLaserOdometryLast = 0;
        // int frame_counter = 0;

        pcl::VoxelGrid<PointType> downSizeFilterCorner;
        pcl::VoxelGrid<PointType> downSizeFilterSurf;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        PointType pointOri, pointSel;

        ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath, pubMatchSurround, pubMatchPose;
        ros::Publisher pubSubmapThumbnail;
        
        nav_msgs::Path laserAfterMappedPath;
        
        std::mutex mBuf;
        std::mutex mCloudGrid;
        
        bool needNewSubMap = false;
        bool alignedToGlobal = false;// first loop arrived?
        bool initWithPose = false;
        bool adjust_finish = false; // shift cube flag
        bool thumbnailGenerated = false;
        
        std::shared_ptr<SubMap> next_submap_ptr;
        // pcl::PointCloud<PointType>::Ptr thumbnailLast;
        pcl::PointCloud<PointType>::Ptr thumbnailCur;  // ground
        // pcl::PointCloud<PointType>::Ptr thumbnailBCur; // building
        Eigen::Vector4f coeffs; // plane norm
    
    // public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        Eigen::Quaterniond q_world_base;//(parameters);
        Eigen::Vector3d t_world_base;//(parameters + 4);
        
        // transformation from base node to current frame
        Eigen::Quaterniond q_base_curr;//(1, 0, 0, 0);
        Eigen::Vector3d t_base_curr;//(0, 0, 0);

        // relative transformation from last frame[received]
        Eigen::Quaterniond q_last_curr;//(1, 0, 0, 0);
        Eigen::Vector3d t_last_curr;//(0, 0, 0);

        // relative transformation from last frame[received]
        Eigen::Quaterniond q_odom_last;//(1, 0, 0, 0);
        Eigen::Vector3d t_odom_last;//(0, 0, 0);

        // transformation from world to current frame
        Eigen::Quaterniond q_world_curr;//(1, 0, 0, 0);
        Eigen::Vector3d t_world_curr;//(0, 0, 0);
        
        // translation from odometry frame to map frame
        Eigen::Quaterniond q_odom_map;
        Eigen::Vector3d t_odom_map;
        
        // 内部维护一段posegraph
        std::vector<PoseNode, Eigen::aligned_allocator<PoseNode>> pose_graph_local; 
        std::vector<MeasurementEdge, Eigen::aligned_allocator<MeasurementEdge>> edges_local;
        
        std::vector<std::pair<int,std::string>> thumbnails_db;
        std::vector<std::pair<int,Eigen::VectorXf>> descriptor_db;

        // pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
        // pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
        
        SubMap(ros::NodeHandle nh_) : nh(nh_)  {
            allocateMemory();
            robot_id = 0;
            submap_id = 0;
            is_drone = false;
        }
        void initParameters(const int& robot_id_, const int& submap_id_,const bool& is_drone_, const double& base_stamp_) {
            robot_id = robot_id_;
            submap_id = submap_id_;
            is_drone = is_drone_;
            access_status = submap_id;
            // base
            PoseNode posenode(base_stamp_, 0, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
            pose_graph_local.push_back(posenode);
        }
        
        // check, is parameter enough?
        SubMap(const int& robot_id_, const int& submap_id_, const bool& is_drone_, const Eigen::Quaterniond& q_world_base_, const Eigen::Vector3d& t_world_base_, 
        const Eigen::Quaterniond& q_odom_last_, const Eigen::Vector3d& t_odom_last_,const double& base_stamp_, ros::NodeHandle nh_) : robot_id(robot_id_), submap_id(submap_id_), is_drone(is_drone_), nh(nh_) {
            allocateMemory();
            access_status = submap_id;
            q_world_base = q_world_base_;
            t_world_base = t_world_base_;
            q_odom_last = q_odom_last_;
            t_odom_last = t_odom_last_;
            stampLatestOdom = base_stamp_;
            timeLaserOdometry = base_stamp_;
            PoseNode posenode(base_stamp_, 0, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
            pose_graph_local.push_back(posenode);
            initWithPose = true;
        }

        SubMap(std::string map_path, const int& robot_id_, const int& submap_id_, const bool& is_drone_, ros::NodeHandle nh_): robot_id(robot_id_), submap_id(submap_id_), is_drone(is_drone_), nh(nh_) {
            //load from file
            allocateMemory();
            access_status = submap_id;
            loadMapFromFile(map_path);
            initWithPose = true;
        }

        // deconstructor
        ~ SubMap() {
            // todo? 
        }
        void setActive(int idx) {
            if (access_status == -1) {
                pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
                std::stringstream ss;
                ss << robot_id*100+submap_id;
                laserCloudCornerArray.resize(laserCloudNum);
                laserCloudSurfArray.resize(laserCloudNum);

                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                    PCL_ERROR ("Couldn't read file corner cloud \n");
                }
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                    PCL_ERROR ("Couldn't read file surf cloud \n");
                }
                if(DEBUG) std::cout << "Load done, start pushing cloud with corner " << laserCloudMapCorner->points.size() <<" and surf " << laserCloudMapSurf->points.size() << "\n";
                for (int i = 0; i < laserCloudMapCorner->points.size(); i++)
                {

                    PointType pointSel = laserCloudMapCorner->points[i];

                    int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                    if (pointSel.x + halfCubeSize < 0)
                        cubeI--;
                    if (pointSel.y + halfCubeSize < 0)
                        cubeJ--;
                    if (pointSel.z + halfCubeSize < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudCornerArray[cubeInd]->push_back(pointSel);
                    }
                }

                for (int i = 0; i < laserCloudMapSurf->points.size(); i++)
                {
                    PointType pointSel = laserCloudMapSurf->points[i];               

                    int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                    if (pointSel.x + halfCubeSize < 0)
                        cubeI--;
                    if (pointSel.y + halfCubeSize < 0)
                        cubeJ--;
                    if (pointSel.z + halfCubeSize < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudSurfArray[cubeInd]->push_back(pointSel);
                    }
                }
                access_status = idx;
            
            }
        }
        void setCache(bool force = false) {
            mCloudGrid.lock(); // prevent set cache and get cloud at the same time
            if (access_status >= 0 || force) {
                pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
                
                for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
                {
                    *laserCloudMapCorner += *laserCloudCornerArray[i];
                    *laserCloudMapSurf += *laserCloudSurfArray[i];
                }
                std::stringstream ss;
                ss << robot_id*100+submap_id;
                pcl::io::savePCDFileASCII(CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner);
                pcl::io::savePCDFileASCII(CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf);
                
                access_status = -1;
                
                for (auto voxel : laserCloudCornerArray) {
                    voxel.reset(new pcl::PointCloud<PointType>());
                }
                for (auto voxel : laserCloudSurfArray) {
                    voxel.reset(new pcl::PointCloud<PointType>());
                }
                laserCloudCornerArray.resize(0);
                laserCloudSurfArray.resize(0);
            }
            mCloudGrid.unlock();
        }
        void saveMapToFile(std::string map_path) {
            // save to file
            std::stringstream ss;
            ss << robot_id*100+submap_id;
            std::string parameters_file = "paras"+ss.str()+".cfg";
            if(DEBUG) std::cout << "\nSaving map to " << parameters_file << "\n";
            std::fstream fs;
            fs.open(map_path+parameters_file, std::ios::out);
            fs << q_world_base.w() << ' ' << q_world_base.w() << ' ' << q_world_base.x()<< ' ' << q_world_base.y()<< ' ' << q_world_base.z() << "\n";
            fs << t_world_base.x() << ' ' << t_world_base.y() << ' ' << t_world_base.z() << "\n";
            fs << q_base_curr.w() << ' ' << q_base_curr.w() << ' ' << q_base_curr.x()<< ' ' << q_base_curr.y()<< ' ' << q_base_curr.z() << "\n";
            fs << t_base_curr.x() << ' ' << t_base_curr.y() << ' ' << t_base_curr.z() << "\n";
            fs << q_last_curr.w() << ' ' << q_last_curr.w() << ' ' << q_last_curr.x()<< ' ' << q_last_curr.y()<< ' ' << q_last_curr.z() << "\n";
            fs << t_last_curr.x() << ' ' << t_last_curr.y() << ' ' << t_last_curr.z() << "\n";
            fs << q_odom_last.w() << ' ' << q_odom_last.w() << ' ' << q_odom_last.x()<< ' ' << q_odom_last.y()<< ' ' << q_odom_last.z() << "\n";
            fs << t_odom_last.x() << ' ' << t_odom_last.y() << ' ' << t_odom_last.z() << "\n";
            fs << q_world_curr.w() << ' ' << q_world_curr.w() << ' ' << q_world_curr.x()<< ' ' << q_world_curr.y()<< ' ' << q_world_curr.z() << "\n";
            fs << t_world_curr.x() << ' ' << t_world_curr.y() << ' ' << t_world_curr.z() << "\n";

            fs << pose_graph_local.size()  << "\n";
            for (auto node: pose_graph_local) {
                fs << std::fixed << node.stamp << " " << node.frame_id << " " 
                    << node.q.w() << " " << node.q.x() << " " << node.q.y() << " "<< node.q.z() << " " 
                    << node.t.x() << " " << node.t.y() << " " << node.t.z() << "\n";        
            }
            fs << edges_local.size() << "\n";
            for (auto edge: edges_local) {
                fs << std::fixed << edge.stamp_from << " " << edge.stamp_to << " "  
                    << edge.robot_from << " " << edge.robot_to << " "
                    << edge.submap_from << " " << edge.submap_to << " "
                    << edge.index_from << " " << edge.index_to << " "
                    << edge.q.w() << " " << edge.q.x() << " " << edge.q.y() << " "<< edge.q.z() << " " 
                    << edge.t.x() << " " << edge.t.y() << " " << edge.t.z() << "\n";        
            }

            fs << coeffs(0) << " " << coeffs(1) << " " << coeffs(2) << " " << coeffs(3) << "\n";
            
            

            fs.close();


            pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
            
            if (access_status == -1) {
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                    PCL_ERROR ("Couldn't read file corner cloud \n");
                }
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                    PCL_ERROR ("Couldn't read file surf cloud \n");
                }
            } else {
                for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
                {
                    *laserCloudMapCorner += *laserCloudCornerArray[i];
                    *laserCloudMapSurf += *laserCloudSurfArray[i];
                }
            }

            pcl::io::savePCDFileASCII(map_path+ss.str()+"corner.pcd", *laserCloudMapCorner);
            pcl::io::savePCDFileASCII(map_path+ss.str()+"surf.pcd", *laserCloudMapSurf);
            if (thumbnailCur->size() > 0) {
                pcl::io::savePCDFileASCII(map_path+ss.str()+"thumbnail.pcd", *thumbnailCur);

            }


        }
        
        void loadMapFromFile(std::string map_path) {
            std::stringstream ss;
            ss << robot_id*100+submap_id;
            std::string parameters_file = "paras"+ss.str()+".cfg";
            std::cout << "\nLoading map from " << parameters_file << "\n";
            std::fstream fs;
            fs.open(map_path+parameters_file, std::ios::in);
            // fs.open(map_path+parameters_file, std::ios::out);
            fs >> q_world_base.w() >> q_world_base.w() >> q_world_base.x() >> q_world_base.y() >> q_world_base.z();
            fs >> t_world_base.x() >> t_world_base.y() >> t_world_base.z();
            fs >> q_base_curr.w() >> q_base_curr.w() >> q_base_curr.x() >> q_base_curr.y() >> q_base_curr.z();
            fs >> t_base_curr.x() >> t_base_curr.y() >> t_base_curr.z();
            fs >> q_last_curr.w() >> q_last_curr.w() >> q_last_curr.x() >> q_last_curr.y() >> q_last_curr.z();
            fs >> t_last_curr.x() >> t_last_curr.y() >> t_last_curr.z();
            fs >> q_odom_last.w() >> q_odom_last.w() >> q_odom_last.x() >> q_odom_last.y() >> q_odom_last.z();
            fs >> t_odom_last.x() >> t_odom_last.y() >> t_odom_last.z();
            fs >> q_world_curr.w() >> q_world_curr.w() >> q_world_curr.x() >> q_world_curr.y() >> q_world_curr.z();
            fs >> t_world_curr.x() >> t_world_curr.y() >> t_world_curr.z();

            int pose_graph_local_size;
            fs >> pose_graph_local_size;
            if(DEBUG) std::cout << " local pose graph size " << pose_graph_local_size << "\n";
            pose_graph_local.clear();
            for (int i = 0; i < pose_graph_local_size; i++) {
                PoseNode node;
                fs >> node.stamp >> node.frame_id >> node.q.w() >> node.q.x() >> node.q.y() >> node.q.z()
                   >> node.t.x() >> node.t.y() >> node.t.z();
                pose_graph_local.push_back(node);
            }

            int edges_local_size;
            fs >> edges_local_size;
            edges_local.clear();
            if(DEBUG) std::cout << " local edges size " << edges_local_size << "\n";
            for (int i=0; i < edges_local_size; i++) {
                MeasurementEdge edge;
                fs >> edge.stamp_from >> edge.stamp_to 
                    >> edge.robot_from >> edge.robot_to
                    >> edge.submap_from >> edge.submap_to
                    >> edge.index_from >> edge.index_to
                    >> edge.q.w() >> edge.q.x() >> edge.q.y() >>edge.q.z()
                    >> edge.t.x() >>edge.t.y() >> edge.t.z();  
                edges_local.push_back(edge);
            }

            fs >> coeffs(0) >> coeffs(1) >> coeffs(2) >>coeffs(3);
            fs.close();

            pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());

            if (pcl::io::loadPCDFile<PointType> (map_path+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                PCL_ERROR ("Couldn't read file corner cloud \n");
            }
            if (pcl::io::loadPCDFile<PointType> (map_path+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                PCL_ERROR ("Couldn't read file surf cloud \n");
            }
            bool can_generate_thumbnail = false;
            if (pcl::io::loadPCDFile<PointType> (map_path+ss.str()+"thumbnail.pcd", *thumbnailCur) == -1) {
                PCL_ERROR ("Couldn't read file thumbnail cloud \n");
            } else {
                can_generate_thumbnail = true;
            }
                
            if(DEBUG) std::cout << "Load done, start pushing cloud with corner " << laserCloudMapCorner->points.size() <<" and surf " << laserCloudMapSurf->points.size() << "\n";
            for (int i = 0; i < laserCloudMapCorner->points.size(); i++)
            {

                PointType pointSel = laserCloudMapCorner->points[i];

                int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                if (pointSel.x + halfCubeSize < 0)
                    cubeI--;
                if (pointSel.y + halfCubeSize < 0)
                    cubeJ--;
                if (pointSel.z + halfCubeSize < 0)
                    cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                    cubeJ >= 0 && cubeJ < laserCloudHeight &&
                    cubeK >= 0 && cubeK < laserCloudDepth)
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                   laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
            }

            for (int i = 0; i < laserCloudMapSurf->points.size(); i++)
            {
                PointType pointSel = laserCloudMapSurf->points[i];               

                int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                if (pointSel.x + halfCubeSize < 0)
                    cubeI--;
                if (pointSel.y + halfCubeSize < 0)
                    cubeJ--;
                if (pointSel.z + halfCubeSize < 0)
                    cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                    cubeJ >= 0 && cubeJ < laserCloudHeight &&
                    cubeK >= 0 && cubeK < laserCloudDepth)
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
            }
            
            if (can_generate_thumbnail) {
                generateThumbnail(true);
            }
            setCache(true);
        }

        void allocateMemory() {
            //todo
            // laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
            // laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
            
            laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());

            // ouput: all visualble cube points
            laserCloudSurround.reset(new pcl::PointCloud<PointType>());

            // surround points in map to build tree
            laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
            
            //input & output: points in one frame. local --> global
            laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

            //kd-tree
            kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
            kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());


            //last thumbnail
            // thumbnailLast.reset(new pcl::PointCloud<PointType>()); //initialize when create new submap
            //cur thumbnail
            thumbnailCur.reset(new pcl::PointCloud<PointType>());
            // thumbnailBCur.reset(new pcl::PointCloud<PointType>());

			laserCloudCornerArray.resize(laserCloudNum);
            laserCloudSurfArray.resize(laserCloudNum);
            for (int i = 0; i < laserCloudNum; i++)
			{
				laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
				laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
			}
            

            Eigen::Vector3d q_v(0,0,0);
            q_world_base.vec() = q_v;
            q_world_base.w() = 1;
            t_world_base << 0, 0, 0;

            q_base_curr.vec() = q_v;
            q_base_curr.w() = 1;
            t_base_curr << 0, 0, 0;

            q_last_curr.vec() = q_v;
            q_last_curr.w() = 1;
            t_last_curr << 0, 0, 0;

            q_odom_last.vec() = q_v;
            q_odom_last.w() = 1;
            t_odom_last << 0, 0, 0;

            q_world_curr.vec() = q_v;
            q_world_curr.w() = 1;
            t_world_curr << 0, 0, 0;

            q_odom_map.vec() = q_v;
            q_odom_map.w() = 1;
            t_odom_map << 0, 0, 0;
            
            pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
            pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_submap", 100);
            pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
            pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
            pubSubmapThumbnail = nh.advertise<sensor_msgs::PointCloud2>("/thumbnail_submap",100);
            

            float lineRes = 0.2; // 0.1
	        float planeRes = 0.4; //0.2
            downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
	        downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);
        }

        
        std::shared_ptr<SubMap> createNewSubMap() {
            
            needNewSubMap = false;
            next_submap_ptr = std::shared_ptr<SubMap>(new SubMap(robot_id, submap_id+1, is_drone, q_world_curr, t_world_curr,q_odom_last, t_odom_last, timeLaserOdometry, nh));
            
            
            pcl::PointCloud<PointType>::Ptr cornerCloudSurround(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surfCloudSurround(new pcl::PointCloud<PointType>());
            // laserCloudSurround->clear();
            for (int i = 0; i < laserCloudSurroundNum; i++)
            {
                int ind = laserCloudSurroundInd[i];
                *cornerCloudSurround += *laserCloudCornerArray[ind];
                *surfCloudSurround += *laserCloudSurfArray[ind];
            }

            for (int i = 0; i < cornerCloudSurround->size(); i++)
            {

                PointType pointSelShift;
                PointType pointSel;
                pointInvTransform(&(cornerCloudSurround->points[i]), &pointSel, q_base_curr, t_base_curr);
                // next_submap_ptr->pointAssociateToMap(&pointSelShift, &pointSel);
                if (pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z > KEEP_SQURE_DISTANCE) {
                    continue;
                }


                int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                if (pointSel.x + halfCubeSize < 0)
                    cubeI--;
                if (pointSel.y + halfCubeSize < 0)
                    cubeJ--;
                if (pointSel.z + halfCubeSize < 0)
                    cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                    cubeJ >= 0 && cubeJ < laserCloudHeight &&
                    cubeK >= 0 && cubeK < laserCloudDepth)
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    next_submap_ptr->laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
            }

            for (int i = 0; i < surfCloudSurround->size(); i++)
            {

                PointType pointSelShift;
                PointType pointSel;
                pointInvTransform(&(surfCloudSurround->points[i]), &pointSel,q_base_curr, t_base_curr);
                // next_submap_ptr->pointAssociateToMap(&pointSelShift, &pointSel);
                if (pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z > KEEP_SQURE_DISTANCE) {
                    continue;
                }

                int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                if (pointSel.x + halfCubeSize < 0)
                    cubeI--;
                if (pointSel.y + halfCubeSize < 0)
                    cubeJ--;
                if (pointSel.z + halfCubeSize < 0)
                    cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                    cubeJ >= 0 && cubeJ < laserCloudHeight &&
                    cubeK >= 0 && cubeK < laserCloudDepth)
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    next_submap_ptr->laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
            }
            
            cornerCloudSurround->clear();
            surfCloudSurround->clear();
            
            // should remove the last node? 
            return next_submap_ptr;
        }

        void pointInvTransform(PointType *pi, PointType *po, Eigen::Quaterniond& q, Eigen::Vector3d &t) {
            Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
            Eigen::Vector3d point_new = q.inverse() * (point_curr - t);
            po->x = point_new.x();
            po->y = point_new.y();
            po->z = point_new.z();
            po->intensity = pi->intensity;
        }
        void pointAssociateToMap(PointType const *const pi, PointType *const po)
        {
            Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
            Eigen::Vector3d point_w = q_base_curr * point_curr + t_base_curr;
            po->x = point_w.x();
            po->y = point_w.y();
            po->z = point_w.z();
            po->intensity = pi->intensity;
            //po->intensity = 1.0;
        }

        void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
        {
            Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
            Eigen::Vector3d point_curr = q_base_curr.inverse() * (point_w - t_base_curr);
            po->x = point_curr.x();
            po->y = point_curr.y();
            po->z = point_curr.z();
            po->intensity = pi->intensity;
        }

        bool checkNeedNewSubMap() {
            // return false; //  skip map spilt
            return needNewSubMap;
        }

        void process(sensor_msgs::PointCloud2ConstPtr cornerLastBufFrontPtr,
                    sensor_msgs::PointCloud2ConstPtr surfLastBufFrontPtr,
                    sensor_msgs::PointCloud2ConstPtr fullResBufFrontPtr,
                    nav_msgs::Odometry::ConstPtr odometryBufFrontPtr) {

            // m_Buf.lock();
            timeLaserOdometryLast = timeLaserOdometry;
            timeLaserOdometry = odometryBufFrontPtr->header.stamp.toSec();
            // ROS_ERROR_STREAM("process begin " << timeLaserOdometry-timeLaserOdometryLast);
            if (timeLaserOdometry-timeLaserOdometryLast <= 0) {
                timeLaserOdometry = timeLaserOdometryLast;
            }

            laserCloudCornerLast->clear();
            // ROS_ERROR_STREAM("clear");
            pcl::fromROSMsg(*cornerLastBufFrontPtr, *laserCloudCornerLast);
            // ROS_ERROR_STREAM("from ros");
            // cornerLastBuf.pop();

            laserCloudSurfLast->clear();
            pcl::fromROSMsg(*surfLastBufFrontPtr, *laserCloudSurfLast);
            // surfLastBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullResBufFrontPtr, *laserCloudFullRes);
            // fullResBuf.pop();

            Eigen::Quaterniond q_odom_curr;
            Eigen::Vector3d t_odom_curr;
            q_odom_curr.x() =  odometryBufFrontPtr->pose.pose.orientation.x;
            q_odom_curr.y() =  odometryBufFrontPtr->pose.pose.orientation.y;
            q_odom_curr.z() =  odometryBufFrontPtr->pose.pose.orientation.z;
            q_odom_curr.w() =  odometryBufFrontPtr->pose.pose.orientation.w;
            t_odom_curr.x() =  odometryBufFrontPtr->pose.pose.position.x;
            t_odom_curr.y() =  odometryBufFrontPtr->pose.pose.position.y;
            t_odom_curr.z() =  odometryBufFrontPtr->pose.pose.position.z;
            
            if (timeLaserOdometryLast == 0 && initWithPose == false) {
                q_world_base = q_odom_curr;
                t_world_base = t_odom_curr;
                q_odom_last = q_odom_curr;
                t_odom_last = t_odom_curr;
                initWithPose = true;
                return;
            }

            
            // q_last_curr = q_odom_last.inverse()*q_odom_curr;
            // q_last_curr.normalize();
            // t_last_curr = q_odom_last.inverse()*(t_odom_curr-t_odom_last);

            // q_odom_last = q_odom_curr;
            // t_odom_last = t_odom_curr;
            // odometryBuf.pop();
            //  ROS_ERROR_STREAM("get topics finish");
            
            if (pose_graph_local.empty()) {
                // should have init at constructor?
                if(DEBUG) ROS_ERROR_STREAM("pose_graph_local empty!");
            } else {

            }

            // get last optimized
            Eigen::Quaterniond q_base_last = pose_graph_local.back().q;
            Eigen::Vector3d t_base_last = pose_graph_local.back().t;

            q_odom_map = q_base_last*q_odom_last.inverse();
            t_odom_map = t_base_last - q_odom_map * t_odom_last;

            // calculate latest estimate
            q_base_curr = q_odom_map * q_odom_curr;
            t_base_curr = q_odom_map * t_odom_curr + t_odom_map;

            q_odom_last = q_odom_curr;
            t_odom_last = t_odom_curr;

            // generate new node base on optimized pose

            PoseNode pose_node(timeLaserOdometry, pose_graph_local.size(), q_base_curr, t_base_curr);
            // add new node to pose graph
            pose_graph_local.push_back(pose_node);

            if (frameCount % 5 == 0) {
                int centerCubeI = int((t_base_curr.x() + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                int centerCubeJ = int((t_base_curr.y() + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                int centerCubeK = int((t_base_curr.z() + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                if (t_base_curr.x() + halfCubeSize < 0)
                    centerCubeI--;
                if (t_base_curr.y() + halfCubeSize < 0)
                    centerCubeJ--;
                if (t_base_curr.z() + halfCubeSize < 0)
                    centerCubeK--;

                // start shift cube
                adjust_finish = false;
                while (centerCubeI < halfSurroundCubeNum)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        { 
                            int i = laserCloudWidth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i >= 1; i--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeI++;
                    laserCloudCenWidth++;
                }

                while (centerCubeI >= laserCloudWidth - halfSurroundCubeNum)
                { 
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i < laserCloudWidth - 1; i++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeI--;
                    laserCloudCenWidth--;
                }

                while (centerCubeJ < halfSurroundCubeNum)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = laserCloudHeight - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j >= 1; j--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeJ++;
                    laserCloudCenHeight++;
                }

                while (centerCubeJ >= laserCloudHeight - halfSurroundCubeNum)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j < laserCloudHeight - 1; j++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeJ--;
                    laserCloudCenHeight--;
                }

                while (centerCubeK < halfSurroundCubeNum)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = laserCloudDepth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k >= 1; k--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeK++;
                    laserCloudCenDepth++;
                }

                while (centerCubeK >= laserCloudDepth - halfSurroundCubeNum)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k < laserCloudDepth - 1; k++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                            laserCloudCubeSurfPointer->clear();//.reset(new pcl::PointCloud<PointType>());
                        }
                    }

                    centerCubeK--;
                    laserCloudCenDepth--;
                }
                adjust_finish = true;

                // ROS_ERROR_STREAM("Adjust finish");



                int laserCloudValidNum = 0;
                laserCloudSurroundNum = 0;

                // fetch points for mapping 
                for (int i = centerCubeI - halfCubeForMapping; i <= centerCubeI + halfCubeForMapping; i++)
                {
                    for (int j = centerCubeJ - halfCubeForMapping; j <= centerCubeJ + halfCubeForMapping; j++)
                    {
                        for (int k = centerCubeK - halfCubeForMapping; k <= centerCubeK + halfCubeForMapping; k++)
                        {
                            if (i >= 0 && i < laserCloudWidth &&
                                j >= 0 && j < laserCloudHeight &&
                                k >= 0 && k < laserCloudDepth)
                            { 
                                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                                laserCloudValidNum++;
                                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                                laserCloudSurroundNum++;
                            }
                        }
                    }
                }

                laserCloudCornerFromMap->clear();//.reset(new pcl::PointCloud<PointType>());
                laserCloudSurfFromMap->clear();//.reset(new pcl::PointCloud<PointType>());
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
                }
                // *laserCloudSurfFromMap += *thumbnailLast;

                int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
                int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();


                pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
                downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
                downSizeFilterCorner.filter(*laserCloudCornerStack);
                int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

                pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
                downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
                downSizeFilterSurf.filter(*laserCloudSurfStack);
                int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

                if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
                {
                    
                    clock_t start, end;
                    start = clock();

                    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
                    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

                    for (int iterCount = 0; iterCount < 2; iterCount++)
                    {
                        //ceres::LossFunction *loss_function = NULL;
                        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                        ceres::LocalParameterization *q_parameterization =
                            new ceres::EigenQuaternionParameterization();
                        ceres::Problem::Options problem_options;

                        ceres::Problem problem(problem_options);
                        problem.AddParameterBlock(pose_graph_local.back().q.coeffs().data(), 4, q_parameterization);
                        problem.AddParameterBlock(pose_graph_local.back().t.data(), 3);

                        int corner_num = 0;

                        for (int i = 0; i < laserCloudCornerStackNum; i++)
                        {
                            pointOri = laserCloudCornerStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

                            if (pointSearchSqDis[4] < 1.0)
                            { 
                                std::vector<Eigen::Vector3d> nearCorners;
                                Eigen::Vector3d center(0, 0, 0);
                                for (int j = 0; j < 5; j++)
                                {
                                    Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                        laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                        laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                                    center = center + tmp;
                                    nearCorners.push_back(tmp);
                                }
                                center = center / 5.0;

                                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                                for (int j = 0; j < 5; j++)
                                {
                                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                                }

                                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                                { 
                                    Eigen::Vector3d point_on_line = center;
                                    Eigen::Vector3d point_a, point_b;
                                    point_a = 0.1 * unit_direction + point_on_line;
                                    point_b = -0.1 * unit_direction + point_on_line;

                                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                                    problem.AddResidualBlock(cost_function, loss_function, pose_graph_local.back().q.coeffs().data(), pose_graph_local.back().t.data());
                                    corner_num++;	
                                }							
                            }
                        }

                        int surf_num = 0;
                        for (int i = 0; i < laserCloudSurfStackNum; i++)
                        {
                            pointOri = laserCloudSurfStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            Eigen::Matrix<double, 5, 3> matA0;
                            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                            if (pointSearchSqDis[4] < 1.0)
                            {
                                
                                for (int j = 0; j < 5; j++)
                                {
                                    matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                                    matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                                    matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                                }
                                // find the norm of plane
                                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                                double negative_OA_dot_norm = 1 / norm.norm();
                                norm.normalize();

                                // Here n(pa, pb, pc) is unit norm of plane
                                bool planeValid = true;
                                for (int j = 0; j < 5; j++)
                                {
                                    // if OX * n > 0.2, then plane is not fit well
                                    if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                            norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                            norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                                    {
                                        planeValid = false;
                                        break;
                                    }
                                }
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                if (planeValid)
                                {
                                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                                    problem.AddResidualBlock(cost_function, loss_function, pose_graph_local.back().q.coeffs().data(), pose_graph_local.back().t.data());
                                    surf_num++;
                                }
                            }
                        }

                       
                        ceres::Solver::Options options;
                        options.linear_solver_type = ceres::DENSE_QR;
                        options.max_num_iterations = 4;
                        options.minimizer_progress_to_stdout = false;
                        options.check_gradients = false;
                        options.gradient_check_relative_precision = 1e-4;
                        ceres::Solver::Summary summary;
                        ceres::Solve(options, &problem, &summary);
                        
                    }
                    end = clock();
                    ofstream outfile;
                    outfile.open(
                        (   std::string(std::getenv("HOME")) + 
                            std::string("/gacm_output/timecost/scan_to_map_time.txt")).c_str(), 
                    ios::app);
                    outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
                    outfile.close();
                } else {
                    ROS_WARN_STREAM("Map corner and surf num are not enough");
                }

                
                // add the current scan to map
                for (int i = 0; i < laserCloudCornerStackNum; i++)
                {

                    float orix = laserCloudCornerStack->points[i].x;
                    float oriy = laserCloudCornerStack->points[i].y;
                    float oriz = laserCloudCornerStack->points[i].z;

                    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
                    pointSel.intensity = 255;

                    int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                    if (pointSel.x + halfCubeSize < 0)
                        cubeI--;
                    if (pointSel.y + halfCubeSize < 0)
                        cubeJ--;
                    if (pointSel.z + halfCubeSize < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudCornerArray[cubeInd]->points.push_back(pointSel);
                    }
                }

                for (int i = 0; i < laserCloudSurfStackNum; i++)
                {

                    float orix = laserCloudSurfStack->points[i].x;
                    float oriy = laserCloudSurfStack->points[i].y;
                    float oriz = laserCloudSurfStack->points[i].z;

                    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
                    pointSel.intensity = 1;

                    int cubeI = int((pointSel.x + halfCubeSize) / cubeSize) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + halfCubeSize) / cubeSize) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + halfCubeSize) / cubeSize) + laserCloudCenDepth;

                    if (pointSel.x + halfCubeSize < 0)
                        cubeI--;
                    if (pointSel.y + halfCubeSize < 0)
                        cubeJ--;
                    if (pointSel.z + halfCubeSize < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudSurfArray[cubeInd]->points.push_back(pointSel);
                    }
                }

                
                // downsample the map
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    int ind = laserCloudValidInd[i];

                    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
                    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                    downSizeFilterCorner.filter(*tmpCorner);
                    laserCloudCornerArray[ind] = tmpCorner;

                    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
                    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                    downSizeFilterSurf.filter(*tmpSurf);
                    laserCloudSurfArray[ind] = tmpSurf;
                }
            } // end framecount mod 5

            q_base_curr = pose_graph_local.back().q;
            q_base_curr.normalize();
            t_base_curr = pose_graph_local.back().t;

            t_world_curr = t_world_base + q_world_base*t_base_curr;
            q_world_curr = q_world_base * q_base_curr;
            q_world_curr.normalize();
            
            
            // update relative pose measurement
            Eigen::Quaterniond q_last_curr_opt = q_base_last.inverse()* q_base_curr;
            q_last_curr_opt.normalize();
            Eigen::Vector3d t_last_curr_opt = q_base_last.inverse()*(t_base_curr-t_base_last);
            

            
            // add measurement edge
            int index = pose_graph_local.size()-1;
            MeasurementEdge measurement_edge(timeLaserOdometryLast, timeLaserOdometry, robot_id,robot_id, submap_id, submap_id, index-1, index,q_last_curr_opt, t_last_curr_opt);
            edges_local.push_back(measurement_edge);

            


            
            // TicToc t_pub;
            //publish surround map for every 5 frame
            if (frameCount % 5 == 0) {
                // publishSurroundMap(laserCloudSurroundNum);
                publishMap();
               
            }


            // real time visualization
            int laserCloudFullResNum = laserCloudFullRes->points.size();
            for (int i = 0; i < laserCloudFullResNum; i++)
            {
                pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
            }

            // publish full res point cloud
            std::stringstream ss;
            ss << (robot_id*100+submap_id);
            std::string s;
            ss >> s;
            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudFullRes3.header.frame_id = "/submap" +s;
            pubLaserCloudFullRes.publish(laserCloudFullRes3); // useless

            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "/camera";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);

            Eigen::Quaterniond q_tmp = q_world_curr;
            Eigen::Vector3d t_tmp = t_world_curr;
            if(!display_frame_cam)
            {
                Eigen::Affine3d T_Cworld_robot = Eigen::Translation3d(t_tmp)*q_tmp.toRotationMatrix();
                Eigen::Affine3d T_Lworld_robot = Eigen::Affine3d(T_LC) * T_Cworld_robot;
                q_tmp = T_Lworld_robot.rotation();
                t_tmp = T_Lworld_robot.translation();
            }

            odomAftMapped.pose.pose.orientation.x = q_tmp.x();
            odomAftMapped.pose.pose.orientation.y = q_tmp.y();
            odomAftMapped.pose.pose.orientation.z = q_tmp.z();
            odomAftMapped.pose.pose.orientation.w = q_tmp.w();
            odomAftMapped.pose.pose.position.x = t_tmp.x();
            odomAftMapped.pose.pose.position.y = t_tmp.y();
            odomAftMapped.pose.pose.position.z = t_tmp.z();
            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;

            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(tf::Vector3(t_world_curr(0),
                                            t_world_curr(1),
                                            t_world_curr(2)));
            q.setW(q_world_curr.w());
            q.setX(q_world_curr.x());
            q.setY(q_world_curr.y());
            q.setZ(q_world_curr.z());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera", "/aft_mapped"));

            frameCount++;
        
            // if (t_base_curr.x()*t_base_curr.x() + t_base_curr.y()*t_base_curr.y() + t_base_curr.z()*t_base_curr.z() > 900) {
            if (frameCount % SUBMAP_LENGTH*5 == 0 && (t_base_curr.x()*t_base_curr.x() + t_base_curr.y()*t_base_curr.y() + t_base_curr.z()*t_base_curr.z() > 25)) {
                if(DEBUG) ROS_WARN_STREAM("++++++++++++++++++++++++++++++++" << t_base_curr.transpose() <<"\n\n\n\n\n\n\n");
                needNewSubMap = true;
            }

            // clear rubbish memory
            laserCloudCornerFromMap->clear();//.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfFromMap->clear();//.reset(new pcl::PointCloud<PointType>());
        } // end process

        void publishMap(const int& laserCloudSurroundNum = -1) {
            static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;
			transform.setOrigin(tf::Vector3(t_world_base(0),
											t_world_base(1),
											t_world_base(2)));
			q.setW(q_world_base.w());
			q.setX(q_world_base.x());
			q.setY(q_world_base.y());
			q.setZ(q_world_base.z());
			transform.setRotation(q);
            std::stringstream ss;
            ss << (robot_id*100+submap_id);
            std::string s;
            ss >> s;
			br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(timeLaserOdometry), "/camera", "/submap"+s));

            pcl::PointCloud<PointType> laserCloudMap;
            if (laserCloudSurroundNum == -1) {
                if (access_status == -1) {
                    pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
                    std::stringstream ss;
                    ss << submap_id;
                    if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                        PCL_ERROR ("Couldn't read file corner cloud \n");
                    }
                    if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                        PCL_ERROR ("Couldn't read file surf cloud \n");
                    }
                    laserCloudMap += *laserCloudMapCorner;
                    laserCloudMap += *laserCloudMapSurf;
                    laserCloudMapCorner -> clear();
                    laserCloudMapSurf -> clear();
                } else {
                    for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
                    {
                        laserCloudMap += *laserCloudCornerArray[i];
                        laserCloudMap += *laserCloudSurfArray[i];
                    }
                }
            } else if (laserCloudSurroundNum == 1){
                // pub surround
                for (int i = 0; i < laserCloudSurroundNum; i++)
                {
                    int ind = laserCloudSurroundInd[i];
                    laserCloudMap += *laserCloudCornerArray[ind];
                    laserCloudMap += *laserCloudSurfArray[ind];
                }
            } else {
                // don't pub
                return;
            }
            sensor_msgs::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudMsg.header.frame_id = "/submap"+s;
            // ROS_WARN_STREAM("Publish submap " << s << " at " <<std::setprecision(20) << timeLaserOdometry);
            pubLaserCloudMap.publish(laserCloudMsg);
        }

        pcl::PointCloud<PointType>::Ptr getMapCloud(bool in_world_frame = false, bool get_surround = false) {
            mCloudGrid.lock();
            // return thumbnailBCur;
            pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>());
            // std::cout <<"\naccess status " << access_status << "\n";
            if (access_status == -1) {
                pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
                std::stringstream ss;
                ss << robot_id*100+submap_id;
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                    PCL_ERROR ("Couldn't read file corner cloud \n");
                }
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                    PCL_ERROR ("Couldn't read file surf cloud \n");
                }
                *laserCloudMap += *laserCloudMapCorner;
                *laserCloudMap += *laserCloudMapSurf;
                
            } else {
                for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
                {
                    *laserCloudMap += *laserCloudCornerArray[i];
                    *laserCloudMap += *laserCloudSurfArray[i];
                }
            }

            if (get_surround) {
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                for (int i=0; i < laserCloudMap->size(); i++) {
                    Eigen::Vector3d temp;
                    temp << laserCloudMap->points[i].x, laserCloudMap->points[i].y, laserCloudMap->points[i].z;
                    if (temp.norm() < 60) {
                        inliers->indices.push_back(i);
                    }
                }
                pcl::ExtractIndices<PointType> extract;
                extract.setInputCloud (laserCloudMap);    //input cloud
                extract.setIndices (inliers);     
                extract.setNegative (false);     
                extract.filter (*laserCloudMap);
            }

            if (in_world_frame) {
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                transform.block(0,0,3,3) = q_world_base.toRotationMatrix().cast<float>();
                transform.block(0,3,3,1) = t_world_base.cast<float>();
            
                // std::cout << " get map size " << laserCloudMap->size() << "\n";
                pcl::transformPointCloud(*laserCloudMap, *laserCloudMap, transform);
            }
            mCloudGrid.unlock();
            return laserCloudMap;
        }

        

        void publishSurroundMap(const int& laserCloudSurroundNum) {
            
            laserCloudSurround->clear();//.reset(new pcl::PointCloud<PointType>());
            
            for (int i = 0; i < laserCloudSurroundNum; i++)
            {
                int ind = laserCloudSurroundInd[i];
                *laserCloudSurround += *laserCloudCornerArray[ind];
                *laserCloudSurround += *laserCloudSurfArray[ind];
            }
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            pcl::PointCloud<PointType>  laserCloudOut;
            pcl::transformPointCloud(*laserCloudSurround, laserCloudOut, transform);

            sensor_msgs::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(laserCloudOut, laserCloudSurround3);
            laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudSurround3.header.frame_id = "/camera";
            pubLaserCloudSurround.publish(laserCloudSurround3);
            // laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        }


        

        void displayCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::string& window_name, const std::string& id) {
            if (cloud->size() < 1)
            {
                std::cout << window_name << " display failure. Cloud contains no points\n";
                return;
            }

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud, "intensity");

            viewer->addPointCloud< pcl::PointXYZI >(cloud, point_cloud_color_handler, id);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);

            // viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

            while (!viewer->wasStopped() ){
                viewer->spinOnce(50);
            }
        }

        /**
         * @brief 
         * 
         * @param headless disable visualization, maybe useful when load from file
         * @return ** void 
         */
        void generateThumbnail(bool headless = false) {
            pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>);
            mCloudGrid.lock();
            if (access_status == -1) {
                pcl::PointCloud<PointType>::Ptr laserCloudMapCorner(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr laserCloudMapSurf(new pcl::PointCloud<PointType>());
                std::stringstream ss;
                ss << robot_id*100+submap_id;
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"corner.pcd", *laserCloudMapCorner) == -1) {
                    PCL_ERROR ("Couldn't read file corner cloud \n");
                }
                if (pcl::io::loadPCDFile<PointType> (CACHE_PATH+ss.str()+"surf.pcd", *laserCloudMapSurf) == -1) {
                    PCL_ERROR ("Couldn't read file surf cloud \n");
                }
                *laserCloudMap += *laserCloudMapCorner;
                *laserCloudMap += *laserCloudMapSurf;

            } else {

                for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
                {
                    *laserCloudMap += *laserCloudCornerArray[i];
                    // *edgeCloudMap += *laserCloudCornerArray[i];
                    *laserCloudMap += *laserCloudSurfArray[i];
                }
            }
            mCloudGrid.unlock();
            // *laserCloudMap += *thumbnailLast;
            
            //down sample
            pcl::ApproximateVoxelGrid<PointType> avg;
            avg.setLeafSize(0.5,0.5,0.5);
            avg.setInputCloud(laserCloudMap);
            avg.filter(*laserCloudMap);


            //outlier remove
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setMeanK (5);
            sor.setStddevMulThresh (1.0);
            sor.setInputCloud (laserCloudMap);
            sor.filter (*laserCloudMap);



            pcl::SACSegmentation<PointType> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            //seg.setMaxIterations(300);
            seg.setDistanceThreshold(0.1);

            seg.setInputCloud(laserCloudMap);
            seg.segment(*inliers, *coefficients);

            pcl::PointCloud<PointType>::Ptr c_plane (new pcl::PointCloud<PointType>());  // store ground points
            pcl::PointCloud<PointType>::Ptr c_building (new pcl::PointCloud<PointType>());
            pcl::ExtractIndices<PointType> extract; 
            // pcl::ExtractIndices<PointType> extract1;  
            extract.setInputCloud (laserCloudMap);    //input cloud
            extract.setIndices (inliers);     
            extract.setNegative (false);      //false for inliers(ground), true for outliers (buildings)
            extract.filter (*c_plane); 
            extract.setInputCloud (laserCloudMap);    //input cloud
            extract.setIndices (inliers);        //extract segmentation to c_plane
            extract.setNegative (true);      //false for inliers(ground), true for outliers (buildings)
            extract.filter (*c_building);
            

            // *c_building = *edgeCloudMap;
            // remove trees
            pcl::PointIndices::Ptr inliers_dist(new pcl::PointIndices);

            // 这里有bug，有可能values的大小不足4个
            if(coefficients->values.size() < 4)
            {
                // -1代表失败
                thumbnails_db.push_back(std::make_pair(-1, ""));
                return;
            }

            coeffs << coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3];
            for (int i = 0; i < c_building->points.size(); i++) {
                Eigen::Vector4f pt(c_building->points[i].x,c_building->points[i].y,c_building->points[i].z, 1);
                float dist = pt.dot(coeffs)/coeffs.head(3).norm();
                c_building->points[i].intensity=255;
                if ((dist > 0.1 && dist < 20) || (dist < -0.1 && dist > -20)) {
                    inliers_dist->indices.push_back(i);
                }
            }
            
            extract.setInputCloud(c_building);
            extract.setIndices(inliers_dist);
            extract.setNegative(false);
            extract.filter(*c_building);
            

            // remove horizonal points
            pcl::NormalEstimation<PointType, pcl::Normal> ne;
            ne.setInputCloud (c_building);
            pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
            ne.setSearchMethod (tree);
            pcl::PointCloud<pcl::Normal>::Ptr building_normals (new pcl::PointCloud<pcl::Normal>);
            // Use all neighbors in a sphere of radius 2m
            ne.setRadiusSearch(2);
            ne.compute (*building_normals);
            
            Eigen::Vector4f norm_plane;
            norm_plane  = coeffs;
            Eigen::Vector4f norm_tmp = norm_plane;
            norm_tmp(3) = 0;
            norm_tmp.normalize();
            norm_plane.head(3) = norm_tmp.head(3);
            pcl::PointIndices::Ptr inliers_normal(new pcl::PointIndices);
            for (int i = 0; i < c_building->points.size(); i++) {
                Eigen::Vector3f norm_point;
                norm_point << building_normals->points[i].normal_x , building_normals->points[i].normal_y, building_normals->points[i].normal_z;
                Eigen::Quaternionf q_temp;
                q_temp.setFromTwoVectors(norm_plane.head(3), norm_point);
                float rad = q_temp.angularDistance(Eigen::Quaternionf::Identity());
                if (rad*57.3 < 45 || rad*57.3> 135) {
                    
                } else {
                    inliers_normal->indices.push_back(i);
                }

            }

            extract.setInputCloud(c_building);
            extract.setIndices(inliers_normal);
            extract.setNegative(false);
            extract.filter(*c_building);
            
            // *thumbnailBCur += *c_building;
            pcl::PointCloud<PointType>::Ptr c_projected (new pcl::PointCloud<PointType>());  //存储平面点云
            pcl::ProjectInliers<PointType> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(c_building);
            proj.setModelCoefficients(coefficients);
            proj.filter(*thumbnailCur);

            

            std::stringstream ss;
            ss << (robot_id*100+submap_id);
            std::string s;
            ss >> s;
           
            static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;
			transform.setOrigin(tf::Vector3(t_world_base(0),
											t_world_base(1),
											t_world_base(2)));
			q.setW(q_world_base.w());
			q.setX(q_world_base.x());
			q.setY(q_world_base.y());
			q.setZ(q_world_base.z());
			transform.setRotation(q);
             
			br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(timeLaserOdometry), "/camera", "/submap"+s));

            sensor_msgs::PointCloud2 laserCloudThumbnail;
            pcl::toROSMsg(*thumbnailCur, laserCloudThumbnail);
            laserCloudThumbnail.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudThumbnail.header.frame_id = "/submap"+s;
            pubSubmapThumbnail.publish(laserCloudThumbnail);


            // add thumbnail to next submap
            // addThumbnailToNext();

            // test image generation
            Eigen::Vector3f norm_temp = coeffs.head(3);
            Eigen::Vector3f z_axis (0,0,1);
            Eigen::Quaternionf q_temp;
            q_temp.setFromTwoVectors(norm_temp, z_axis);
            // check inverse norm, air case only
            if (NEED_CHECK_DIRECTION == 1 ) {
                // lidar 45degree down facing
                Eigen::Quaternionf q_check;
                q_check.setFromTwoVectors(norm_temp, Eigen::Vector3f(0,0,1));
                if(DEBUG) ROS_ERROR_STREAM(s << " normal angle " << q_check.angularDistance(Eigen::Quaternionf::Identity())*57.3);
                if(DEBUG) ROS_ERROR_STREAM(s << " z angle " << q_temp.angularDistance(Eigen::Quaternionf::Identity())*57.3);
                if(q_check.angularDistance(Eigen::Quaternionf::Identity())*57.3 < 90) {
                    norm_temp *= -1;
                    
                    q_temp.setFromTwoVectors(norm_temp, z_axis);
                }
            }
            Eigen::Matrix4f transform_temp = Eigen::Matrix4f::Identity();
            transform_temp.topLeftCorner(3,3) = q_temp.toRotationMatrix();
            pcl::PointCloud<PointType>::Ptr z_aligned (new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*thumbnailCur, *z_aligned, transform_temp);
            if(DEBUG) ROS_ERROR_STREAM("done transform");

            // demean air case only
            if (NEED_CHECK_DIRECTION == 1 ) {
                Eigen::Matrix<float,4,1> centroid;
                pcl::compute3DCentroid(*z_aligned, centroid);
                pcl::demeanPointCloud(*z_aligned, centroid, *z_aligned);
            }

            *thumbnailCur = *z_aligned; // save thumbnail point cloud
            if(DEBUG) ROS_ERROR_STREAM("done check direction");

            std::vector<int> vecDataX(z_aligned->size(), 0), vecDataY(z_aligned->size(), 0);
            int shiftX = 0, shiftY = 0;
            for (int i = 0; i < z_aligned->size(); i++)
            {
                //      vecDataX.push_back(round(cloud->points[i].x));
                //		vecDataY.push_back(round(cloud->points[i].y));
                vecDataX[i] = round(2*z_aligned->points[i].x);
                vecDataY[i] = round(2*z_aligned->points[i].y);
                // ROS_ERROR_STREAM(" rrrr" << z_aligned->points[i].z);
            }
            // ROS_ERROR_STREAM("1 ");
            double maxElementX = 50;//*(std::max_element(vecDataX.begin(), vecDataX.end()));
            double minElementX = -50;//*(std::min_element(vecDataX.begin(), vecDataX.end()));
            double maxElementY = 50;//*(std::max_element(vecDataY.begin(), vecDataY.end()));
            double minElementY = -50;//*(std::min_element(vecDataY.begin(), vecDataY.end()));
            // ROS_ERROR_STREAM("2 " << maxElementX << " " << minElementX << " " << maxElementY << " " << minElementY);
            if (-minElementX > shiftX)
                shiftX = -minElementX;
            if (-minElementY > shiftY)
                shiftY = -minElementY;
            // ROS_ERROR_STREAM("3 ");
            cv::Mat image(1.0 * (maxElementY - minElementY), 1.0 * (maxElementX - minElementX), CV_8UC3, cv::Scalar(204,200,202));
            // ROS_ERROR_STREAM("4 ");
            for (int i = 0; i < z_aligned->size(); i++)
            {
                vecDataX[i] += shiftX;
                vecDataY[i] += shiftY;
            }
            for (int i = 0; i < z_aligned->size(); i++)
            {   
                int u =  vecDataX[i];
                int v =  maxElementY - minElementY - vecDataY[i];//abs(maxElementY - minElementY - vecDataY[i]);
                if (u <0 || v<0 || u >= image.cols|| v >= image.rows) {
                    continue;
                }
                
                // cv::circle(image, cv::Point2f(u,v) , 1, cv::Scalar(0, 0, 255), -1);
                image.at<cv::Vec3b>(v,u)[0] = 168;
                image.at<cv::Vec3b>(v,u)[1] = 107;
                image.at<cv::Vec3b>(v,u)[2] = 65;
            }

            if (!headless) {
                if(DEBUG) ROS_ERROR_STREAM("wait display thumbnail");
                cv::imshow("thumbnail", image);
                cv::waitKey(5);
            }
            std::string home_dir = std::getenv("HOME");    
            imwrite( home_dir + "/gacm_output/pictures/submap_img/submap" +s+ ".png", image);
            thumbnails_db.push_back(std::make_pair(0, home_dir + "/gacm_output/pictures/submap_img/submap" +s+ ".png"));
            thumbnailGenerated = true;
            if(DEBUG) ROS_ERROR_STREAM("done generate thumbnail");
        }

        void getWorldFramePoseAt(int fid, Eigen::Quaterniond& q, Eigen::Vector3d& t) {
            q = pose_graph_local[fid].q * q_world_base;
            t = t_world_base + q_world_base*pose_graph_local[fid].t;
        }



        void generateThumbnailForFrame(int fid) {
            pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>);
            for (int i = 0; i <laserCloudCornerArray.size(); i++) // voxel number
            {
                *laserCloudMap += *laserCloudCornerArray[i];
                *laserCloudMap += *laserCloudSurfArray[i];
            }

            Eigen::Quaterniond wq = pose_graph_local[fid].q;
            Eigen::Vector3d wt = pose_graph_local[fid].t;
            // getWorldFramePoseAt(fid, wq, wt);

            Eigen::Matrix4f transform;
			transform.topRightCorner(3,1) = -1*(wq.inverse()*wt).cast<float>();
			transform.topLeftCorner(3,3) = wq.inverse().toRotationMatrix().cast<float>();

            // transform = transform.inverse;
            
            pcl::transformPointCloud(*laserCloudMap, *laserCloudMap, transform);

            // *laserCloudMap += *thumbnailLast;

            //outlier remove
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setMeanK (20);
            sor.setStddevMulThresh (1.0);
            sor.setInputCloud (laserCloudMap);
            sor.filter (*laserCloudMap);



            pcl::SACSegmentation<PointType> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(300);
            seg.setDistanceThreshold(0.2);

            seg.setInputCloud(laserCloudMap);
            seg.segment(*inliers, *coefficients);

            pcl::PointCloud<PointType>::Ptr c_plane (new pcl::PointCloud<PointType>());  // store ground points
            pcl::PointCloud<PointType>::Ptr c_building (new pcl::PointCloud<PointType>());
            pcl::ExtractIndices<PointType> extract; 
            // pcl::ExtractIndices<PointType> extract1;  
            extract.setInputCloud (laserCloudMap);    //input cloud
            extract.setIndices (inliers);     
            extract.setNegative (false);      //false for inliers(ground), true for outliers (buildings)
            extract.filter (*c_plane); 
            extract.setInputCloud (laserCloudMap);    //input cloud
            extract.setIndices (inliers);        //extract segmentation to c_plane
            extract.setNegative (true);      //false for inliers(ground), true for outliers (buildings)
            extract.filter (*c_building);
            
            // remove trees
            pcl::PointIndices::Ptr inliers_dist(new pcl::PointIndices);
            coeffs << coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3];
            for (int i = 0; i < c_building->points.size(); i++) {
                Eigen::Vector4f pt(c_building->points[i].x,c_building->points[i].y,c_building->points[i].z, 1);
                float dist = pt.dot(coeffs)/coeffs.head(3).norm();
                c_building->points[i].intensity=255;
                // if (dist >= 1.5 && dist < 60000 || dist <= -1.5 && dist > -6000000) { //car
                if (dist >= 1 && dist < 10 || dist <= -1 && dist > -10) { // air
                    inliers_dist->indices.push_back(i);
                }
            }
            
            extract.setInputCloud(c_building);
            extract.setIndices(inliers_dist);
            extract.setNegative(false);
            extract.filter(*c_building);


            // remove horizonal points
            pcl::NormalEstimation<PointType, pcl::Normal> ne;
            ne.setInputCloud (c_building);
            pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
            ne.setSearchMethod (tree);
            pcl::PointCloud<pcl::Normal>::Ptr building_normals (new pcl::PointCloud<pcl::Normal>);
            // Use all neighbors in a sphere of radius 2m
            ne.setRadiusSearch (2.0);
            ne.compute (*building_normals);
            
            Eigen::Vector4f norm_plane;
            norm_plane  = coeffs;
            Eigen::Vector4f norm_tmp = norm_plane;
            norm_tmp(3) = 0;
            norm_tmp.normalize();
            norm_plane.head(3) = norm_tmp.head(3);
            pcl::PointIndices::Ptr inliers_normal(new pcl::PointIndices);
            for (int i = 0; i < c_building->points.size(); i++) {
                Eigen::Vector3f norm_point;
                norm_point << building_normals->points[i].normal_x , building_normals->points[i].normal_y, building_normals->points[i].normal_z;
                Eigen::Quaternionf q_temp;
                q_temp.setFromTwoVectors(norm_plane.head(3), norm_point);
                float rad = q_temp.angularDistance(Eigen::Quaternionf::Identity());
                if (rad*57.3 < 15 || rad*57.3> 165) {
                    
                } else {
                    inliers_normal->indices.push_back(i);
                }

            }

            extract.setInputCloud(c_building);
            extract.setIndices(inliers_normal);
            extract.setNegative(false);
            extract.filter(*c_building);
            
            // *thumbnailBCur += *c_building;
            pcl::PointCloud<PointType>::Ptr c_projected (new pcl::PointCloud<PointType>());  //存储平面点云
            pcl::ProjectInliers<PointType> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(c_building);
            proj.setModelCoefficients(coefficients);
            proj.filter(*c_projected);

            

            std::stringstream ss;
            ss << (robot_id*10000+submap_id*1000+fid);
            std::string s;
            ss >> s;
			

            // test image generation
            Eigen::Vector3f norm_temp = coeffs.head(3);
            Eigen::Vector3f z_axis (0,0,1);
            Eigen::Quaternionf q_temp;
            q_temp.setFromTwoVectors(norm_temp, z_axis);
            // check inverse norm, air case only

            if (NEED_CHECK_DIRECTION == 1) {

                Eigen::Vector3f ideal_gravity(0,1,1); // lidar 45degree down facing
                Eigen::Quaternionf q_check;
                q_check.setFromTwoVectors(norm_temp, ideal_gravity);
                if(DEBUG) ROS_ERROR_STREAM(s << "normal angle " << q_check.angularDistance(Eigen::Quaternionf::Identity())*57.3);
                if(DEBUG) ROS_ERROR_STREAM(s << "z angle " << q_temp.angularDistance(Eigen::Quaternionf::Identity())*57.3);
                if(q_check.angularDistance(Eigen::Quaternionf::Identity())*57.3 < 90) {
                    norm_temp *= -1;
                    q_temp.setFromTwoVectors(norm_temp, z_axis);
                }
            }
            Eigen::Matrix4f transform_temp = Eigen::Matrix4f::Identity();
            transform_temp.topLeftCorner(3,3) = q_temp.toRotationMatrix();
            pcl::PointCloud<PointType>::Ptr z_aligned (new pcl::PointCloud<PointType>());
            // pcl::PointCloud<PointType>::Ptr z_aligned_plane (new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*c_projected, *z_aligned, transform_temp);

            std::vector<int> vecDataX(z_aligned->size(), 0), vecDataY(z_aligned->size(), 0);
            // std::vector<int> vecDataX_plane(z_aligned_plane->size(), 0), vecDataY_plane(z_aligned_plane->size(), 0);
            int shiftX = 0, shiftY = 0;
            for (int i = 0; i < z_aligned->size(); i++)
            {
                vecDataX[i] = round(2*z_aligned->points[i].x);
                vecDataY[i] = round(2*z_aligned->points[i].y);
            }
            double maxElementX = 50;//*(std::max_element(vecDataX.begin(), vecDataX.end()));
            double minElementX = -50;//*(std::min_element(vecDataX.begin(), vecDataX.end()));
            double maxElementY = 50;//*(std::max_element(vecDataY.begin(), vecDataY.end()));
            double minElementY = -50;//*(std::min_element(vecDataY.begin(), vecDataY.end()));
            // ROS_ERROR_STREAM("2 " << maxElementX << " " << minElementX << " " << maxElementY << " " << minElementY);
            if (-minElementX > shiftX)
                shiftX = -minElementX;
            if (-minElementY > shiftY)
                shiftY = -minElementY;
            // ROS_ERROR_STREAM("3 ");
            cv::Mat image(1.0 * (maxElementY - minElementY), 1.0 * (maxElementX - minElementX), CV_8UC3, cv::Scalar(0,255,0));
            // ROS_ERROR_STREAM("4 ");
            for (int i = 0; i < z_aligned->size(); i++)
            {
                vecDataX[i] += shiftX;
                vecDataY[i] += shiftY;
            }
            for (int i = 0; i < z_aligned->size(); i++)
            {   
                int u =  vecDataX[i];
                int v = maxElementY - minElementY - vecDataY[i];//abs(maxElementY - minElementY - vecDataY[i]);
                if (u < 0 || v< 0 || u >= image.cols|| v >= image.rows) {
                    continue;
                }
                image.at<cv::Vec3b>(v,u)[0] = 0;
                image.at<cv::Vec3b>(v,u)[1] = 0;
                image.at<cv::Vec3b>(v,u)[2] = 255;
            }
            
            imwrite( std::string(std::getenv("HOME")) + "/gacm_output/pictures/eachframe/submap" +s+".png", image);


        }

    

};



#endif
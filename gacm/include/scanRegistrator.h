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

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>

#include "cloud_msgs/cloud_info.h"
#include "util/ip_basic.h"
#include "parameters.h"

typedef pcl::PointXYZI PointType;

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class ScanRegistrator {

    private:

        camodocal::CameraPtr m_camera;

        std_msgs::Header cloudHeader;
        pcl::PointCloud<PointType>::Ptr laserCloudIn;

        pcl::PointCloud<PointType>::Ptr fullCloudImage;
        pcl::PointCloud<PointType>::Ptr fullRangeCloudImage;

        //  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudScans;
        pcl::PointCloud<PointType>::Ptr laserCloud;
        pcl::PointCloud<PointType>::Ptr laserRangeCloud;

        pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

        pcl::VoxelGrid<PointType> downSizeFilter;

       
        cloud_msgs::cloud_info segMsg;
        
        cv::Mat rangeMat;
        PointType nanPoint;
        std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

        std::vector<float> cloudCurvature;
        std::vector<int> cloudSortInd;
        std::vector<smoothness_t> cloudSmoothness;
        std::vector<int> cloudNeighborPicked;
        std::vector<int> cloudLabel;

        template <typename PointT>
        void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                    pcl::PointCloud<PointT> &cloud_out, float thres) {
                if (&cloud_in != &cloud_out)
                {
                    cloud_out.header = cloud_in.header;
                    cloud_out.points.resize(cloud_in.points.size());
                }

                size_t j = 0;

                for (size_t i = 0; i < cloud_in.points.size(); ++i)
                {
                    if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                        continue;
                    cloud_out.points[j] = cloud_in.points[i];
                    j++;
                }
                if (j != cloud_in.points.size())
                {
                    cloud_out.points.resize(j);
                }

                cloud_out.height = 1;
                cloud_out.width = static_cast<uint32_t>(j);
                cloud_out.is_dense = true;
        }

    public:

        ScanRegistrator() {

            

            nanPoint.x = std::numeric_limits<float>::quiet_NaN();
            nanPoint.y = std::numeric_limits<float>::quiet_NaN();
            nanPoint.z = std::numeric_limits<float>::quiet_NaN();
            nanPoint.intensity = -1;

            allocateMemory();
            resetParameters();
        }

        void allocateMemory() {

            laserCloudIn.reset(new pcl::PointCloud<PointType>());

            fullCloudImage.reset(new pcl::PointCloud<PointType>());
            fullRangeCloudImage.reset(new pcl::PointCloud<PointType>());

            cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
            cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
            surfPointsFlat.reset(new pcl::PointCloud<PointType>());
            surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

            surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
            surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());


            fullCloudImage->points.resize(N_SCANS*HORIZON_SCANS);
            fullRangeCloudImage->points.resize(N_SCANS*HORIZON_SCANS);

            
            // for (auto pcptr: laserCloudScans) {
            //     pcptr.reset(new pcl::PointCloud<PointType>());
            // }
            // for (size_t i = 0; i < N_SCANS; i++) {
            //     laserCloudScans.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()));
            // }
            
            laserCloud.reset(new pcl::PointCloud<PointType>());
            laserRangeCloud.reset(new pcl::PointCloud<PointType>());
            cloudCurvature.assign(N_SCANS*HORIZON_SCANS, -1);
            cloudNeighborPicked.assign(N_SCANS*HORIZON_SCANS, -1);
            cloudSortInd.assign(N_SCANS*HORIZON_SCANS, -1);
            cloudLabel.assign(N_SCANS*HORIZON_SCANS, 0);
            cloudSmoothness.resize(N_SCANS*HORIZON_SCANS);

            segMsg.startRingIndex.assign(N_SCANS, 0);
            segMsg.endRingIndex.assign(N_SCANS, 0);

            segMsg.segmentedCloudGroundFlag.assign(N_SCANS*HORIZON_SCANS, false);
            segMsg.segmentedCloudColInd.assign(N_SCANS*HORIZON_SCANS, 0);
            segMsg.segmentedCloudRange.assign(N_SCANS*HORIZON_SCANS, 0);

            std::pair<int8_t, int8_t> neighbor;
            neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
            neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
            neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
            neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

            
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        }

        void resetParameters(){
            laserCloudIn->clear();
            // ROS_INFO_STREAM("N_SCANS " << N_SCANS << " HS " << HORIZON_SCANS);
            rangeMat = cv::Mat(N_SCANS, HORIZON_SCANS, CV_32F, cv::Scalar::all(-1));
            

            std::fill(fullCloudImage->points.begin(), fullCloudImage->points.end(), nanPoint);
            std::fill(fullRangeCloudImage->points.begin(), fullRangeCloudImage->points.end(), nanPoint);
        }

        void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
            cloudHeader = laserCloudMsg->header;
            pcl::PointCloud<PointType>::Ptr templaser(new pcl::PointCloud<PointType>());

            pcl::fromROSMsg(*laserCloudMsg, *templaser);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            for (int i=0; i < templaser->size(); i++) {
                Eigen::Vector3d temp;
                temp << templaser->points[i].x, templaser->points[i].y, templaser->points[i].z;
                if (temp.norm() > CROP_NEAR && temp.norm() < CROP_FAR) {
                    inliers->indices.push_back(i);
                }
            }
            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud (templaser);    //input cloud
            extract.setIndices (inliers);     
            extract.setNegative (false);     
            extract.filter (*laserCloudIn);

            
        }

        void findStartEndAngle(){
            segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
            segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                        laserCloudIn->points[laserCloudIn->points.size() - 2].x) + 2 * M_PI;
            if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
                segMsg.endOrientation -= 2 * M_PI;
            } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
                segMsg.endOrientation += 2 * M_PI;
            segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
        }

        // project into a 16*1800 range image
        void projectPointCloud(){
            // for (auto scan_cloud: laserCloudScans) {
            //     scan_cloud->clear();
            // }
            // laserRangeCloud->clear();

            float verticalAngle, horizonAngle, range;
            size_t rowIdn, columnIdn, index, cloudSize; 
            PointType thisPoint;
            PointType thisPointProjected;

            cloudSize = laserCloudIn->points.size();

            for (size_t i = 0; i < cloudSize; ++i){

                thisPoint.x = laserCloudIn->points[i].x;
                thisPoint.y = laserCloudIn->points[i].y;
                thisPoint.z = laserCloudIn->points[i].z;

                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ANGLE_BOTTOM) / ANGLE_RES_Y;
                if (rowIdn < 0 || rowIdn >= N_SCANS)
                    continue;

                horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

                columnIdn = -round((horizonAngle-90.0)/ANGLE_RES_X) + HORIZON_SCANS/2;
                if (columnIdn >= HORIZON_SCANS)
                    columnIdn -= HORIZON_SCANS;

                if (columnIdn < 0 || columnIdn >= HORIZON_SCANS)
                    continue;

                range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
                // ROS_INFO_STREAM("Processing point " << rowIdn << ", " << columnIdn);
                rangeMat.at<float>(rowIdn, columnIdn) = range;
                // ROS_INFO_STREAM("Stack in range Image");

                thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;


                thisPointProjected.x = (float)rowIdn;
                thisPointProjected.y = (float)columnIdn;
                thisPointProjected.z = range;
                thisPointProjected.intensity = range;

                index = columnIdn  + rowIdn * HORIZON_SCANS;

                fullCloudImage->points[index] = thisPoint;
                fullRangeCloudImage->points[index] = thisPointProjected;
                
            }

            // for (size_t i = 0; i < N_SCANS; i++) {

            //     ROS_INFO_STREAM("After Project pc Lasercloudscans " << i << laserCloudScans[i]->size());
            // }

        }

        void organizePointCloud() {
            // ROS_INFO_STREAM("LaserCloud origin size " << laserCloud->size());
            laserCloud->clear();
            laserRangeCloud->clear();
            // ROS_INFO_STREAM("LaserCloud afterclear " << laserCloud->size());
            // for (int i = 0; i < N_SCANS; i++) { 
            //     segMsg.startRingIndex[i] = laserCloud->size() + 5;
            //     ROS_INFO_STREAM("laserCloud Scan " << i << " size " << laserCloudScans[i]->size());
            //     *laserCloud += *laserCloudScans[i];
            //     segMsg.endRingIndex[i] = laserCloud->size() - 6;
            // }

            PointType thisPointProjected;
            for (int i = 0; i < N_SCANS; i++) {
                segMsg.startRingIndex[i] = laserCloud->size() + 5;
                for (int j = 0; j < HORIZON_SCANS; j++) {
                    float range = rangeMat.at<float>(i, j);
                    int index = j  + i * HORIZON_SCANS;
                    if (rangeMat.at<float>(i, j) > 0) {
                        // thisPointProjected.x = (float)i;
                        // thisPointProjected.y = (float)j;
                        // thisPointProjected.z = range;
                        // thisPointProjected.intensity = range;
                        laserRangeCloud->points.push_back(fullRangeCloudImage->points[index]);
                        laserCloud->push_back(fullCloudImage->points[index]);
                    }
                }
                segMsg.endRingIndex[i] = laserCloud->size() - 6;
            }
        }

        void calculateSmoothness(){

            int cloudSize = laserCloud->points.size();
            for (int i = 5; i < cloudSize - 5; i++) {

                float diffRange = laserRangeCloud->points[i-5].intensity + laserRangeCloud->points[i-4].intensity
                                + laserRangeCloud->points[i-3].intensity + laserRangeCloud->points[i-2].intensity
                                + laserRangeCloud->points[i-1].intensity - laserRangeCloud->points[i].intensity * 10
                                + laserRangeCloud->points[i+1].intensity + laserRangeCloud->points[i+2].intensity
                                + laserRangeCloud->points[i+3].intensity + laserRangeCloud->points[i+4].intensity
                                + laserRangeCloud->points[i+5].intensity;            

                // Lego_LOAM和LIO_SAM这里都是：cloudCurvature[i] = diffRange*diffRange;
                // cloudCurvature[i] = diffRange*diffRange/laserRangeCloud->points[i].intensity;
                cloudCurvature[i] = diffRange*diffRange;

                cloudNeighborPicked[i] =  0;
                cloudLabel[i] = 0;

                cloudSmoothness[i].value = cloudCurvature[i];
                cloudSmoothness[i].ind = i;
            }
        }

        void markOccludedPoints() {
            int cloudSize = laserCloud->points.size();

            for (int i = 5; i < cloudSize - 6; ++i){

                float depth1 = laserRangeCloud->points[i].intensity;
                float depth2 = laserRangeCloud->points[i+1].intensity;
                float depth3 = laserRangeCloud->points[i-1].intensity;
                int columnDiff = std::abs(int(laserRangeCloud->points[i+1].y - laserRangeCloud->points[i].y));

                if (columnDiff < 10){
                    // points far away are occluded
                    if (depth1 - depth2 > OCCLUDE_THRESHOLD){
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    } else if (depth2 - depth1 > OCCLUDE_THRESHOLD){
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }

                float diff1 = std::abs(depth3 - depth1);
                float diff2 = std::abs(depth2 - depth1);

                if (diff1 > 0.02 * depth1 && diff2 > 0.02 * depth1)
                    cloudNeighborPicked[i] = 1;
            }
        }

         void extractFeatures() {
            cornerPointsSharp->clear();
            cornerPointsLessSharp->clear();
            surfPointsFlat->clear();
            surfPointsLessFlat->clear();

            for (int i = 0; i < N_SCANS; i++) {

                surfPointsLessFlatScan->clear();

                for (int j = 0; j < 6; j++) {

                    int sp = (segMsg.startRingIndex[i] * (6 - j)    + segMsg.endRingIndex[i] * j) / 6;
                    int ep = (segMsg.startRingIndex[i] * (5 - j)    + segMsg.endRingIndex[i] * (j + 1)) / 6 - 1;
                    // ROS_INFO_STREAM("Processing scan line " << i << "sp " << sp << " ep " << ep);
                    if (sp >= ep)
                        continue;
                    
 
                    std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());
 

                    int largestPickedNum = 0;
                    for (int k = ep; k >= sp; k--) {
                        int ind = cloudSmoothness[k].ind;
                        if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] > EDGE_THRESHOLD ) {
                        
                            largestPickedNum++;
                            if (largestPickedNum <= 2) {
                                cloudLabel[ind] = 2;
                                cornerPointsSharp->push_back(laserCloud->points[ind]);
                                // ROS_INFO_STREAM("Push back corner " << cloudCurvature[ind] << "$" << cloudSmoothness[k].value);
                                cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                                // cornerPointsLessSharp->back().intensity = cloudSmoothness[k].value;
                            } else if (largestPickedNum <= 30) {
                                cloudLabel[ind] = 1;
                                cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                            } else {
                                break;
                            }

                            cloudNeighborPicked[ind] = 1;
                            for (int l = 1; l <= 5; l++) {
                                int columnDiff = std::abs(int(laserRangeCloud->points[ind + l].y - laserRangeCloud->points[ind + l - 1].y));
                                if (columnDiff > 10)
                                    break;
                                cloudNeighborPicked[ind + l] = 1;
                            }
                            for (int l = -1; l >= -5; l--) {
                                int columnDiff = std::abs(int(laserRangeCloud->points[ind + l].y - laserRangeCloud->points[ind + l + 1].y));
                                if (columnDiff > 10)
                                    break;
                                cloudNeighborPicked[ind + l] = 1;
                            }
                        }
                    }

                    int smallestPickedNum = 0;
                    // ROS_INFO_STREAM("Check Surf threshold " << SURF_THRESHOLD);
                    for (int k = sp; k <= ep; k++) {
                        int ind = cloudSmoothness[k].ind;
                        if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] < SURF_THRESHOLD) {

                            cloudLabel[ind] = -1;
                            surfPointsFlat->push_back(laserCloud->points[ind]);
                            // ROS_INFO_STREAM("Push back surf");

                            smallestPickedNum++;
                            if (smallestPickedNum >= 4) {
                                break;
                            }

                            cloudNeighborPicked[ind] = 1;
                            for (int l = 1; l <= 5; l++) {

                                int columnDiff = std::abs(int(laserRangeCloud->points[ind + l].y - laserRangeCloud->points[ind + l - 1].y));
                                if (columnDiff > 10)
                                    break;

                                cloudNeighborPicked[ind + l] = 1;
                            }
                            for (int l = -1; l >= -5; l--) {

                                int columnDiff = std::abs(int(laserRangeCloud->points[ind + l].y - laserRangeCloud->points[ind + l + 1].y));
                                if (columnDiff > 10)
                                    break;

                                cloudNeighborPicked[ind + l] = 1;
                            }
                        }
                        // 增加特征点数量
                        else if(cloudCurvature[ind] < SURF_THRESHOLD)
                        {
                            cloudLabel[ind] = -1;
                        }
                    }

                    for (int k = sp; k <= ep; k++) {
                        // ROS_INFO_STREAM("Check label " << cloudLabel[k] << " $ " << cloudCurvature[k]);
                        // LIO-SAM and Lego-LOAM：if (cloudLabel[k] <= 0)
                        // 这里只取小于0的点
                        if (cloudLabel[k] < 0) {
                            // ROS_INFO("Push back less flat");
                            surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                            // surfPointsLessFlatScan->back().intensity = cloudCurvature[k];
                        }
                    }
                }

                surfPointsLessFlatScanDS->clear();
                downSizeFilter.setInputCloud(surfPointsLessFlatScan);
                downSizeFilter.filter(*surfPointsLessFlatScanDS);

                *surfPointsLessFlat += *surfPointsLessFlatScanDS;
            }

            // ROS_INFO_STREAM("After Feature Extract size " << cornerPointsSharp->size() << ";" << cornerPointsLessSharp->size() << ";" << surfPointsFlat->size() << ";" << surfPointsLessFlat->size() << ";" << surfPointsLessFlatScanDS->size());
            // ROS_INFO_STREAM("scan corner: " << cornerPointsSharp->size() << ", scan plane: " << surfPointsFlat->size() << ", scan Less plane: " << surfPointsLessFlat->size());
        }

        // 这个函数根本没有被调用过
        void pointCloudToImage(pcl::PointCloud<PointType>::Ptr pointcloud) {
            cv::Mat depthmap(ROW, COL, CV_16UC1, cv::Scalar(0));
            // int valid_cnt = 0;

            /* 谁他妈写的，居然把外参写死了 */
            // Eigen::Quaterniond q_extrinsic(0.704888,    0.709282, 0.007163,0.000280);
            // Eigen::Vector3d t_extrinsic(0.287338, -0.371119, -0.071403);

            // R_extrinsic << 0.9999, 0.0098, 0.0105, 0.0106, -0.0062, -0.9999, -0.0097, 0.9999,-0.0063;
            // q_extrinsic = R_extrinsic;
            // std::cout << "Check extrinsic \n" << R_extrinsic << std::endl; 

            for (auto point: pointcloud->points) {
                // Eigen::Vector3d P(-point.y, -point.z, point.x); // under camera coordinate
                // PointType temp = point;
                Eigen::Vector3d P(point.x, point.y, point.z);
                P = RCL * P + TCL;
                
                Eigen::Vector2d p;
                m_camera->spaceToPlane(P,p);
                
                if (p[1] >0 && p[0] >0 && p[1]< ROW && p[0] < COL && P[2] > 0) {
                    depthmap.at<ushort>(p[1], p[0]) = (ushort)P[2];
                    // valid_cnt++;
                } else {
                    // ROS_INFO_STREAM("PCL point" << point.x << " ;" << point.y << " ;" << point.z << " Current point " << P[0]<<" ;" << P[1]<<" ;" <<P[2] << " Project point " << p[0]<<" ;"<<p[1]);
                }
            }
            displayFalseColors(depthmap, "laser origin");
            // ROS_INFO_STREAM("Depth count in range " << valid_cnt << "/" << pointcloud->size());
            customDilate(depthmap, depthmap, 5, KERNEL_TYPE_DIAMOND);
            customDilate(depthmap, depthmap, 3, KERNEL_TYPE_DIAMOND);
            displayFalseColors(depthmap, "laser map");
            cv::waitKey(5);
        }

        pcl::PointCloud<PointType>::Ptr toCameraFrame(pcl::PointCloud<PointType>::Ptr& inCloud) {
            pcl::PointCloud<PointType>::Ptr outCloud;
            outCloud.reset(new pcl::PointCloud<PointType>());
            // Eigen::Quaterniond q_extrinsic(0.704888,    0.709282, 0.007163,0.000280);
            // Eigen::Vector3d t_extrinsic(0.287338, -0.371119, -0.071403);
            outCloud->points.reserve(inCloud->points.size());
            for (auto point : inCloud->points) {
                PointType temp = point;
                Eigen::Vector3d temp_P(temp.x, temp.y, temp.z);
                temp_P = RCL*temp_P+TCL; //q_extrinsic * temp_P + t_extrinsic;
                temp.x = temp_P(0);
                temp.y = temp_P(1);
                temp.z = temp_P(2);
                temp.intensity = point.intensity;
                // temp.x = -1*point.y;
                // temp.y = -1*point.z;
                // temp.z = point.x;
                outCloud->push_back(temp);
            }
            return outCloud;
        }

        pcl::PointCloud<PointType>::Ptr getFullResCloud() {
            // return laserCloud;
            return toCameraFrame(laserCloud);
        }

        pcl::PointCloud<PointType>::Ptr getCornerPointsSharp () {
            // ROS_INFO_STREAM("Return Corner points Sharp " << cornerPointsSharp->points.size());
            // return cornerPointsSharp;
            return toCameraFrame(cornerPointsSharp);
        }

        pcl::PointCloud<PointType>::Ptr getCornerPointsLessSharp () {
            // ROS_INFO_STREAM("Return Corner points less Sharp " << cornerPointsLessSharp->points.size());
            // return cornerPointsLessSharp;
            return toCameraFrame(cornerPointsLessSharp);
        }
        pcl::PointCloud<PointType>::Ptr getSurfPointsFlat () {
            // ROS_INFO_STREAM("Return Surf points Flat " << surfPointsFlat->points.size());
            // return surfPointsFlat;
            return toCameraFrame(surfPointsFlat);
        }
        pcl::PointCloud<PointType>::Ptr getSurfPointsLessFlat () {
            // ROS_INFO_STREAM("Return Surf points Less Flat " << surfPointsLessFlat->points.size());
            // return surfPointsLessFlat;
            return toCameraFrame(surfPointsLessFlat);
        }

        double getStamp() {
            return cloudHeader.stamp.toSec();
        }

        void readIntrinsicParameter(const std::string &calib_file) {
            if(DEBUG) ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
            m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
        }
        
        void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laser_msg) {
            
            copyPointCloud(laser_msg);
            // ROS_INFO_STREAM("Finish copy point cloud");
            findStartEndAngle();
            // ROS_INFO_STREAM("Finish find start end angle");
            projectPointCloud();
            // ROS_INFO_STREAM("Finish project point cloud");
            organizePointCloud();
            calculateSmoothness();
            // ROS_INFO_STREAM("Finish calculate smoothness");
            markOccludedPoints();
            // ROS_INFO_STREAM("Finish mark Occluded point");
            extractFeatures();
            // ROS_INFO_STREAM("Finish extract feature");
            // pointCloudToImage(laserCloudIn);
            resetParameters();

        }
};
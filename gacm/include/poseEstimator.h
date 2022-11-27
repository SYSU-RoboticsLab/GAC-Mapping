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

#include <mutex>
#include <queue>
#include <map>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>


#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "util/ip_basic.h"
#include "util/godec.h"
#include "util/jbf_filter.h"
#include "featureDefinition.h"
#include "lidarFactor.h"
// #include "errorFactors.h"
#include "parameters.h"

#include "ga_posegraph/poseFactor.h"

typedef pcl::PointXYZI PointType;

class PoseEstimator {
    
    private:
        camodocal::CameraPtr m_camera;
        std::vector<double> m_intrinsic_params;
        
        // image features
        cv::Mat rgb_cur, rgb_last;
        pcl::PointCloud<ImagePoint>::Ptr imagePointsCur, imagePointsLast;

        // full lidar cloud and enhanced depth map
        pcl::PointCloud<PointType>::Ptr laserCloudFullResCur;
        pcl::PointCloud<PointType>::Ptr laserCloudFullResLast;
        cv::Mat depth_map;
        cv::Mat laser_map;

        

        // lidar features
        pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        
        // KD Tree to search
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

        // from feature extractor
        std::queue <sensor_msgs::ImageConstPtr> imageRawBuf;
        std::queue <sensor_msgs::PointCloud2ConstPtr> featurePointsBuf;
        // from scan registrator
        std::queue <sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
        std::queue <sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
        std::queue <sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
        std::queue <sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
        std::queue <sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
        std::mutex mBuf; // beacause there is only one access thread , a single mutex is adequate
        
        // std::mutex mutex_pointmap;

        double timeImageRaw;
        double timeFeaturePoints;
        
        double timeCornerPointsSharp;
        double timeCornerPointsLessSharp;
        double timeSurfPointsFlat;
        double timeSurfPointsLessFlat;
        double timeLaserCloudFullRes;

        int skipFrameNum;
        int laserFrameCount;
        int laserFrameCountFromBegin;
        // int lastOptimizeFrame;
        int frameTobeMap;
        bool systemInited;
        bool need_pub_odom;
        bool need_pub_cloud;
        bool mapping_ready;
        bool lidar_only;

        int corner_correspondence;
        int plane_correspondence;
        int point_correspondence;

        int laserCloudCornerLastNum;
        int laserCloudSurfLastNum;

        int imagePointsLastNum, imagePointsCurNum;

        // Transformation from current frame to world frame
        Eigen::Quaterniond q_w_curr;// (1, 0, 0, 0);
        Eigen::Vector3d t_w_curr; // (0, 0, 0);
        Eigen::Quaterniond q_w_curr_hfreq;// (1, 0, 0, 0);
        Eigen::Vector3d t_w_curr_hfreq; // (0, 0, 0);
        


        Eigen::Matrix<double, 7, 1> se3_last_curr; // x, y, z, w,; x, y, z
        Eigen::Matrix<double, 7, 1> se3_last_curr_hfreq;  // image high frequency odometry 
        std::vector<PosePerFrame,Eigen::aligned_allocator<PosePerFrame>> poseSequence; // (frameid, pose) 
        std::deque<MappingDataPerFrame> mapDataPerFrameBuf; // buffer storing feature ids in each frame
        


        // global point feature map
        // std::map<int, PointFeaturePerId> globalFeatureMap;
        std::map<int, PointFeaturePerId, std::less<int>, Eigen::aligned_allocator<std::pair<const int, PointFeaturePerId>>> idPointFeatureMap;
       
        // for mapping node
        sensor_msgs::Image::Ptr rgb_image_ptr;
        sensor_msgs::Image::Ptr depth_image_ptr;
        geometry_msgs::PoseStamped mapping_pose;
        sensor_msgs::Image::Ptr depth_rgb_ptr;
        

        nav_msgs::Odometry laserOdometry;
        nav_msgs::Odometry laserOdometryHfreq;
        nav_msgs::Path laserPath; // path after odom
        nav_msgs::Path laserPath2;  // path after optimization
        

        sensor_msgs::PointCloud2 laserCloudCornerLastMsg;
        sensor_msgs::PointCloud2 laserCloudSurfLastMsg;
        sensor_msgs::PointCloud2 laserCloudFullResMsg;

        // visualization_msgs::Marker line3dMsg;
        sensor_msgs::PointCloud2 pointFeatureCloudMsg;
        
        
        void allocateMemory() {

            rgb_cur = cv::Mat(ROW, COL, CV_8UC3, cv::Scalar(0,0,0));
            rgb_last = rgb_cur.clone();
            depth_map = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));
            laser_map = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));
            
            imagePointsCur.reset(new pcl::PointCloud<ImagePoint>());
            imagePointsLast.reset(new pcl::PointCloud<ImagePoint>());


            laserCloudFullResCur.reset(new pcl::PointCloud<PointType>());
            laserCloudFullResLast.reset(new pcl::PointCloud<PointType>());

            cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
            cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
            surfPointsFlat.reset(new pcl::PointCloud<PointType>());
            surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

            laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());

            kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
            kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

            timeImageRaw = 0;
            timeFeaturePoints = 0;

            timeCornerPointsSharp = 0;
            timeCornerPointsLessSharp = 0;
            timeSurfPointsFlat = 0;
            timeSurfPointsLessFlat = 0;
            timeLaserCloudFullRes = 0;

            skipFrameNum = 1;
            laserFrameCount = 0;
            laserFrameCountFromBegin = 0;
            frameTobeMap = 0;
            // lastOptimizeFrame = 0;
            systemInited =false;
            need_pub_cloud = false;
            need_pub_odom = false;
            mapping_ready = false;

            corner_correspondence = 0;
            plane_correspondence = 0;
            point_correspondence = 0;

            laserCloudCornerLastNum = 0;
            laserCloudSurfLastNum = 0;

            imagePointsLastNum = 0;
            imagePointsCurNum = 0;
            
            Eigen::Vector3d q_v(0,0,0);

            q_w_curr.vec() = q_v;
            q_w_curr.w() = 1;
            t_w_curr << 0,0,0;

            q_w_curr_hfreq.vec() = q_v;
            q_w_curr_hfreq.w() = 1;
            t_w_curr_hfreq << 0,0,0;

            
            
        }

        void readIntrinsicParameter(const std::string &calib_file) {
            if(DEBUG) ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
            m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
            m_camera->writeParameters(m_intrinsic_params);
        }

        // undistort lidar point
        // Tranform points to the last frame (to find correspondence)
        void TransformToStart(PointType const *const pi, PointType *const po) {
            //interpolation ratio
            double s;
            if (DISTORTION)
                s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
            else
                s = 1.0;
            //s = 1;
            Eigen::Quaterniond q_last_curr(se3_last_curr.head(4).data());
            Eigen::Vector3d t_last_curr(se3_last_curr.tail(3).data());
            Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
            Eigen::Vector3d t_point_last = s * t_last_curr;
            Eigen::Vector3d point(pi->x, pi->y, pi->z);
            Eigen::Vector3d un_point = q_point_last * point + t_point_last;

            po->x = un_point.x();
            po->y = un_point.y();
            po->z = un_point.z();
            po->intensity = pi->intensity;
        }

        // transform all lidar points to the start of the next frame

        void TransformToEnd(PointType const *const pi, PointType *const po) {
            // undistort point first
            pcl::PointXYZI un_point_tmp;
            TransformToStart(pi, &un_point_tmp);

            Eigen::Quaterniond q_last_curr(se3_last_curr.head(4).data());
            Eigen::Vector3d t_last_curr(se3_last_curr.tail(3).data());

            Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
            Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

            po->x = point_end.x();
            po->y = point_end.y();
            po->z = point_end.z();

            //Remove distortion time info
            po->intensity = int(pi->intensity);
        }

        

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseEstimator(const std::string &calib_file) /*: q_last_curr(Eigen::Map<Eigen::Quaterniond>(NULL)), t_last_curr(Eigen::Map<Eigen::Vector3d>(NULL)) , se3_last_curr(Eigen::Map<Eigen::Matrix<double,6,1>>(NULL))*/ {
            allocateMemory();
            if(DEBUG) ROS_WARN_STREAM("finish allocate memory");
            readIntrinsicParameter(calib_file);
            if(DEBUG) ROS_WARN_STREAM("finish read intrinsic");
            se3_last_curr << 0,0,0,1, 0,0,0;
        }
        
        void imageHandler(const sensor_msgs::ImageConstPtr &image_msg) {
            mBuf.lock();
            imageRawBuf.push(image_msg);
            mBuf.unlock();
        }
        

        void featurePointsHandler(const sensor_msgs::PointCloud2ConstPtr &feature_points_msg) {
            mBuf.lock();
            featurePointsBuf.push(feature_points_msg);
            mBuf.unlock();
        }

        
        void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp) {
            mBuf.lock();
            cornerSharpBuf.push(corner_points_sharp);
            mBuf.unlock();
            // ROS_INFO_STREAM("Receive sharp ");
        }

        void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp) {
            mBuf.lock();
            cornerLessSharpBuf.push(corner_points_less_sharp);
            mBuf.unlock();
            // ROS_INFO_STREAM("Receive less sharp ");
        }

        void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_flat) {
            mBuf.lock();
            surfFlatBuf.push(surf_points_flat);
            mBuf.unlock();
            // ROS_INFO_STREAM("Receive flat");
        }

        void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_less_flat) {
            mBuf.lock();
            surfLessFlatBuf.push(surf_points_less_flat);
            mBuf.unlock();
            // ROS_INFO_STREAM("Receive less flat");
        }

        void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_fullres) {
            mBuf.lock();
            fullPointsBuf.push(laser_cloud_fullres);
            mBuf.unlock();
            // ROS_INFO_STREAM("Receive full res");
        }



        void fetchImageFromBUf() {
            mBuf.lock();
            
            cv_bridge::CvImageConstPtr ptr;
            ptr= cv_bridge::toCvCopy(*imageRawBuf.front(), sensor_msgs::image_encodings::RGB8);
            rgb_cur = ptr->image;
            imageRawBuf.pop();

            imagePointsCur->clear();
            pcl::fromROSMsg(*featurePointsBuf.front(), *imagePointsCur);
            imagePointsCurNum = imagePointsCur->size();
            featurePointsBuf.pop();

            mBuf.unlock();
        }

        void fetchAllFromBuf() {
            mBuf.lock();
            
            cv_bridge::CvImageConstPtr ptr;
            ptr= cv_bridge::toCvCopy(*imageRawBuf.front(), sensor_msgs::image_encodings::RGB8);
            rgb_cur = ptr->image;
            imageRawBuf.pop();

            imagePointsCur->clear();
            pcl::fromROSMsg(*featurePointsBuf.front(), *imagePointsCur);
            imagePointsCurNum = imagePointsCur->size();
            featurePointsBuf.pop();

            

            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullResCur->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullResCur);
            fullPointsBuf.pop();


            mBuf.unlock();

            // laserFrameCountFromBegin++;
        }




        // TODO: try finish in new thread
        void generateDepthMap() {
            depth_map = cv::Mat(ROW, COL, CV_16UC1, cv::Scalar(0));

            // TODO: project triangulate points to image
            // laser point project to image
            for (auto point: laserCloudFullResLast->points) {
                // Eigen::Vector3d P(-point.y, -point.z, point.x); // under camera coordinate
                Eigen::Vector3d P(point.x, point.y, point.z); // under camera coordinate
                Eigen::Vector2d p;
                m_camera->spaceToPlane(P,p);
                if (p[1] >0 && p[0] >0 && p[1]< ROW && p[0] < COL && P[2] > 0 && P[2] <= 65) {
                    // Scale: P[2] -> [0, 60] ushort->[0, 65535]
                    depth_map.at<ushort>(p[1], p[0]) = (ushort)(1000.0*P[2]);
                    // ROS_WARN_STREAM("##Check depth " << P[2] << " ; " << depth_map.at<ushort>(p[1], p[0]));
                }
            }
            // save laser depth image as mapping node input
            // laser_map = depth_map;

            // TODO: try other method
            // displayFalseColors(depth_map, "before diliate");
            // cv::waitKey(0);
            customDilate(depth_map,  depth_map, 7, KERNEL_TYPE_DIAMOND);
            // customDilate(depth_map, depth_map, 5, KERNEL_TYPE_DIAMOND);

            // displayFalseColors(depth_map, "falsecolor");
            cv::Mat bgr;
            generateFalseColors(depth_map, bgr);
            // cv::waitKey(5);
            cv_bridge::CvImage bridge;    
            bridge.image = bgr;
            bridge.encoding = sensor_msgs::image_encodings::BGR8;
            depth_rgb_ptr = bridge.toImageMsg();
            depth_rgb_ptr->header.stamp = ros::Time().fromSec(mapDataPerFrameBuf[0].timestamp);
        }

          
        void swap(int& a, int &b) {
            int temp = a;
            a = b;
            b = temp;
        }

        

        // TODO: try finish in new thread
        void updatePointFeatureMap(bool highfreq = false) {
             for (ImagePoint point : imagePointsLast->points) {
                auto it = idPointFeatureMap.find(point.index);
                Eigen::Matrix<double, 3, 1> eigen_point;
                double inverse_depth = 0;
                unsigned char depth_type = 0;
                if (!highfreq) {
                    double depth = depth_map.at<ushort>(point.v, point.u);
                    depth /= 1000.0;
                    depth_type = depth >0? 1:0; // 0 no depth 1 from lidar 2 from triangulation
                    if (depth > 0) {
                        inverse_depth = 1.0/depth;
                    }
                }
                 eigen_point << inverse_depth , point.u , point.v;
                
                // if (!highfreq) {
                // record feature in each frame for mapping use
                mapDataPerFrameBuf[laserFrameCountFromBegin-frameTobeMap].point_feature_ids.push_back(point.index);
                // }
                
                if (it != idPointFeatureMap.end()) {
                    // ROS_WARN_STREAM("###Check feature track " << it->second.endFrame() << " now " << laserFrameCountFromBegin);
                    it->second.feature_per_frame.push_back(PointFeaturePerFrame(eigen_point, depth_type, laserFrameCountFromBegin));
                    it->second.setDepthType(depth_type); // TODO: maybe not correct
                    // it->second.setDepth(depth);
                } else {
                    PointFeaturePerId pfpid(point.index, laserFrameCountFromBegin); // create a new structure with (featureid, startfreame)
                    pfpid.feature_per_frame.push_back(PointFeaturePerFrame(eigen_point, depth_type, laserFrameCountFromBegin));
                    pfpid.setDepthType(depth_type);
                    // pfpid.setInitialEstimation(0); // TODO: maybe useless
                    // pfpid.setDepth(depth);
                    idPointFeatureMap.insert(std::pair<int, PointFeaturePerId> (point.index, pfpid));

                }

            }

        }
       

        void optimizeMap(bool lidar_only) {
            // ROS_WARN_STREAM("### Begin OptimizeMap ###");
            // if (laserFrameCountFromBegin < 5) return;
            int win_size =  7;
            int keep_constant_len = 0;
            int min_track_len = 2; 
            ceres::Problem problem;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::LocalParameterization *t_parameterization = new ceres::IdentityParameterization(3);
            ceres::LocalParameterization *pose_parameterization = new ceres::ProductParameterization(q_parameterization, t_parameterization);
            for (size_t i = 0; i < poseSequence.size(); i++) {
                problem.AddParameterBlock(poseSequence[i].pose.data(),7, pose_parameterization);
                // problem.AddParameterBlock(poseSequence[i].q.coeffs().data(),4,q_parameterization);
                // problem.AddParameterBlock(poseSequence[i].t.data(),3);
                if (i > 0) {
                    Eigen::Matrix<double,6,6> sqrt = Eigen::Matrix<double,6,6>::Identity();
                    
                    ceres::CostFunction *cost_function = PoseGraph3dErrorTerm2::Create(poseSequence[i].rel, sqrt*100);
                    problem.AddResidualBlock(cost_function, loss_function, poseSequence[i-1].pose.data(),poseSequence[i].pose.data());
                }
                if (lidar_only) {
                    problem.SetParameterBlockConstant(poseSequence[i].pose.data());
                    continue;
                }
                if (i <= 5 || i < laserFrameCountFromBegin-win_size) { // set the first frame to be fix
                    problem.SetParameterBlockConstant(poseSequence[i].pose.data());
                }

            }
            
            




            

            // add residual block for feature point
            for (auto it = idPointFeatureMap.begin(); it!=idPointFeatureMap.end(); it++) {
                

                if (it->second.depth_type == 0 ||  it->second.endFrame() < laserFrameCountFromBegin-win_size) {
                    continue;
                }

                

                size_t trackcount = it->second.feature_per_frame.size();
                if (trackcount < min_track_len) { 
                    continue;
                }

                problem.AddParameterBlock(it->second.inverse_depth, 1);

                int start_frame = it->second.startFrame();
                int depth_frame = it->second.depthFrame();
                int depth_frame_index = it->second.depthFrameIndex();
                Eigen::Vector2d p_first_depth = it->second.feature_per_frame[depth_frame_index].uv;
                Eigen::Vector3d P_first_depth;
                Eigen::Quaterniond q_first_depth(poseSequence[depth_frame].pose.head(4).data());
                Eigen::Vector3d t_first_depth(poseSequence[depth_frame].pose.tail(3).data());
                m_camera->liftProjective(p_first_depth, P_first_depth);

                
                for (size_t j = 0; j < trackcount; j++) {
                    
                    int frame_id = it->second.getFrameIdAt(j);
                    if (frame_id == depth_frame) {
                        continue;
                    }
                    Eigen::Vector2d cp;
                    cp = it->second.feature_per_frame[j].uv;
                    ceres::CostFunction *cost_function = ReprojectionPoint32Factor3new::Create(m_intrinsic_params, cp , P_first_depth);
                    problem.AddResidualBlock(cost_function, loss_function,
                                    poseSequence[depth_frame].pose.data(),
                                    poseSequence[frame_id].pose.data(),
                                    //it->second.estimated_depth
                                    it->second.inverse_depth);
                                    // it->second.estimated_point.data());
                }
            }




            ceres::Solver::Options options;
            options.gradient_tolerance = 1e-16;
            options.function_tolerance = 1e-16;
            options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
            // options.linear_solver_type = ceres::SPARSE_SCHUR;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            // options.minimizer_type = ceres::TRUST_REGION;
            // options.trust_region_strategy_type = ceres::DOGLEG;
            // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
            options.dynamic_sparsity = true;
            options.minimizer_progress_to_stdout = false;
            options.num_threads = 2;
            options.max_num_iterations = 3;
            
            ceres::Solver::Summary summary;
            // ROS_WARN_STREAM("### Begin BA ###");
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << "\n";

            

        }

        // check outliers    
        void updateLandmark() {
            for (auto it = idPointFeatureMap.begin(); it!=idPointFeatureMap.end();) {

                if (it->second.isOutlier()) {
                    bool is_active = it->second.endFrame() >= laserFrameCountFromBegin - 7;
                    
                    bool end_track = it->second.endFrame() < laserFrameCountFromBegin;
                    if (end_track && it->second.feature_per_frame.size() <5) {
                        it = idPointFeatureMap.erase(it);
                    }

                    else if (is_active) {
                        bool next_depth_sucess = it->second.useNextDepth();
                        if (next_depth_sucess) {
                            it++;
                        } else {
                            it = idPointFeatureMap.erase(it);
                        }
                    } else {
                        // TODO:  maginalization
                        it = idPointFeatureMap.erase(it);
                    }
                } else {
                    // not a outlier
                    it++;
                }
            }

            
        }

        void fetchAllTimeStamp() {
            timeImageRaw = imageRawBuf.front()->header.stamp.toSec();
            timeFeaturePoints = featurePointsBuf.front()->header.stamp.toSec();
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
        }

        void fetchImageTimeStamp() {
            timeImageRaw = imageRawBuf.front()->header.stamp.toSec();
            timeFeaturePoints = featurePointsBuf.front()->header.stamp.toSec();
        }

        void saveHistoryMessage(bool highfreq = false) {

            // image feature and picture
            rgb_last = rgb_cur.clone();
            pcl::PointCloud<ImagePoint>::Ptr imagePointsTemp = imagePointsCur;
            imagePointsCur = imagePointsLast;
            imagePointsLast = imagePointsTemp;

            // feature number
            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();
            imagePointsLastNum = imagePointsCurNum;

            if (highfreq) return;

            // laser feature and cloud
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudTemp = laserCloudFullResLast;
            laserCloudFullResLast = laserCloudFullResCur;
            laserCloudFullResCur = laserCloudTemp;
            
            // laser features set into kd tree
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }

        void generateOdomMessage(bool highfreq = false) {

            // ROS_INFO_STREAM("Generating message");
            if (highfreq) {
                laserOdometry.header.frame_id = "/camera"; // need check
                laserOdometry.child_frame_id = "/laser_odom";
                laserOdometry.header.stamp = ros::Time().fromSec(timeImageRaw);
                laserOdometry.pose.pose.orientation.x = q_w_curr_hfreq.x();
                laserOdometry.pose.pose.orientation.y = q_w_curr_hfreq.y();
                laserOdometry.pose.pose.orientation.z = q_w_curr_hfreq.z();
                laserOdometry.pose.pose.orientation.w = q_w_curr_hfreq.w();
                laserOdometry.pose.pose.position.x = t_w_curr_hfreq.x();
                laserOdometry.pose.pose.position.y = t_w_curr_hfreq.y();
                laserOdometry.pose.pose.position.z = t_w_curr_hfreq.z();
            } else {
                // generate odometry message and add to path message
                laserOdometry.header.frame_id = "/camera"; // need check
                laserOdometry.child_frame_id = "/laser_odom";
                laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                // ROS_ERROR_STREAM("Generate odom " << ros::Time().fromSec(timeSurfPointsLessFlat).toNSec());
                laserOdometry.pose.pose.orientation.x = q_w_curr.x();
                laserOdometry.pose.pose.orientation.y = q_w_curr.y();
                laserOdometry.pose.pose.orientation.z = q_w_curr.z();
                laserOdometry.pose.pose.orientation.w = q_w_curr.w();
                laserOdometry.pose.pose.position.x = t_w_curr.x();
                laserOdometry.pose.pose.position.y = t_w_curr.y();
                laserOdometry.pose.pose.position.z = t_w_curr.z();
                // need_pub_odom = true;
            }


            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera"; // need check
        }

        void updatePoseSequence(bool highfreq = false) {
            Eigen::Matrix<double,7,1> temp_pose;
            // Eigen::Matrix<double,7,1> temp_rel;
            if (highfreq) {
                temp_pose <<  q_w_curr_hfreq.x(), q_w_curr_hfreq.y(), q_w_curr_hfreq.z(), q_w_curr_hfreq.w() , t_w_curr_hfreq.x(), t_w_curr_hfreq.y(), t_w_curr_hfreq.z();
                // temp_rel = se3_last_curr;
            } else {
                temp_pose <<  q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w() , t_w_curr.x(), t_w_curr.y(), t_w_curr.z();
                // temp_rel = se3_last_curr;
            }
            poseSequence.push_back(PosePerFrame(laserFrameCountFromBegin, temp_pose, se3_last_curr));
        }

        void generateLaserMessage() {
            if (laserFrameCount % skipFrameNum == 0) {
                laserFrameCount = 0;
                need_pub_cloud = true;
                need_pub_odom = true;

                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLastMsg);
                laserCloudCornerLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLastMsg.header.frame_id = "/camera";
                // pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLastMsg);
                laserCloudSurfLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLastMsg.header.frame_id = "/camera";
                // pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                
                pcl::toROSMsg(*laserCloudFullResCur, laserCloudFullResMsg);
                laserCloudFullResMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullResMsg.header.frame_id = "/camera";
                // pubLaserCloudFullRes.publish(laserCloudFullRes3);
            } else {
                need_pub_cloud = false;
                need_pub_odom = false;
            }
            laserFrameCount++;
        }
        
        bool checkAllMessageSynced() {
            if (fabs(timeImageRaw - timeLaserCloudFullRes) > 0.03 ||
                fabs(timeFeaturePoints - timeLaserCloudFullRes) > 0.03 ||
                

                fabs(timeCornerPointsSharp - timeLaserCloudFullRes)>0.01 ||
                fabs(timeCornerPointsLessSharp - timeLaserCloudFullRes)>0.01 ||
                fabs(timeSurfPointsFlat - timeLaserCloudFullRes)>0.01 ||
                fabs(timeSurfPointsLessFlat - timeLaserCloudFullRes)>0.01) {

                // ROS_WARN_STREAM("\ntimeImageRaw " << std::setprecision(10) << timeImageRaw*100000 << "\n"<<
                // "timeFeaturePoints " << timeFeaturePoints*100000 << "\n"<<
                // "timeLaserCloudFullRes " << timeLaserCloudFullRes*100000 << "\n"
                // << timeImageRaw - timeLaserCloudFullRes << "\n" 
                // << timeFeaturePoints - timeLaserCloudFullRes << "\n" 
                // << timeCornerPointsSharp - timeLaserCloudFullRes<< "\n"
                // << timeCornerPointsLessSharp - timeLaserCloudFullRes<< "\n"
                // << timeSurfPointsFlat - timeLaserCloudFullRes<< "\n"
                // << timeSurfPointsLessFlat - timeLaserCloudFullRes<< "\n");
                ROS_WARN_STREAM("Message unsynced");
                return false;
            }
            return true;
        }

        void estimatePoseWithImage() {
            fetchImageFromBUf();

            point_correspondence = 0;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::LocalParameterization *t_parameterization = new ceres::IdentityParameterization(3);
            ceres::LocalParameterization *pose_parameterization = new ceres::ProductParameterization(q_parameterization, t_parameterization);
            ceres::Problem problem;

            problem.AddParameterBlock(se3_last_curr_hfreq.data(),7,pose_parameterization);

            // find correspondence for each cur point features 
            if (imagePointsCurNum > 10) {
                for (int i = 0; i < imagePointsCurNum; ++i) {
                    
                    ImagePoint pointSelCur = imagePointsCur->points[i];
                    Eigen::Vector3d lP;
                    Eigen::Quaterniond lq(poseSequence[laserFrameCountFromBegin].pose.head(4).data()); // 最后一帧是雷达&图像帧
                    Eigen::Vector3d lt(poseSequence[laserFrameCountFromBegin].pose.tail(3).data());
                    
                    bool is_found = false;
                    // for (int j = 0; j < imagePointsLastNum; ++j) {
                    auto it = idPointFeatureMap.find(pointSelCur.index);
                    if (it != idPointFeatureMap.end() && it->second.depth_type > 0) {
                    // if (pointSelCur.index == imagePointsLast->points[j].index) {
                        // pointSelLast = imagePointsLast->points[i];
                        // int sf = it->second.startFrame();
                        int ef = it->second.endFrame();
                        int df = it->second.depthFrame();
                        int dfi = it->second.depthFrameIndex();
                        int ndi = it->second.nextDepthIndex();
                        
                        double d = it->second.inverse_depth[0];
                        if (d > 0) {
                            d = 1.0/d;
                        } else {
                            d = 0;
                        }
                        if (ef-df <= 60 && d >0) { 
                            //TODO: is it necessary?
                            Eigen::Quaterniond dq(poseSequence[df].pose.head(4).data());
                            Eigen::Vector3d dt(poseSequence[df].pose.tail(3).data());
                            Eigen::Vector2d duv = it->second.feature_per_frame[dfi].uv;
                            Eigen::Vector3d dP;
                            m_camera->liftProjective(duv, dP);
                            dP *= d;
                            dP = dq*dP+dt; // transform to world frame
                            lP = lq.inverse()*(dP-lt); // transform to last camera frame
                        } else {
                            lP = Eigen::Vector3d::Zero();
                            // lP = it->second.feature_per_frame[ef-sf].point;
                        }
                        
                        
                        if (lP[2] > 0 && lP[2] < 100) {
                            is_found = true;
                        }
                        // break;
                        // }
                    }
                    // }

                    if (is_found) {
                        // 3d-2d case
                        // ROS_WARN_STREAM("Found point correspondence");
                        // TODO: need check the depth unit
                        // if (depth_map.at<ushort>(pointSel.v, pointSel.u) > 0 && depth_map.at<ushort>(pointSel.v, pointSel.u) < 45) {
                            // TODO: depthflag?
                            // Eigen::Vector3d lP;
                            // Eigen::Vector2d lp;
                        Eigen::Vector2d cp;
                        // lp(0) = pointSelLast.u;
                        // lp(1) = pointSelLast.v;
                        cp(0) = pointSelCur.u;
                        cp(1) = pointSelCur.v;
                        // Eigen::Vector3d cP;
                        // m_camera->liftProjective(cp, cP);
                        // cP *= depth_map.at<ushort>(cp(0), cp(1));
                        ceres::CostFunction *cost_function = ReprojectionPoint32Factor1::Create(m_intrinsic_params, lP, cp);
                        problem.AddResidualBlock(cost_function, loss_function, se3_last_curr_hfreq.data());
                        point_correspondence++;
                        // } 
                    }
                    
                }
            } else {
                if(DEBUG) ROS_INFO_STREAM("Few point feature. ignore!");
            }
            
            // ROS_WARN_STREAM("IOPE correspondence " << point_correspondence);
            
            if (point_correspondence > 10) {

                // solve ceres problem
                // ROS_INFO_STREAM("Begin slove pose");
                ceres::Solver::Options options;
                
                
                options.dynamic_sparsity = true;
                options.max_num_iterations = 8;
                options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                
                

                
                
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                if(DEBUG) std::cout << summary.BriefReport() << "\n";

                // ROS_INFO_STREAM("Begin update pose high freq");
                
                Eigen::Quaterniond q_last_curr(se3_last_curr_hfreq.head(4).data());
                Eigen::Vector3d t_last_curr(se3_last_curr_hfreq.tail(3).data());
                Eigen::Quaterniond last_q(poseSequence[poseSequence.size()-1].pose.head(4).data()); // 最后一帧是纯图像帧
                Eigen::Vector3d last_t(poseSequence[poseSequence.size()-1].pose.tail(3).data());
                t_w_curr_hfreq = last_t + last_q * t_last_curr;
                q_w_curr_hfreq = q_last_curr * last_q;
            }

            // generate odometry message to be published
            generateOdomMessage(true);

            // save history result
            saveHistoryMessage(true);

            // update posegraph
            updatePoseSequence(true);

            mapDataPerFrameBuf.push_back(MappingDataPerFrame(laserFrameCountFromBegin, timeSurfPointsLessFlat, rgb_last, depth_map));

            updatePointFeatureMap(true);

            

            q_w_curr_hfreq = Eigen::Quaterniond(poseSequence.back().pose.head(4).data());
            t_w_curr_hfreq = Eigen::Vector3d(poseSequence.back().pose.tail(3).data());

            // check outliers after optimization
            // updateLandmark();

            // generateLaserMessage();

            laserFrameCountFromBegin++;
        } 

        void estimatePoseWithImageAndLaser(bool need_update = true) {
            
            fetchAllFromBuf();

            clock_t start, end;
            start = clock();
            // generate depth image
            generateDepthMap();
            end = clock();
            ofstream outfile;
            outfile.open
            (
                (   std::string(std::getenv("HOME")) + 
                    std::string("/gacm_output/timecost/depth_completion_time.txt")).c_str(), 
                ios::app
            );
            outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
            outfile.close();

            start = clock();
            if (need_update) {
                
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();
        
                for (size_t opti_counter = 0; opti_counter < 8; ++opti_counter) {
                    corner_correspondence = 0;
                    plane_correspondence = 0;
                    point_correspondence = 0;

                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
                    ceres::LocalParameterization *t_parameterization = new ceres::IdentityParameterization(3);
                    ceres::Problem::Options problem_options;
                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(se3_last_curr.head(4).data(), 4, q_parameterization);
                    problem.AddParameterBlock(se3_last_curr.tail(3).data(), 3);
                    


                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;


                    
                    
                    // find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; ++i) {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j) {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2) {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j) {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2) {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }

                            // both closestPointInd and minPointInd2 is valid
                        if (minPointInd2 >= 0) {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                    cornerPointsSharp->points[i].y,
                                                    cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                        laserCloudCornerLast->points[closestPointInd].y,
                                                        laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                        laserCloudCornerLast->points[minPointInd2].y,
                                                        laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, se3_last_curr.head(4).data(), se3_last_curr.tail(3).data());
                            
                            corner_correspondence++;
                        }
                    }// find correspondence for corner features

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i) {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j) {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j) {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0) {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, se3_last_curr.head(4).data(), se3_last_curr.tail(3).data());
                                plane_correspondence++;
                            }
                        }
                    } // find correspondence for plane
                    
                    lidar_only = true; // already tested, scale weird if set false
                    float visual_weight = std::min(150.0/(corner_correspondence + plane_correspondence), (double)VISUAL_WEIGHT_MAX);
                    if ((corner_correspondence + plane_correspondence) < 20) {
                        printf("less  lidar correspondence! *************************************************\n");
                        // lidar_only = false;
                    } // test
                        // find correspondence for each cur point features 
                        if (imagePointsCurNum > 10) {
                            for (int i = 0; i < imagePointsCurNum; ++i) {
                                
                                ImagePoint pointSelCur = imagePointsCur->points[i];
                                // ImagePoint pointSelLast;// = imagePointsLast->points[i];
                                Eigen::Vector3d lP;
                                Eigen::Vector3d dP;
                                Eigen::Quaterniond lq(poseSequence.back().pose.head(4).data());
                                Eigen::Vector3d lt(poseSequence.back().pose.tail(3).data());
                                bool is_found = false;
                                // for (int j = 0; j < imagePointsLastNum; ++j) {
                                    auto it = idPointFeatureMap.find(pointSelCur.index);
                                    if (it != idPointFeatureMap.end() && it->second.depth_type > 0 && it->second.feature_per_frame.size()>3) {
                                    // if (pointSelCur.index == imagePointsLast->points[j].index) {
                                        int ef = it->second.endFrame();
                                        int df = it->second.depthFrame();
                                        int dfi = it->second.depthFrameIndex();
                                        int ndi = it->second.nextDepthIndex();
                                        
                                        double d = it->second.inverse_depth[0];
                                        if (d > 0) {
                                            d = 1.0/d;
                                        } else {
                                            d = 0;
                                        }
                                            
                                        if (/*ef-df <= 60 &&*/ d > 2 && d < 60) { 
                                            //TODO: is it necessary?
                                            Eigen::Quaterniond dq(poseSequence[df].pose.head(4).data());
                                            Eigen::Vector3d dt(poseSequence[df].pose.tail(3).data());
                                            Eigen::Vector2d duv = it->second.feature_per_frame[dfi].uv;
                                            m_camera->liftProjective(duv, dP);
                                            dP *= d;
                                            dP = dq*dP+dt; // transform to world frame
                                            lP = lq.inverse()*(dP-lt); // transform to camera frame
                                            // lP = d>0 ? lq.inverse()*(dP-lt) : Eigen::Vector3d::Zero(); // transform to camera frame
                                        } else if (ndi > 0) {

                                            int ndf = it->second.feature_per_frame[ndi].frame_id;
                                            Eigen::Quaterniond ndq(poseSequence[ndf].pose.head(4).data());
                                            Eigen::Vector3d ndt(poseSequence[ndf].pose.tail(3).data());
                                            Eigen::Vector2d nduv = it->second.feature_per_frame[ndi].uv;
                                            m_camera->liftProjective(nduv, dP);
                                            d = it->second.feature_per_frame[ndi].inverse_depth;
                                            if (d > 0) {
                                                d = 1.0/d;
                                            } else {
                                                d = 0;
                                            }

                                            dP *= d;
                                            dP = ndq*dP+ndt; // transform to world frame
                                            lP = (d > 0 && d < 60) ? lq.inverse()*(dP-lt) : Eigen::Vector3d::Zero(); // transform to camera frame
                                             
                                        } else {
                                            lP = Eigen::Vector3d::Zero();
                                            // lP = it->second.feature_per_frame[ef-sf].point;
                                        }
                                        
                                        if (lP[2] > 0 && lP[2] < 60) {
                                            is_found = true;
                                        } else {
                                            // ROS_ERROR_STREAM("check point " << lP.transpose() << " ; " << d << " ; " << dP.transpose());
                                        }
                                        // break;
                                        // }
                                    }

                                if (is_found) {
                                    // 3d-2d case
                                    
                                        Eigen::Vector2d cp;
                                        cp(0) = pointSelCur.u;
                                        cp(1) = pointSelCur.v;
                                        Eigen::Vector3d cP;
                                        m_camera->liftProjective(cp, cP);
                                            std::vector<double> m_intrinsic_params;
                                            m_camera->writeParameters(m_intrinsic_params);

                                            ceres::CostFunction *cost_function = PointPosition33FactorQT::Create(lP, cP, visual_weight);
                                            problem.AddResidualBlock(cost_function, loss_function, se3_last_curr.head(4).data(), se3_last_curr.tail(3).data());
                                            point_correspondence++;

                                }
                                
                            }
                        } else {
                            if(DEBUG) ROS_INFO_STREAM("Few point feature. ignore!");
                        }
                    //} // end lidar only


                    if ((corner_correspondence + plane_correspondence + point_correspondence) < 10) {
                        printf("less correspondence! *************************************************\n");
                    } else {
                        
                    }
                    
                    
                    // solve ceres problem
                    // ROS_INFO_STREAM("Begin slove pose");
                    ceres::Solver::Options options;
                    
                    
                    options.dynamic_sparsity = true;
                    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                    // options.linear_solver_type = ceres::DENSE_QR;
                    // options.trust_region_strategy_type = ceres::DOGLEG;
                    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
                    options.max_num_iterations = 8;
                    options.minimizer_progress_to_stdout = false;

                    

                    
                    
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    // std::cout << summary.BriefReport() << "\n";

                } // optimization loop



                Eigen::Quaterniond q_last_curr(se3_last_curr.head(4).data());
                Eigen::Vector3d t_last_curr(se3_last_curr.tail(3).data());
                Eigen::Quaterniond last_q(poseSequence[poseSequence.size()-1].pose.head(4).data()); 
                Eigen::Vector3d last_t(poseSequence[poseSequence.size()-1].pose.tail(3).data());
                t_w_curr = last_t + last_q * t_last_curr;
                q_w_curr = last_q*q_last_curr;

                // PoseType temp_pose = PoseType(t_w_curr, q_w_curr);
                    
            
            }

            end = clock();
            std::string home_dir = std::getenv("HOME");
            outfile.open(
                (home_dir + std::string("/gacm_output/timecost/pose_estimate_time.txt")).c_str(), ios::app
            );
            outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
            outfile.close();

            // generate odometry message to be published
            generateOdomMessage();

            // save history result
            saveHistoryMessage();

            // update posegraph
            updatePoseSequence();


            mapDataPerFrameBuf.push_back(MappingDataPerFrame(laserFrameCountFromBegin, timeSurfPointsLessFlat, rgb_last, depth_map));

            updatePointFeatureMap();

            start = clock();
            
            optimizeMap(lidar_only);

            end = clock();
            outfile.open(
                (home_dir + std::string("/gacm_output/timecost/landmark_time.txt")).c_str(), ios::app
            );
            outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
            outfile.close();

            q_w_curr_hfreq = q_w_curr;
            t_w_curr_hfreq = t_w_curr;

            // check outliers after optimization
            updateLandmark();

            generateLaserMessage();

            laserFrameCountFromBegin++;
        }


        void handleImage() {
            need_pub_odom = false;
            if (!systemInited) {
                // should wait for the first laser frame and image frame together
                
                // if all buf have message
                if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
                    !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty() &&
                    !imageRawBuf.empty() && !featurePointsBuf.empty()) { 
                    
                    if(DEBUG) ROS_ERROR_STREAM("Try initialization");
                    // fetch timestamp from messages
                    fetchAllTimeStamp();

                    // if message not synced
                    if (!checkAllMessageSynced()) {
                        // drop the first few messages
                        if (timeImageRaw < timeLaserCloudFullRes) {
                            mBuf.lock();
                            imageRawBuf.pop();
                            featurePointsBuf.pop();
                            mBuf.unlock();
                        } else if (timeImageRaw > timeLaserCloudFullRes) {
                            mBuf.lock();
                            cornerSharpBuf.pop();
                            cornerLessSharpBuf.pop();
                            surfFlatBuf.pop();
                            surfLessFlatBuf.pop();
                            fullPointsBuf.pop();
                            mBuf.unlock();
                        }
                        return; // time not synced wait for next loop
                    }
                    estimatePoseWithImageAndLaser(false);

                    systemInited = true;
                    need_pub_odom = true;
                    if(DEBUG) ROS_ERROR_STREAM("Initialization finished");
                } // end if all buf have message



                
            } // end if system not initialized

            else {
                // system is initialized, do pose estimation

                if (!imageRawBuf.empty() && !featurePointsBuf.empty()) {
                    if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
                        !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty()) {
                        // fetch timestamp from messages
                        fetchAllTimeStamp();
                        // check synced
                        if (!checkAllMessageSynced()) {
                            if (timeImageRaw < timeLaserCloudFullRes) {
                                mBuf.lock();
                                imageRawBuf.pop();
                                featurePointsBuf.pop();
                                mBuf.unlock();
                            } else if (timeImageRaw > timeLaserCloudFullRes) {
                                mBuf.lock();
                                cornerSharpBuf.pop();
                                cornerLessSharpBuf.pop();
                                surfFlatBuf.pop();
                                surfLessFlatBuf.pop();
                                fullPointsBuf.pop();
                                mBuf.unlock();
                            }

                        } else {
                            // pose estimation using laser & image
                            estimatePoseWithImageAndLaser(true);
                            // ROS_ERROR_STREAM("estimatePoseWithImageAndLaser");
                            need_pub_odom = true;
                        }
                    } 
                } // end if image not empty
            }
        }
       



        const nav_msgs::Odometry & getLaserOdometry() {

            return laserOdometry;
        }
        const nav_msgs::Path & getLaserPath() {
            return laserPath;
        }
        const nav_msgs::Path & getLaserPath2() {
            laserPath2.poses.clear();
            for (size_t i =0 ; i < poseSequence.size(); i++) {
                geometry_msgs::PoseStamped laserPose2;
                
                Eigen::Quaterniond q(poseSequence[i].pose.head(4).data());
                Eigen::Vector3d t(poseSequence[i].pose.tail(3).data());
                

                laserPose2.header = laserPath.poses[i].header;
                laserPose2.pose.orientation.x = q.x();
                laserPose2.pose.orientation.y = q.y();
                laserPose2.pose.orientation.z = q.z();
                laserPose2.pose.orientation.w = q.w();
                laserPose2.pose.position.x = t.x();
                laserPose2.pose.position.y = t.y();
                laserPose2.pose.position.z = t.z();
                // laserPose2.header.stamp = laserPath.poses[i].header.stamp;
                // laserPose2.header = laserOdometry.header;
                // laserPose2.header.child_frame_id = "/laser_odom";
                laserPath2.header.stamp = laserOdometry.header.stamp;
                laserPath2.poses.push_back(laserPose2);
                laserPath2.header.frame_id = "/camera";
            }

            return laserPath2;
        }

        

        const sensor_msgs::PointCloud2 & getMapPointCloud() {
            // sensor_msgs::PointCloud2 points;//(new sensor_msgs::PointCloud2);
            pointFeatureCloudMsg.header.stamp = ros::Time().fromSec(timeImageRaw);
            pointFeatureCloudMsg.header.frame_id = "/camera";

            pointFeatureCloudMsg.data.clear();

            pointFeatureCloudMsg.height = 1;
            pointFeatureCloudMsg.width = idPointFeatureMap.size();
            pointFeatureCloudMsg.fields.resize(4);
            pointFeatureCloudMsg.fields[0].name = "x";
            pointFeatureCloudMsg.fields[0].offset = 0;
            pointFeatureCloudMsg.fields[0].count = 1;
            pointFeatureCloudMsg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            pointFeatureCloudMsg.fields[1].name = "y";
            pointFeatureCloudMsg.fields[1].offset = 4;
            pointFeatureCloudMsg.fields[1].count = 1;
            pointFeatureCloudMsg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            pointFeatureCloudMsg.fields[2].name = "z";
            pointFeatureCloudMsg.fields[2].offset = 8;
            pointFeatureCloudMsg.fields[2].count = 1;
            pointFeatureCloudMsg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            pointFeatureCloudMsg.fields[3].name = "rgb";
            pointFeatureCloudMsg.fields[3].offset = 12;
            pointFeatureCloudMsg.fields[3].count = 1;
            pointFeatureCloudMsg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            //pointFeatureCloudMsg.is_bigendian = false; ???
            pointFeatureCloudMsg.point_step = 16;
            pointFeatureCloudMsg.row_step = pointFeatureCloudMsg.point_step * pointFeatureCloudMsg.width;
            pointFeatureCloudMsg.data.resize(pointFeatureCloudMsg.row_step * pointFeatureCloudMsg.height);
            pointFeatureCloudMsg.is_dense = false; // there may be invalid points

            float bad_point = std::numeric_limits<float>::quiet_NaN(); // for invalid part
            int counter = 0;
            for (auto it = idPointFeatureMap.begin(); it != idPointFeatureMap.end(); it++, counter++) {
                if (it->second.depth_type>0) {
                    int32_t rgb = ((uint8_t)78 << 16) | ((uint8_t)238 << 8) | (uint8_t)148;
                    double d = it->second.inverse_depth[0];
                    if (d > 0) {
                        d = 1.0/d;
                    } else {
                        d = 0;
                        continue;
                    }
                    int df = it->second.depthFrame();
                    int dfi = it->second.depthFrameIndex();
                    Eigen::Quaterniond dq(poseSequence[df].pose.head(4).data());
                    Eigen::Vector3d dt(poseSequence[df].pose.tail(3).data());
                    Eigen::Vector2d duv = it->second.feature_per_frame[dfi].uv;
                    Eigen::Vector3d dP;
                    m_camera->liftProjective(duv, dP);
                    dP *= d;
                    dP = dq*dP+dt;
                    float x = (float)dP(0);
                    float y = (float)dP(1);
                    float z = (float)dP(2);
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 0], &x, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 4], &y, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 8], &z, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 12], &rgb, sizeof(int32_t));
                } else {
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 0], &bad_point, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 4], &bad_point, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 8], &bad_point, sizeof(float));
                    memcpy(&pointFeatureCloudMsg.data[counter * pointFeatureCloudMsg.point_step + 12], &bad_point, sizeof(float));
                }
            }

            return pointFeatureCloudMsg;
        }

        const  sensor_msgs::PointCloud2 & getLaserCloudCornerLastMsg() {
            return laserCloudCornerLastMsg;
        }
        const  sensor_msgs::PointCloud2 & getLaserCloudSurfLastMsg() {
            return laserCloudSurfLastMsg;
        }
        const  sensor_msgs::PointCloud2 & getLaserCloudFullResMsg() {
            return laserCloudFullResMsg;
        }
        const bool cloudNeedPub() {
            return need_pub_cloud;
        }
        const bool odomNeedPub() {
            // bool temp = need_pub_odom;
            // if (temp){
            //     need_pub_odom = false;
            // }
            return need_pub_odom;
        }

        const bool mappingReady() {
            return mapping_ready;
        }
        const sensor_msgs::Image::Ptr & getRGBImage() {
            return rgb_image_ptr;
        }
        const sensor_msgs::Image::Ptr & getDepthImage() {
            return depth_image_ptr;
        }
        const geometry_msgs::PoseStamped & getMappingPose() {
            return mapping_pose;
        }
        const sensor_msgs::Image::Ptr & getDepthImageRGB() {
            return depth_rgb_ptr;
        }

        void publishMappingTopics() {
            // mapping buffer size bigger than pose optimization window size
            if (mapDataPerFrameBuf.size() > 10) {
                // ROS_ERROR_STREAM("#########MappingTopics###########");
                cv_bridge::CvImage bridge;
                
                bridge.image = mapDataPerFrameBuf[0].rgb_image;
                bridge.encoding = sensor_msgs::image_encodings::RGB8;
                rgb_image_ptr = bridge.toImageMsg();
                rgb_image_ptr->header.stamp = ros::Time().fromSec(mapDataPerFrameBuf[0].timestamp);
                int mf = mapDataPerFrameBuf[0].frame_id;
                Eigen::Quaterniond q_mf(poseSequence[mf].pose.head(4).data());
                Eigen::Vector3d t_mf(poseSequence[mf].pose.tail(3).data());

                cv::Mat depthmap = mapDataPerFrameBuf[0].depth_image;
                

                bridge.image = depthmap;
                bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                depth_image_ptr = bridge.toImageMsg();
                depth_image_ptr->header.stamp = rgb_image_ptr->header.stamp;
                
                mapping_pose.header.frame_id = "/camera"; 
                mapping_pose.header.stamp = rgb_image_ptr->header.stamp;
                mapping_pose.pose.position.x = t_mf(0);
                mapping_pose.pose.position.y = t_mf(1);
                mapping_pose.pose.position.z = t_mf(2);
                mapping_pose.pose.orientation.x = q_mf.x();
                mapping_pose.pose.orientation.y = q_mf.y();
                mapping_pose.pose.orientation.z = q_mf.z();
                mapping_pose.pose.orientation.w = q_mf.w();

                mapDataPerFrameBuf.pop_front();
                frameTobeMap++;
                mapping_ready = true;
            // end if
            } else {
                mapping_ready = false;
            } 
        }

};


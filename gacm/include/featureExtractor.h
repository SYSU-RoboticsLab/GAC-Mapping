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
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "tic_toc.h"
#include "parameters.h"
#include "pcl/point_cloud.h"


struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const std::vector<cv::DMatch>& a, const std::vector<cv::DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};



struct sort_points_by_response
{
    inline bool operator()(const cv::KeyPoint& a, const cv::KeyPoint& b){
        return ( a.response > b.response );
    }
};

class FeatureTracker {

    private:

        int temp_counter = 0;
        ros::Subscriber imageDataSub;
        camodocal::CameraPtr m_camera;
        
        cv::Mat image_cur, image_last;
        cv::Mat image_cur_with_text, image_last_with_text;
        double time_cur=0, time_last=0;
        
        // point feature
        // cv::Ptr<cv::ORB> orb;
        std::vector<cv::Point2f> keypoints_cur, keypoints_last;
        std::vector<cv::Point2f> keypoints_add;
        cv::Mat point_descriptors_cur, point_descriptors_last;

        // point feature info
        std::vector<int> keypoints_id;
        std::vector<int> keypoints_track_count;
        cv::Mat point_mask;
        int keypoint_id_cur = 0;

        


        // publish data
        pcl::PointCloud<ImagePoint>::Ptr point_to_pub;
        
    public:


        FeatureTracker() {

           point_to_pub = pcl::PointCloud<ImagePoint>::Ptr(new pcl::PointCloud<ImagePoint>());
        }

        void imageDataCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
            clock_t start, finish;
            time_cur = image_msg->header.stamp.toSec();
            cv_bridge::CvImageConstPtr ptr;
            ptr= cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            if (image_cur.empty()) {
                     
                ROS_INFO_STREAM("First image");
                image_cur = ptr->image.clone();
                if (DOWN_SAMPLE_NEED) {
                    cv::resize(image_cur, image_cur, cv::Size(COL, ROW), cv::INTER_LINEAR);
                }
                image_last = image_cur.clone(); // first image

                image_cur_with_text = image_cur.clone();
                image_last_with_text = image_cur_with_text.clone();
            } else {
                image_cur = ptr->image.clone();
                if (DOWN_SAMPLE_NEED) {
                    cv::resize(image_cur, image_cur, cv::Size(COL, ROW), cv::INTER_LINEAR);
                }
                image_cur_with_text = image_cur.clone();
            }
           
            if (keypoints_last.size() > 0) {
                // ROS_INFO_STREAM("Detect optical flow");
                std::vector<unsigned char> status;
                std::vector<float> error;
                cv::calcOpticalFlowPyrLK(image_last,image_cur,keypoints_last,keypoints_cur,
                                    status, error, cv::Size(21,21),3);
                for (int i = 0; i < int(keypoints_cur.size()); i++) {
                    if (status[i] && !inBorder(keypoints_cur[i])) {
                        status[i] = 0;
                    }
                }
                reduceVector(keypoints_last, status);
                reduceVector(keypoints_cur, status);
                reduceVector(keypoints_id, status);
                reduceVector(keypoints_track_count, status);
            }

            for (auto &n : keypoints_track_count) {
                n++;
            }
            // ROS_ERROR_STREAM("done1");
            // if (keypoints_last.size() > 0) {
            //     displayFeature("feature last",image_last,keypoints_last,keypoints_track_count, keypoints_id);
            // }
            rejectWithFundamentalMat(keypoints_last, keypoints_cur, keypoints_track_count, keypoints_id);
            setMask(point_mask, keypoints_cur, keypoints_track_count, keypoints_id, MIN_DIST_CORNER);
            int max_count_allow = MAX_CNT_CORNER - static_cast<int>(keypoints_cur.size());
            if (max_count_allow > 0) {
                keypoints_add.clear();
                cv::Mat image_cur_gray;
                cv::cvtColor(image_cur, image_cur_gray, cv::COLOR_RGB2GRAY);
                cv::goodFeaturesToTrack(image_cur_gray, keypoints_add, max_count_allow, 0.01, MIN_DIST_CORNER, point_mask);
                addPoints(keypoints_add, keypoints_cur, keypoints_track_count, keypoints_id, keypoint_id_cur);
            }
            // ROS_ERROR_STREAM("done2");
            
            displayFeature("feature cur", image_cur, keypoints_cur, keypoints_track_count, keypoints_id, image_cur_with_text);
            // ROS_ERROR_STREAM("done3");
            // cv::waitKey(5);

            

            time_last = time_cur;
            image_last = image_cur.clone();
            image_last_with_text = image_cur_with_text.clone();
            keypoints_last = keypoints_cur;
            keypoints_cur.clear();
            point_descriptors_last = point_descriptors_cur;

            // ROS_INFO_STREAM("GeneratePointCloud");
            generatePointCloud(); // publish features
            // ROS_INFO_STREAM("GenerateLineCloud");
            // generateLineCloud(); // publish features
            // ROS_INFO_STREAM("FinishGenerateLineCloud");
        }





        bool inBorder(const cv::Point2f &pt) {
            const int BORDER_SIZE = 2;
            int img_x = cvRound(pt.x);
            int img_y = cvRound(pt.y);
            return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
        }

        template <typename T>
        void reduceVector(std::vector<T> &v, std::vector<uchar> status) {
            int j = 0;
            for (int i = 0; i < int(v.size()); i++)
                if (status[i])
                    v[j++] = v[i];
            v.resize(j);
        }



        void setMask(cv::Mat& mask, std::vector<cv::Point2f>& points, std::vector<int>& track_cnt, std::vector<int>& ids,const int MIN_DIST) {
            mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
            

            // prefer to keep features that are tracked for long time
            std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

            for (unsigned int i = 0; i < points.size(); i++)
                cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(points[i], ids[i])));

            sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b)
                {
                    return a.first > b.first;
                });

            points.clear();
            ids.clear();
            track_cnt.clear();

            for (auto &it : cnt_pts_id)
            {
                if (mask.at<uchar>(it.second.first) == 255)
                {
                    points.push_back(it.second.first);
                    ids.push_back(it.second.second);
                    track_cnt.push_back(it.first);
                    cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
                }
            }
        }

        void addPoints(std::vector<cv::Point2f>& keypoints_add, std::vector<cv::Point2f>& keypoints_cur, std::vector<int>& track_cnt, std::vector<int>& ids, int& id_cur) {
            for (auto &p : keypoints_add) {
                keypoints_cur.push_back(p);
                ids.push_back(id_cur++);
                track_cnt.push_back(1);
            }
        }

        void rejectWithFundamentalMat(std::vector<cv::Point2f>& keypoints_last, std::vector<cv::Point2f>& keypoints_cur, std::vector<int>& track_cnt, std::vector<int>& ids) {
            if (keypoints_cur.size() >= 8)
            {
                ROS_DEBUG("FM ransac begins");
                TicToc t_f;
                std::vector<cv::Point2f> un_cur_pts(keypoints_last.size()), un_forw_pts(keypoints_cur.size());
                for (unsigned int i = 0; i < keypoints_last.size(); i++)
                {
                    Eigen::Vector3d tmp_p;
                    m_camera->liftProjective(Eigen::Vector2d(keypoints_last[i].x, keypoints_last[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                    m_camera->liftProjective(Eigen::Vector2d(keypoints_cur[i].x, keypoints_cur[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                }

                std::vector<uchar> status;
                cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                int size_a = keypoints_last.size();
                reduceVector(keypoints_last, status);
                reduceVector(keypoints_cur, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, keypoints_cur.size(), 1.0 * keypoints_cur.size() / size_a);
                ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
            }
        }
            
        void readIntrinsicParameter(const std::string &calib_file) {
            ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
            m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
        }

        void displayFeature(std::string window_name, cv::Mat& image, std::vector<cv::Point2f>& points, std::vector<int>& track_cnt, std::vector<int>& ids, cv::Mat& image_out) {
            // cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
            cv::Mat temp_img = image.clone();
            // ROS_WARN_STREAM("Debug size " << points.size() << " " << track_cnt.size() << " " << ids.size());
            for (size_t i = 0 ; i < points.size(); i++) {
                double len = std::min(1.0, 1.0 * track_cnt[i] / WINDOW_SIZE);
                cv::circle(temp_img, points[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                char name[20];
                sprintf(name, "%d(%d)", ids[i], track_cnt[i]);
                cv::putText(temp_img, name, points[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            image_out = temp_img.clone();
            // ROS_WARN_STREAM("Now displaying feature");
            // cv::imshow(window_name, temp_img);
            // cv::waitKey(5);
            // std::stringstream img_name;
            // img_name << "/home/ncslab_slam/Pictures/experiment/testrgb/" <<temp_counter++<<".png";
            // cv::imwrite(img_name.str(), image);
        }


        void generatePointCloud() {
            point_to_pub->clear();
            ImagePoint point;
            for (size_t i = 0; i < keypoints_last.size(); i++) {
                point.index = keypoints_id[i];
                point.track_cnt = keypoints_track_count[i];
                // point.u = keypoints_cur[i].x; // in OpenCV x is correspond to cols 
                // point.v = keypoints_cur[i].y;
                // Eigen::Vector2d a(keypoints_last[i].x, keypoints_last[i].y);
                // Eigen::Vector3d b;
                // m_camera->liftProjective(a,b);
                point.u = keypoints_last[i].x; // in OpenCV x correspond to cols, that is u
                point.v = keypoints_last[i].y; // in OpenCV y correspond to rows, that is v 
                // point.x = b[0];
                // point.y = b[1];
                // point.z = b[2];
                point_to_pub->push_back(point);
            }
        }

   



                    
        

        pcl::PointCloud<ImagePoint>::Ptr getPointToPub() {
            return point_to_pub;
        }


        double getTimeStampLast() {
            return time_last;
        }

        const cv::Mat getImageLast() {
            return image_last.clone();
        }

        const cv::Mat getKeyPointImageLast() {
            return image_last_with_text.clone();
        }

};





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

#include<memory>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Empty.h>


#include "featureDefinition.h"
#include "featureExtractor.h"
#include "scanRegistrator.h"




std::shared_ptr<FeatureTracker> feature_tracker;
std::shared_ptr<ScanRegistrator> scan_registrator;
ros::Publisher pub_image, pub_keypoints;//, pub_keylines;
ros::Publisher pub_feature_image;

ros::Publisher pubFullResPoints;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;

ros::NodeHandle* nhp_ptr;
// cv::Mat rgb_last;
// cv::
void imageDataCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
// void imageDataCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
    
    // double time_image = image_msg->header.stamp.toSec();

    feature_tracker->imageDataCallback(image_msg);
    feature_tracker->generatePointCloud();
    
    sensor_msgs::PointCloud2 keypoints, keylines;
    if (feature_tracker->getPointToPub()->size() > 10) {
        pcl::toROSMsg(*feature_tracker->getPointToPub(), keypoints);
    }
    keypoints.header.stamp = ros::Time().fromSec(feature_tracker->getTimeStampLast());
    pub_keypoints.publish(keypoints);

    cv::Mat feature_image(feature_tracker->getImageLast());
    cv::Mat feature_image_text(feature_tracker->getKeyPointImageLast());
    cv_bridge::CvImage bridge;
    bridge.image = feature_image;
    bridge.encoding = "bgr8";
    sensor_msgs::Image::Ptr feature_image_ptr = bridge.toImageMsg();
    feature_image_ptr->header.stamp = ros::Time().fromSec(feature_tracker->getTimeStampLast());
    pub_image.publish(feature_image_ptr);
    bridge.image = feature_image_text;
    feature_image_ptr = bridge.toImageMsg();
    feature_image_ptr->header.stamp = ros::Time().fromSec(feature_tracker->getTimeStampLast());
    pub_feature_image.publish(feature_image_ptr);

}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laser_msg) {

    // double time_laser = laser_msg->header.stamp.toSec();

    scan_registrator->laserCloudHandler(laser_msg);

    sensor_msgs::PointCloud2 laserCloudOutMsg;

    if (pubFullResPoints.getNumSubscribers() != 0){
        pcl::toROSMsg(*(scan_registrator->getFullResCloud()), laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = ros::Time().fromSec(scan_registrator->getStamp());
        laserCloudOutMsg.header.frame_id = "/camera";
        pubFullResPoints.publish(laserCloudOutMsg);
    }

    if (pubCornerPointsSharp.getNumSubscribers() != 0){
        pcl::toROSMsg(*(scan_registrator->getCornerPointsSharp()), laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = ros::Time().fromSec(scan_registrator->getStamp());
        laserCloudOutMsg.header.frame_id = "/camera";
        pubCornerPointsSharp.publish(laserCloudOutMsg);
    }

    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
        pcl::toROSMsg(*(scan_registrator->getCornerPointsLessSharp()), laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = ros::Time().fromSec(scan_registrator->getStamp());
        laserCloudOutMsg.header.frame_id = "/camera";
        pubCornerPointsLessSharp.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsFlat.getNumSubscribers() != 0){
        pcl::toROSMsg(*(scan_registrator->getSurfPointsFlat()), laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = ros::Time().fromSec(scan_registrator->getStamp());
        laserCloudOutMsg.header.frame_id = "/camera";
        pubSurfPointsFlat.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
        pcl::toROSMsg(*(scan_registrator->getSurfPointsLessFlat()), laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = ros::Time().fromSec(scan_registrator->getStamp());
        laserCloudOutMsg.header.frame_id = "/camera";
        pubSurfPointsLessFlat.publish(laserCloudOutMsg);
    }
}

void imagePointCloudCb(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr &laser_msg) {
    // double time_image = image_msg->header.stamp.toSec();
    // double time_laser = laser_msg->header.stamp.toSec();
    // ROS_INFO_STREAM("Message Receive: " << time_image << "&&" << time_laser <<'\n');

    imageDataCallback(image_msg);
    pointCloudCallback(laser_msg);

}

void newSessionSignalHandler(const std_msgs::EmptyConstPtr& empty) {
    // ROS_ERROR_STREAM("CONFIG_ID tracker " << CONFIG_ID);
    CONFIG_ID++;
    readParameters(*nhp_ptr);
    feature_tracker = std::shared_ptr<FeatureTracker>(new FeatureTracker()) ;
    scan_registrator = std::shared_ptr<ScanRegistrator>(new ScanRegistrator());
    feature_tracker->readIntrinsicParameter(CAM_NAME);
    scan_registrator->readIntrinsicParameter(CAM_NAME);
}

int main(int argc, char** argv) {

    ros::init(argc,argv,"featureTracking");
    ros::NodeHandle nh_p ("~");
    nhp_ptr = &nh_p;
    ros::NodeHandle nh;
    readParameters(nh_p);
    
    feature_tracker = std::shared_ptr<FeatureTracker>(new FeatureTracker()) ;
    scan_registrator = std::shared_ptr<ScanRegistrator>(new ScanRegistrator());
    // Subscribers
    // ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>("/image_raw",10, imageDataCallback);
    // ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, pointCloudCallback);
    // message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, "/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, "/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud(nh, "/velodyne_points", 1);
    
    // If strictly synchronized use timestamp to synchronize the topics 
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(sub_image, sub_pointcloud, 10);
    // sync.registerCallback(boost::bind(&imagePointCloudCb, _1, _2));

    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_pointcloud);
    sync.registerCallback(boost::bind(&imagePointCloudCb, _1, _2));

    // Publishers
    pub_image = nh.advertise<sensor_msgs::Image> ("feature_image",10);
    pub_feature_image = nh.advertise<sensor_msgs::Image>("feature_image_with_text",10);
    pub_keypoints = nh.advertise<sensor_msgs::PointCloud2> ("feature_points",10);
    // pub_keylines = nh.advertise<sensor_msgs::PointCloud2> ("/feature_lines",10);

    pubFullResPoints = nh.advertise<sensor_msgs::PointCloud2>("laser_full",1);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_sharp", 1);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_less_sharp", 1);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_flat", 1);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_less_flat", 1);


    feature_tracker->readIntrinsicParameter(CAM_NAME);
    scan_registrator->readIntrinsicParameter(CAM_NAME);

    ros::Subscriber subNewSessionSignal = nh.subscribe<std_msgs::Empty>("/start_new_session", 10, newSessionSignalHandler);

    
    ros::spin();
    return 0;
}
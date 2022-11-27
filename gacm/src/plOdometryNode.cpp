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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Empty.h>



#include "poseEstimator.h"


std::shared_ptr<PoseEstimator> pose_estimator;

ros::Publisher pubLaserCloudCornerLast;
ros::Publisher pubLaserCloudSurfLast;
ros::Publisher pubLaserCloudFullRes;
ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserPath;
ros::Publisher pubLaserPath2;
ros::Publisher pubPointFeatures;

ros::Publisher pubRGBImage;
ros::Publisher pubDepthImage;
ros::Publisher pubMappingPose;

ros::Publisher pubDepthImageRGB;


ros::NodeHandle* nhp_ptr;

int frame_count = 0;

void featureCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                                                    const sensor_msgs::PointCloud2ConstPtr &feature_points,/*  const sensor_msgs::PointCloud2ConstPtr &feature_lines,*/
                                                    const sensor_msgs::PointCloud2ConstPtr &feature_sharp,const sensor_msgs::PointCloud2ConstPtr &feature_less_sharp,
                                                    const sensor_msgs::PointCloud2ConstPtr &feature_flat,const sensor_msgs::PointCloud2ConstPtr &feature_less_flat,
                                                    const sensor_msgs::PointCloud2ConstPtr &laser_full) {

    
}

void imageHandler(const sensor_msgs::ImageConstPtr &image_msg) {
    pose_estimator->imageHandler(image_msg);
}

void featurePointsHandler(const sensor_msgs::PointCloud2ConstPtr &feature_points_msg) {
    pose_estimator->featurePointsHandler(feature_points_msg);
}


void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp) {
    pose_estimator->laserCloudSharpHandler(corner_points_sharp);
    // ROS_INFO_STREAM("Receive sharp ");
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp) {
    pose_estimator->laserCloudLessSharpHandler(corner_points_less_sharp);
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_flat) {
    pose_estimator->laserCloudFlatHandler(surf_points_flat);
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_less_flat) {
    pose_estimator->laserCloudLessFlatHandler(surf_points_less_flat);
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_fullres) {
    pose_estimator->laserCloudFullResHandler(laser_cloud_fullres);
}

void publishAll() {
    pubLaserPath.publish(pose_estimator->getLaserPath());
    pubLaserPath2.publish(pose_estimator->getLaserPath2());
    pubPointFeatures.publish(pose_estimator->getMapPointCloud());

    if (pose_estimator->cloudNeedPub() && pose_estimator->odomNeedPub()) {
        pubLaserCloudCornerLast.publish(pose_estimator->getLaserCloudCornerLastMsg());
        pubLaserCloudSurfLast.publish(pose_estimator->getLaserCloudSurfLastMsg());
        pubLaserCloudFullRes.publish(pose_estimator->getLaserCloudFullResMsg());
        // ROS_ERROR_STREAM("Cloud Pub Finished####################");
        pubLaserOdometry.publish(pose_estimator->getLaserOdometry());  
        pubDepthImageRGB.publish(pose_estimator->getDepthImageRGB());
    }

    if (pose_estimator->mappingReady()) {
        pubRGBImage.publish(pose_estimator->getRGBImage());
        pubDepthImage.publish(pose_estimator->getDepthImage());
        pubMappingPose.publish(pose_estimator->getMappingPose());
    }
}

void newSessionSignalHandler(const std_msgs::EmptyConstPtr& empty) {
    CONFIG_ID++;
    readParameters(*nhp_ptr);
    pose_estimator = std::shared_ptr<PoseEstimator>(new PoseEstimator(CAM_NAME));
}


int main(int argc, char** argv) {

    ros::init(argc,argv,"plOdometry");
    ros::NodeHandle nh_p ("~");
    ros::NodeHandle nh;
    nhp_ptr = &nh_p;
    readParameters(nh_p);
    
    pose_estimator = std::shared_ptr<PoseEstimator>(new PoseEstimator(CAM_NAME));

    // Subscribers
    
    message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, "feature_image", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_feature_points(nh, "feature_points", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_feature_lines(nh, "/feature_lines", 1);

    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser_sharp(nh, "/laser_cloud_sharp", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser_less_sharp(nh, "/laser_cloud_less_sharp", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser_flat(nh, "/laser_cloud_flat", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser_less_flat(nh, "/laser_cloud_less_flat", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser_full(nh, "/laser_full", 1);

    // for submap manager
    pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_corner_last", 10);
    pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_surf_last", 10);
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("laser_full_3", 10);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("laser_odom_to_init", 10);

    pubLaserPath = nh.advertise<nav_msgs::Path>("laser_odom_path", 10);
    pubLaserPath2 = nh.advertise<nav_msgs::Path>("laser_odom_path2", 10);
    pubPointFeatures = nh.advertise<sensor_msgs::PointCloud2>("point_3d", 10);

    // for densemapping
    pubRGBImage = nh.advertise<sensor_msgs::Image>("mapping_rgb", 10);
    pubDepthImage = nh.advertise<sensor_msgs::Image>("mapping_depth", 10);
    pubMappingPose = nh.advertise<geometry_msgs::PoseStamped>("mapping_pose", 10);
    pubDepthImageRGB = nh.advertise<sensor_msgs::Image>("depth_rgb", 10);

    // subscribe topics from feature extractor

    ros::Subscriber subImageRaw = nh.subscribe<sensor_msgs::Image>("feature_image", 10, imageHandler);
    ros::Subscriber subFeaturePoints = nh.subscribe<sensor_msgs::PointCloud2>("feature_points", 10, featurePointsHandler);
    // subscribe topics from scan registrator
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_sharp", 10, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_less_sharp", 10, laserCloudLessSharpHandler);
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_flat", 10, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud_less_flat", 10, laserCloudLessFlatHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("laser_full", 10, laserCloudFullResHandler);
    

    ros::Subscriber subNewSessionSignal = nh.subscribe<std_msgs::Empty>("/start_new_session", 10, newSessionSignalHandler);
    // synchronizer
    
    if(DEBUG) ROS_WARN_STREAM("Ready to loop");

    ros::Rate loop_rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        pose_estimator->handleImage();
        publishAll();
        loop_rate.sleep();
    }

    return 0;
}
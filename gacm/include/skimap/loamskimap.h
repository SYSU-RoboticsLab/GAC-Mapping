#ifndef LOAM_LOAMSKIMAP_H
#define LOAM_LOAMSKIMAP_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <skimap/SkiMapInterface.h>

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

// OPENCV
#include <opencv2/opencv.hpp>

#include <iostream>  
#include <string>

namespace loam {
    class loamskimap {
    public:
        loamskimap() {
        }
        ~loamskimap() {
	    }
        void Init(ros::NodeHandle *nh) {
            skiMapInterface.init(nh);
        }
        void updateskimap(sensor_msgs::PointCloud2Ptr &cloud, Eigen::Matrix4d loam_pose) {
            skiMapInterface.updateMap1(cloud, loam_pose);
        }
        void publishskimap() {
            skiMapInterface.pointCloudMapPublish();
        }
        void getRP(std::vector<Voxel3D> &voxels, Eigen::Matrix4d lidarPose) {
            skiMapInterface.getRadioPoints(voxels, lidarPose);
        }
        visualization_msgs::Marker createmarker(std::string frame_id, int id, VisualizationType type) {
            visualization_msgs::Marker map_marker = skiMapInterface.createVisualizationMarker(frame_id, id, type);
            return map_marker;
        }
        void fillvoxels(visualization_msgs::Marker &voxels_marker, std::vector<Voxel3D> &voxels, int min_weight_th) {
            skiMapInterface.fillVisualizationMarkerWithVoxels(voxels_marker, voxels, min_weight_th);
        }
        void publishvoxelmap(visualization_msgs::Marker marker) {
            skiMapInterface.mapPublisher(marker);
        }
    private:
        SkiMapInterface skiMapInterface;
    };
}

#endif  //LOAM_LOAMSKIMAP_H

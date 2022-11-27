
#ifndef LIDAR_SLAM_INCLUDE_LIDAR_SLAM_UTILS_H_
#define LIDAR_SLAM_INCLUDE_LIDAR_SLAM_UTILS_H_

#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <set>

#include "transform.h"
#include "st_lidar_slam_utils.h"

#define CONFIG_LIDAR_SLAM_VIEWER_WIDTH  1440
#define CONFIG_LIDAR_SLAM_VIEWER_HEIGHT 1080
#define CONFIG_LIDAR_SLAM_SHOW_VIEWER   0x00000001 

#define CONFIG_LIDAR_SLAM_INPUT_LIDAR      0x00000001
#define CONFIG_LIDAR_SLAM_INPUT_GNSS_POSE  0x00000010
#define CONFIG_LIDAR_SLAM_INPUT_GNSS_DATA  0x00000100
#define CONFIG_LIDAR_SLAM_INPUT_CAMERA     0x00001000

namespace slam3d {

typedef st_point3d_t PointT;
typedef std::vector<PointT> PointCloudT; 

struct lidar_slam_config_t {
    lidar_slam_config_t() {
        display_map_viewer_flag = true;
        pts_semantic_classify_flag = true;
        global_optimize_with_gnss_flag = true;
        pts_downsample_flag = true;
        pts_sample_step = 5;
        local_map_size = 5;
    }

    bool display_map_viewer_flag;
    
    bool pts_semantic_classify_flag;       ///< point semantic classify flag
    bool global_optimize_with_gnss_flag;   ///< process global optimize with gnss flag
    bool pts_downsample_flag;              ///< points downsample flag
    int pts_sample_step;  ///< points sample step
    int local_map_size;   ///< local map size
};

struct sensors_data_t {
    sensors_data_t() {
        config = 0;
        frame_id = 0;
    }

    int frame_id;
    struct timeval time_stamp;
    std::vector<st_point3d_t> lidar_points_3d;  // lidar 3d points
    Eigen::Matrix4d gnssins_pose;               // gnss pose 
    st_imu_data_t imu_data;
    unsigned int config;
};

struct SubmapData {
    SubmapData() {
        index = -1; 
        has_gnss_pose = false;
    }

	std::set<int> node_idxs;
//	Submap::state_t state = Submap::Active;

	int index;
	Transform3D pose;
	Transform3D gnss_pose;
    bool has_gnss_pose;
};

Eigen::Matrix3d vector_to_Rotation(const Eigen::Vector3d& vec);

typedef std::vector<PointT> PointCloudT; 

//inline void PclPointCloudToPointCloudT(const pcl::PointCloud<pcl::PointXYZI>& pts,
//        PointCloudT& out) {
//    out.resize(pts.size());
//    for (int i = 0; i < pts.size(); ++i) {
//        out[i].x = pts[i].x;
//        out[i].y = pts[i].y;
//        out[i].z = pts[i].z;
//        out[i].intensity = pts[i].intensity;
//    }
//}
//
//inline void PointCloudTtoPclPointCloud(const PointCloudT& in,
//        pcl::PointCloud<pcl::PointXYZI>& out) {
//    out.resize(in.size());
//    for (int i = 0; i < in.size(); ++i) {
//        out[i].x = in[i].x;
//        out[i].y = in[i].y;
//        out[i].z = in[i].z;
//        out[i].intensity = in[i].intensity;
//    }
//}

inline Eigen::Vector3f TransformPoint(const Eigen::Isometry3f& T, const PointT& pt) {
    Eigen::Vector3f result(pt.x, pt.y, pt.z);
    result = T * result;
    return result;
}

}  // namespace slam3d

#endif  // LIDAR_SLAM_INCLUDE_LIDAR_SLAM_UTILS_H_

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */

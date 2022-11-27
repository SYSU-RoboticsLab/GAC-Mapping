//
// Created by huyh on 18-12-17.
//

#ifndef PROJECT_SKIMAPINTERFACE_H
#define PROJECT_SKIMAPINTERFACE_H

// Ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

// OPENCV
#include <opencv2/opencv.hpp>
#include <pcl/impl/point_types.hpp>


// lidarSlam
#include "skimap/cloud.h"
#include "skimap/map_viewer.h"

/**
 * skimap Voxel3d definition
 */
typedef float CoordinateType;
typedef int16_t IndexType;
typedef uint16_t WeightType;
typedef skimap::VoxelDataRGBW<IndexType, WeightType> VoxelDataColor;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType> SKIMAP;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType>::Voxel3D
        Voxel3D;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType>::Tiles2D
        Tiles2D;

/**
 * Map Parameters
 */
struct MapParameters {
    float ground_level;
    float agent_height;
    float map_resolution;
    int min_voxel_weight;
    bool enable_chisel;
    bool height_color;
    int chisel_step;
} mapParameters;

/**
 * Visualizatoin types for Markers
 */
enum VisualizationType {
    POINT_CLOUD,
    VOXEL_MAP,
    VOXEL_GRID,
};

class SkiMapInterface{
public:
    /**
     * Construction
     */
    SkiMapInterface(){
    }
    /**
     * Deconstruction
     */
    ~SkiMapInterface(){
    }

    /**
     * Initialization
     * @param nh
     */
    void init(ros::NodeHandle *nh){
        nh_ = nh;
        // SkiMap Parameters
        nh->param<float>("map_resolution", mapParameters.map_resolution, 0.1f);
        nh->param<float>("ground_level", mapParameters.ground_level, 0.15f);
        nh->param<int>("min_voxel_weight", mapParameters.min_voxel_weight, 2);
        nh->param<bool>("enable_chisel", mapParameters.enable_chisel, false);
        nh->param<bool>("height_color", mapParameters.height_color, false);
        nh->param<int>("chisel_step", mapParameters.chisel_step, 10);
        nh->param<float>("agent_height", mapParameters.agent_height, 1.0f);
        map_ = new SKIMAP(mapParameters.map_resolution, mapParameters.ground_level);

        // Builds the map
        float map_resolution = 0.1;
        map_ = new SKIMAP(map_resolution);

        /**
         * This command enables the Concurrent Access Self Management. If it is
         * enabled
         * you can use OPENMP to call the method 'integrateVoxel' in a concurrent
         * manner safely.
         */
        map_->enableConcurrencyAccess();

        /// publisher initialization
        std::string map_2d_topic =
                nh->param<std::string>("map_2d_publisher_topic", "live_map_2d");
        map_2d_publisher_ = nh->advertise<visualization_msgs::Marker>(map_2d_topic, 1);
        std::string map_topic =
                nh->param<std::string>("map_publisher_topic", "live_map");
        map_publisher_ = nh->advertise<visualization_msgs::Marker>(map_topic, 1);
        std::string odom_topic =
                nh->param<std::string>("odom_publisher_topic", "odom_pub");
        odom_publisher_ = nh->advertise<nav_msgs::Odometry>(odom_topic,1);
        // publish map as pointcloud
        std::string point_cloud_map_topic=
                nh_->param<std::string>("point_cloud_map_publisher_topic", "map");
        point_cloud_map_publisher_ = nh->advertise<sensor_msgs::PointCloud2>(point_cloud_map_topic,1);
    }
    /**
     * update lidar points to skiMap
     * @param cur_lidar_pts
     */
    void updateMap(std::vector<map_point3d_t> cur_lidar_pts){

        for(int k=0;k<cur_lidar_pts.size();k++){
            VoxelDataColor voxel;
            voxel.r = cur_lidar_pts[k].intensity*0.3;
            voxel.g = cur_lidar_pts[k].intensity*0.59;
            voxel.b = cur_lidar_pts[k].intensity*0.11;
            voxel.w = 1.0;
            double x = cur_lidar_pts[k].x;
            double y = cur_lidar_pts[k].y;
            double z = cur_lidar_pts[k].z;
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
        }
    }
    /**
     * update semantic cloud to the map
     * @param semantic_cloud
     * @param lidar_pose
     */
    void updateMap(slam3d::Cloud semantic_cloud, Eigen::Matrix4f lidar_pose){
        VoxelDataColor voxel;
        unsigned long semantic_cloud_size = semantic_cloud.grounds_.points.size() + semantic_cloud.edges_.points.size() +
                semantic_cloud.planes_.points.size();
        std::vector<map_point3d_t> cur_lidar_pts(semantic_cloud_size);
        int index=0;
        for (int k = 0; k < semantic_cloud.planes_.points.size(); ++k) {
            cur_lidar_pts[index].x = semantic_cloud.planes_.points[k].x * lidar_pose(0, 0) + semantic_cloud.planes_.points[k].y * lidar_pose(0, 1) +
                    semantic_cloud.planes_.points[k].z * lidar_pose(0, 2) + lidar_pose(0, 3);
            cur_lidar_pts[index].y = semantic_cloud.planes_.points[k].x * lidar_pose(1, 0) + semantic_cloud.planes_.points[k].y * lidar_pose(1, 1) +
                    semantic_cloud.planes_.points[k].z * lidar_pose(1, 2) + lidar_pose(1, 3);
            cur_lidar_pts[index].z = semantic_cloud.planes_.points[k].x * lidar_pose(2, 0) + semantic_cloud.planes_.points[k].y * lidar_pose(2, 1) +
                    semantic_cloud.planes_.points[k].z * lidar_pose(2, 2) + lidar_pose(2, 3);
            cur_lidar_pts[index].intensity = semantic_cloud.planes_.points[k].intensity;
            voxel.r = 255;
            voxel.g = 0;
            voxel.b = 0;
            voxel.w = 1.0;
            double x = cur_lidar_pts[index].x;
            double y = cur_lidar_pts[index].y;
            double z = cur_lidar_pts[index].z;
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
            index++;
        }
        for (int k = 0; k < semantic_cloud.edges_.points.size(); ++k) {
            cur_lidar_pts[index].x = semantic_cloud.edges_.points[k].x * lidar_pose(0, 0) + semantic_cloud.edges_.points[k].y * lidar_pose(0, 1) +
                                 semantic_cloud.edges_.points[k].z * lidar_pose(0, 2) + lidar_pose(0, 3);
            cur_lidar_pts[index].y = semantic_cloud.edges_.points[k].x * lidar_pose(1, 0) + semantic_cloud.edges_.points[k].y * lidar_pose(1, 1) +
                                 semantic_cloud.edges_.points[k].z * lidar_pose(1, 2) + lidar_pose(1, 3);
            cur_lidar_pts[index].z = semantic_cloud.edges_.points[k].x * lidar_pose(2, 0) + semantic_cloud.edges_.points[k].y * lidar_pose(2, 1) +
                                 semantic_cloud.edges_.points[k].z * lidar_pose(2, 2) + lidar_pose(2, 3);
            cur_lidar_pts[index].intensity = semantic_cloud.edges_.points[k].intensity;
            voxel.r = 0;
            voxel.g = 255;
            voxel.b = 0;
            voxel.w = 1.0;
            double x = cur_lidar_pts[index].x;
            double y = cur_lidar_pts[index].y;
            double z = cur_lidar_pts[index].z;
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
            index++;
        }
        for (int k = 0; k < semantic_cloud.grounds_.points.size(); ++k) {
            cur_lidar_pts[index].x = semantic_cloud.grounds_.points[k].x * lidar_pose(0, 0) + semantic_cloud.grounds_.points[k].y * lidar_pose(0, 1) +
                                 semantic_cloud.grounds_.points[k].z * lidar_pose(0, 2) + lidar_pose(0, 3);
            cur_lidar_pts[index].y = semantic_cloud.grounds_.points[k].x * lidar_pose(1, 0) + semantic_cloud.grounds_.points[k].y * lidar_pose(1, 1) +
                                 semantic_cloud.grounds_.points[k].z * lidar_pose(1, 2) + lidar_pose(1, 3);
            cur_lidar_pts[index].z = semantic_cloud.grounds_.points[k].x * lidar_pose(2, 0) + semantic_cloud.grounds_.points[k].y * lidar_pose(2, 1) +
                                 semantic_cloud.grounds_.points[k].z * lidar_pose(2, 2) + lidar_pose(2, 3);
            cur_lidar_pts[index].intensity = semantic_cloud.planes_.points[k].intensity;
            voxel.r = 0;
            voxel.g = 0;
            voxel.b = 255;
            voxel.w = 1.0;
            double x = cur_lidar_pts[index].x;
            double y = cur_lidar_pts[index].y;
            double z = cur_lidar_pts[index].z;
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
            index++;
        }
#if 0 //update intensity
        for(int k=0;k<cur_lidar_pts.size();k++){
            voxel.r = cur_lidar_pts[k].intensity*0.3;
            voxel.g = cur_lidar_pts[k].intensity*0.59;
            voxel.b = cur_lidar_pts[k].intensity*0.11;
            voxel.w = 1.0;
            double x = cur_lidar_pts[k].x;
            double y = cur_lidar_pts[k].y;
            double z = cur_lidar_pts[k].z;
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
        }
#endif
    }
    /**
     * update cloud to the map
     * @param cur_cloud
     * @param loam_pose
     */
    void updateMap1(sensor_msgs::PointCloud2Ptr &cur_cloud, Eigen::Matrix4d loam_pose){

        VoxelDataColor voxel;
        unsigned long cur_cloud_size = cur_cloud->height * cur_cloud->width;
        int index=0;
        for (int k = 0; k < cur_cloud_size; ++k) {

            float cur_x, cur_y, cur_z;
            uint8_t bb,gg,rr;
            memcpy(&cur_x, &cur_cloud->data[k * cur_cloud->point_step + 0], sizeof(float));
            memcpy(&cur_y, &cur_cloud->data[k * cur_cloud->point_step + 4], sizeof(float));
            memcpy(&cur_z, &cur_cloud->data[k * cur_cloud->point_step + 8], sizeof(float));
            //memcpy(&g, &cur_cloud->data[k * cur_cloud->point_step + 12], sizeof(uchar));
            memcpy(&bb, &cur_cloud->data[k * cur_cloud->point_step + 12], sizeof(uint8_t));
            memcpy(&gg, &cur_cloud->data[k * cur_cloud->point_step + 13], sizeof(uint8_t));
            memcpy(&rr, &cur_cloud->data[k * cur_cloud->point_step + 14], sizeof(uint8_t));
            double x, y, z;
            x = loam_pose(0,0)*cur_x + loam_pose(0,1)*cur_y + loam_pose(0,2)*cur_z + loam_pose(0,3);
            y = loam_pose(1,0)*cur_x + loam_pose(1,1)*cur_y + loam_pose(1,2)*cur_z + loam_pose(1,3);
            z = loam_pose(2,0)*cur_x + loam_pose(2,1)*cur_y + loam_pose(2,2)*cur_z + loam_pose(2,3);
            voxel.r = rr;
            voxel.g = gg;
            voxel.b = bb;
            voxel.w = 1.0;

            map_->integrateVoxel(CoordinateType(x), CoordinateType(y),
                                 CoordinateType(z), &voxel);
            index++;
        }
    }
    /**
     * get Voxel from the SkiMap
     * @param voxels
     */
    void pointCloudMapPublish(){
        ROS_DEBUG("Point Cloud Map Publish started.");
        std::vector<Voxel3D> voxels;
        map_->fetchVoxels(voxels);
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud_map;
        point_cloud_map.resize(voxels.size());
        for(size_t i=0;i<voxels.size();++i){
            point_cloud_map.points[i].x = voxels[i].x;
            point_cloud_map.points[i].y = voxels[i].y;
            point_cloud_map.points[i].z = voxels[i].z;
            point_cloud_map.points[i].r = voxels[i].data->r;
            point_cloud_map.points[i].g = voxels[i].data->g;
            point_cloud_map.points[i].b = voxels[i].data->b;
        }
        sensor_msgs::PointCloud2 point_cloud_map_out;
        pcl::toROSMsg(point_cloud_map,point_cloud_map_out);
        point_cloud_map_out.header.frame_id="/camera";
        point_cloud_map_publisher_.publish(point_cloud_map_out);
        /// output as ply file
//        pcl::PCLPointCloud2 point_cloud_out;
//        point_cloud_out.data=point_cloud_map_out.data;
//        pcl::PLYWriter writer;
//        pcl::PCDWriter writer;
//        writer.writeBinary("map_result.pcd", point_cloud_map);
    }
    /**
     * get 2d grid from the Map
     * @param tiles
     */
    void get2dGrid(std::vector<Tiles2D> &tiles){
        map_->fetchTiles(tiles, 0.2);
    }

/**
 * Helper function to obtain a random double in range.
 */
    double fRand(double fMin, double fMax) {
        double f = (double)rand() / RAND_MAX;
        return fMin + f * (fMax - fMin);
    }

/**
 * Creates a Visualization Marker representing a Voxel Map of the environment
 * @param voxels_marker Marker to fill
 * @param voxels 3D Voxel list
 * @param min_weight_th Minimum weight for a voxel to be displayed
 */
    void fillVisualizationMarkerWithVoxels(
            visualization_msgs::Marker &voxels_marker, std::vector<Voxel3D> &voxels,
            int min_weight_th) {

        cv::Mat colorSpace(1, voxels.size(), CV_32FC3);
        if (mapParameters.height_color) {
            for (int i = 0; i < voxels.size(); i++) {
                colorSpace.at<cv::Vec3f>(i)[0] = 180 - (voxels[i].z / 2) * 180;
                colorSpace.at<cv::Vec3f>(i)[1] = 1;
                colorSpace.at<cv::Vec3f>(i)[2] = 1;
            }
            cv::cvtColor(colorSpace, colorSpace, CV_HSV2BGR);
        }

        //ROS_WARN("voxel size %d", voxels.size());
        for (int i = 0; i < voxels.size(); i++) {
            if (voxels[i].data->w < min_weight_th)
                continue;
            //if (i > 1000) 
            //    continue;
            /**
             * Create 3D Point from 3D Voxel
             */
            geometry_msgs::Point point;
            point.x = voxels[i].x;
            point.y = voxels[i].y;
            point.z = voxels[i].z;

            /**
             * Assign Cube Color from Voxel Color
             */
            std_msgs::ColorRGBA color;
            if (mapParameters.height_color) {
                color.r = colorSpace.at<cv::Vec3f>(i)[2];
                color.g = colorSpace.at<cv::Vec3f>(i)[1];
                color.b = colorSpace.at<cv::Vec3f>(i)[0];
            } else {
                color.r = float(voxels[i].data->r) / 255.0;
                color.g = float(voxels[i].data->g) / 255.0;
                color.b = float(voxels[i].data->b) / 255.0;
            }
            color.a = 1;

            voxels_marker.points.push_back(point);
            //ROS_WARN("push point");
            voxels_marker.colors.push_back(color);
            //ROS_WARN("push color");
        }
    }

/**
 * Fills Visualization Marker with 2D Tiles coming from a 2D Query in SkiMap.
 * Represent in a black/white chessboard the occupied/free space respectively
 *
 * @param voxels_marker Marker to fill
 * @param tiles Tiles list
 */
    void fillVisualizationMarkerWithTiles(visualization_msgs::Marker &voxels_marker,
                                          std::vector<Tiles2D> &tiles) {
        for (int i = 0; i < tiles.size(); i++) {

            /**
             * Create 3D Point from 3D Voxel
             */
            geometry_msgs::Point point;
            point.x = tiles[i].x;
            point.y = tiles[i].y;
            point.z = tiles[i].z;

            /**
             * Assign Cube Color from Voxel Color
             */
            std_msgs::ColorRGBA color;
            if (tiles[i].data != NULL) {
                color.r = color.g = color.b =
                        tiles[i].data->w >= mapParameters.min_voxel_weight ? 0.0 : 1.0;
                color.a = 1;
            } else {
                color.r = color.g = color.b = 1.0;
                color.a = 1;
            }

            voxels_marker.points.push_back(point);
            voxels_marker.colors.push_back(color);
        }
    }

/**
 * Creates a "blank" visualization marker with some attributes
 * @param frame_id Base TF Origin for the map points
 * @param time Timestamp for relative message
 * @param id Unique id for marker identification
 * @param type Type of Marker.
 * @return
 */
    visualization_msgs::Marker createVisualizationMarker(std::string frame_id, int id,
                                                         VisualizationType type) {
        ros::Time ros_time = ros::Time::now();
        /**
         * Creating Visualization Marker
         */
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros_time;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = id;

        if (type == VisualizationType::POINT_CLOUD) {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
        } else if (type == VisualizationType::VOXEL_MAP) {
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.scale.x = mapParameters.map_resolution;
            marker.scale.y = mapParameters.map_resolution;
            marker.scale.z = mapParameters.map_resolution;
        } else if (type == VisualizationType::VOXEL_GRID) {
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.scale.x = mapParameters.map_resolution;
            marker.scale.y = mapParameters.map_resolution;
            marker.scale.z = mapParameters.map_resolution;
        }
        return marker;
    }

    void mapPublisher(visualization_msgs::Marker marker){
        map_publisher_.publish(marker);
    }

    void map2dPublisher(visualization_msgs::Marker marker){
        map_2d_publisher_.publish(marker);
    }

    void odomPublisher(nav_msgs::Odometry odom_msgs){
        odom_publisher_.publish(odom_msgs);
    }

    void getRadioPoints(std::vector<Voxel3D> &voxels, Eigen::Matrix4d lidarPose){ //const Eigen::Matrix4f
//        std::vector<Voxel3D> voxels;
        int DIM = 3;
        std::vector<CoordinateType> radius_center(DIM);
        radius_center[0] = lidarPose(0,3);
        radius_center[1] = lidarPose(1,3);
        radius_center[2] = lidarPose(2,3);
        CoordinateType radius_x = 50.0f;
        CoordinateType radius_y = 50.0f;
        CoordinateType radius_z = 20.0f;
        map_->radiusSearch(radius_center[0], radius_center[1], radius_center[2], radius_x, radius_y, radius_z, voxels);
    }

private:
    ros::NodeHandle *nh_;
    SKIMAP *map_;
    ros::Publisher map_2d_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher point_cloud_map_publisher_;
    ros::Publisher odom_publisher_;
    // point cloud frame_id
    std::string map_frame_id_;
};

#endif //PROJECT_SKIMAPINTERFACE_H

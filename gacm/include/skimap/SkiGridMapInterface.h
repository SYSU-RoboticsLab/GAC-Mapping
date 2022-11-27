//
// Created by huyh on 18-12-18.
//

#ifndef PROJECT_SKIGRIDMAPINTERFACE_H
#define PROJECT_SKIGRIDMAPINTERFACE_H

// Ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/SkipListGrid.hpp>
//#include <skimap/voxels/VoxelDataMatrix.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

// OPENCV
#include <opencv2/opencv.hpp>
#include "lidar_slam/mapping/map_viewer.h"

/**
 * skimap Grid2d definition
 */
typedef float CoordinateType;
typedef int16_t IndexType;
typedef uint16_t WeightType;
typedef Eigen::Vector2f PointType;
typedef float CoordType;
//typedef skimap::VoxelDataMatrix<CoordType> VoxelData;
typedef skimap::VoxelDataRGBW<IndexType, WeightType> VoxelDataColor;
typedef skimap::SkipListGrid<VoxelDataColor, IndexType, CoordinateType, 10, 10> SkiGrid;
typedef skimap::SkipListGrid<VoxelDataColor, IndexType, CoordinateType, 10, 10>::Voxel2D Voxel2D;

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

class SkiGridMapInterface{
public:
    /**
     * Construction
     */
    SkiGridMapInterface(){
    }
    /**
     * Deconstruction
     */
    ~SkiGridMapInterface(){
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
        nh->param<int>("min_voxel_weight", mapParameters.min_voxel_weight, 1);
        nh->param<bool>("enable_chisel", mapParameters.enable_chisel, false);
        nh->param<bool>("height_color", mapParameters.height_color, true);
        nh->param<int>("chisel_step", mapParameters.chisel_step, 10);
        nh->param<float>("agent_height", mapParameters.agent_height, 1.0f);

        // Builds the map
        float map_resolution = 0.1;
        map_ = new SkiGrid(map_resolution);

        /**
         * This command enables the Concurrent Access Self Management. If it is
         * enabled
         * you can use OPENMP to call the method 'integrateVoxel' in a concurrent
         * manner safely.
         */
        map_->enableConcurrencyAccess();

        /// publisher initialization
        std::string map_topic =
                nh->param<std::string>("map_publisher_topic", "live_map");
        map_publisher_ = nh->advertise<visualization_msgs::Marker>(map_topic, 1);
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
            map_->integrateVoxel(CoordinateType(x), CoordinateType(y), &voxel);
        }
    }
    /**
     * get Voxel from the SkiMap
     * @param voxels
     */
    void getVoxel(std::vector<Voxel2D> &voxels){
        map_->fetchVoxels(voxels);
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
            visualization_msgs::Marker &voxels_marker, std::vector<Voxel2D> &voxels) {

        cv::Mat colorSpace(1, voxels.size(), CV_32FC3);
        if (mapParameters.height_color) {
            for (int i = 0; i < voxels.size(); i++) {
                colorSpace.at<cv::Vec3f>(i)[0] = 180 - (voxels[i].z / 2) * 180;
                colorSpace.at<cv::Vec3f>(i)[1] = 1;
                colorSpace.at<cv::Vec3f>(i)[2] = 1;
            }
            cv::cvtColor(colorSpace, colorSpace, CV_HSV2BGR);
        }

        for (int i = 0; i < voxels.size(); i++) {

//    if (voxels[i].data->w < min_weight_th)
//      continue;
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
            }
            color.a = 1;

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

    void getRadioPoints(std::vector<Voxel2D> &voxels, const Eigen::Matrix4f lidarPose){
//        std::vector<Voxel3D> voxels;
        int DIM = 2;
        std::vector<CoordinateType> radius_center(DIM);
        radius_center[0] = lidarPose(0,3);
        radius_center[1] = lidarPose(1,3);
        CoordinateType radius_x = 50.0f;
        CoordinateType radius_y = 50.0f;
        map_->radiusSearch(radius_center[0], radius_center[1], radius_x, radius_y, voxels);
    }

private:
    ros::NodeHandle *nh_;
    SkiGrid *map_;
    ros::Publisher map_publisher_;
};

#endif //PROJECT_SKIGRIDMAPINTERFACE_H

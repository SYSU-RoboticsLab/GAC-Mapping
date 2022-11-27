
#ifndef LIDAR_SLAM_CLOUD_H_
#define LIDAR_SLAM_CLOUD_H_

#include <eigen3/Eigen/Dense>
#include <Eigen/Core>

#include "lidar_slam_utils.h"

namespace slam3d {

struct SpecialPoints {
    enum Type{Ground, Plane, Edge, Unknown};
	SpecialPoints() {}
	~SpecialPoints() {}

    inline void clear() {
        points.clear();
        std::vector<Eigen::Vector3f>().swap(normals);
        std::vector<Eigen::Matrix3f>().swap(pointInformationMatrix);
        std::vector<Eigen::Matrix3f>().swap(normalInformationMatrix);
    }

    inline void add(const SpecialPoints& cloud) {
		size_t k = points.size();
        points.resize(k + cloud.points.size());
//        normals.resize(k + cloud.points.size());
        pointInformationMatrix.resize(k + cloud.points.size());
//        normalInformationMatrix.resize(k + cloud.points.size());

        for(int i = 0; i < cloud.points.size(); k++, i++) {
//            points[k] = cloud.points.points[i];
            points[k] = cloud.points[i];
//        	pointInformationMatrix[k] = cloud.pointInformationMatrix[i];	
//			normalInformationMatrix[k] = cloud.normalInformationMatrix[i];
//			normals[k] = cloud.normals[i];
		}
    }

    inline void check(bool no_normal) {
        if (type_ == Ground) {
            std::vector<Eigen::Vector3f>().swap(normals);
            pointInformationMatrix.resize(points.size());
            std::vector<Eigen::Matrix3f>().swap(normalInformationMatrix);
        }

        if (type_ == Plane) {
            normals.resize(points.size());
            pointInformationMatrix.resize(points.size());
            normalInformationMatrix.resize(points.size());
            if (no_normal) {
                for (int i = 0; i < points.size(); i++) {
                    normals[i].setZero();
                }
			}
        }

        if (type_ == Edge || type_ == Unknown) {
            normals.resize(points.size());
            pointInformationMatrix.resize(points.size());
            std::vector<Eigen::Matrix3f>().swap(normalInformationMatrix);
            if (no_normal) {
                for (int i = 0; i < points.size(); i++) {
                    normals[i].setZero();
                }
			}
        }

    }

	PointCloudT points;
	std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Matrix3f> pointInformationMatrix;
    std::vector<Eigen::Matrix3f> normalInformationMatrix;
	std::vector<int> view_counts;

    Type type_;
};

class Cloud {
public:
    Cloud(void);
    ~Cloud();

	void Clear(void);
    void TransformInplace(Eigen::Isometry3f T);
    Cloud Transform(Eigen::Isometry3f T) const;
    void TransformInplace(Eigen::Matrix3d R, Eigen::Vector3d T);
    void TransformInplace(Eigen::Matrix3d R);
    void Add(const Cloud& cloud);

	void TransformPointCloud(const Eigen::Isometry3f& T, PointCloudT& pts);
	void TransformPointCloud(const Eigen::Isometry3f& T,
			const PointCloudT& in_pts, PointCloudT& out_pts) const;
	
    SpecialPoints grounds_;
    SpecialPoints planes_;
    SpecialPoints edges_;
    SpecialPoints unknowns_;
	int index_;
};

}  // namespace slam3d

#endif  // LIDAR_SLAM_CLOUD_H_

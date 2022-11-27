#ifndef POSEGRAPHSTRUCTURE_H
#define POSEGRAPHSTRUCTURE_H
#include "poseFactor.h"

class PoseNode {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double stamp;
        int frame_id; // identify which robot and which frame TODO
        Eigen::Quaterniond q;   
        Eigen::Vector3d  t;
        PoseNode() {
            stamp = 0;
            frame_id = -1;
            q = Eigen::Quaterniond::Identity();
            t << 0,0,0;
        }
        PoseNode(const double& stamp_, const int& frame_id_, const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_) {
            stamp = stamp_;
            frame_id = frame_id_;
            q = q_;
            t = t_;
        }
};

class MeasurementEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double stamp_from;
        double stamp_to;
        int robot_from;
        int robot_to;
        int submap_from;
        int submap_to;
        int index_from;
        int index_to;
        Eigen::Quaterniond q; // TODO: frame definition? q_ab or q_ba?
        Eigen::Vector3d t;

        MeasurementEdge() {
            stamp_from = 0;
            stamp_to = 0;
            robot_from = -1;
            robot_to = -1;
            submap_from = -1;
            submap_to = -1;
            index_from = -1;
            index_to = -1;
            q = Eigen::Quaterniond::Identity();
            t << 0, 0, 0;
        }

        MeasurementEdge(const double& stamp_from_, const double& stamp_to_,
                        const int& robot_from_, const int& robot_to_,
                        const int& submap_from_, const int& submap_to_,
                        const int& index_from_, const int& index_to_,
                        const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_) {
            stamp_from = stamp_from_;
            stamp_to = stamp_to_;
            robot_from = robot_from_;
            robot_to = robot_to_;
            submap_from = submap_from_;
            submap_to = submap_to_;
            index_from = index_from_;
            index_to = index_to_;
            q = q_;
            t = t_;
        }

        bool operator == (const MeasurementEdge& B) {
            if (this->robot_from == B.robot_from && this->robot_to == B.robot_to) {
                if (this->submap_from == B.submap_from && this->submap_to == B.submap_to) {
                    return true;
                }
            }

            if (this->robot_from == B.robot_to && this->robot_to == B.robot_from) {
                if (this->submap_from == B.submap_to && this->submap_to == B.submap_from) {
                    return true;
                }
            }
            return false;
        }
};
#endif
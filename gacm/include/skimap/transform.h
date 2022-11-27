#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

class Transform3D {
public:
    Transform3D(){ translation_.setZero(); rotation_.setIdentity();}
    Transform3D(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
            :translation_(translation), rotation_(rotation){}

    Transform3D(Eigen::Vector3d translation)
            :translation_(translation){
            rotation_.setIdentity();
    }
    Transform3D(Eigen::Quaterniond rotation)
            :rotation_(rotation){
            translation_.setZero();
    }

    Transform3D(Eigen::Isometry3d T)
            :rotation_(T.linear()), translation_(T.translation()) {}

    Transform3D operator *(Transform3D in) const {
        Transform3D temp;
        temp.rotation_ = rotation_ * in.rotation_;
        temp.translation_ = rotation_ * in.translation_ + translation_;
        return temp;
    }

    void setIdentify(){
        translation_.setZero();
        rotation_.setIdentity();
    }

    double getAngle(Eigen::Quaterniond& in) {
        return std::atan2(in.vec().norm(), std::abs(in.w()));
    }

    Transform3D inverse() const {
        return Transform3D(- (rotation_.conjugate() * translation_), rotation_.conjugate());
    }

    Eigen::Isometry3d Trans2Iso() const{
        Eigen::Isometry3d temp;
        temp.setIdentity();
        temp.prerotate(rotation_.matrix());
        temp.pretranslate(translation_);
        return temp;
    }

    Eigen::Isometry3f Trans2Iso_f() const{
        Eigen::Isometry3f temp;
        temp.setIdentity();
        temp.prerotate(rotation_.matrix().cast<float>());
        temp.pretranslate(translation_.cast<float>());
        return temp;
    }

	Eigen::Matrix4f getPose() const {
		Eigen::Matrix3d R = rotation_.toRotationMatrix();
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 3; ++c) {
				pose(r, c) = R(r, c);
			}
		}
		pose(0, 3) = translation_[0];
		pose(1, 3) = translation_[1];
		pose(2, 3) = translation_[2];
		return pose;
	}

	void setPose(const Eigen::Matrix4f& pose) {
		translation_[0] = pose(0, 3);	
		translation_[1] = pose(1, 3);	
		translation_[2] = pose(2, 3);	
		rotation_ = pose.block<3, 3>(0, 0).cast<double>();
	}

	void setPoseD(const Eigen::Matrix4d& pose) {
		translation_[0] = pose(0, 3);	
		translation_[1] = pose(1, 3);	
		translation_[2] = pose(2, 3);	
		rotation_ = pose.block<3, 3>(0, 0);
	}

    Eigen::Vector3d translation_;
    Eigen::Quaterniond rotation_;
};

template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
        const Eigen::Quaternion<T>& quaternion) {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    // We choose the quaternion with positive 'w', i.e., the one with a smaller
    // angle that represents this orientation.
    if (normalized_quaternion.w() < 0.) {
        normalized_quaternion.w() *= T(-1.);
        normalized_quaternion.x() *= T(-1.);
        normalized_quaternion.y() *= T(-1.);
        normalized_quaternion.z() *= T(-1.);
    }
    // We convert the normalized_quaternion into a vector along the rotation axis
    // with length of the rotation angle.
    const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                  normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
}

#endif //PROJECT_TRANSFORM_H

#ifndef POSEFACTOR_H
#define POSEFACTOR_H
#include <ceres/ceres.h>

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (t_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraph3dErrorTerm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseGraph3dErrorTerm(const Eigen::Quaterniond& q_ab_measured,
                       const Eigen::Vector3d& t_ab_measured,
                       const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : q_ab_measured_(q_ab_measured), t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()( const T* const q_a_ptr, const T* const t_a_ptr,
                   const T* const q_b_ptr, const T* const t_b_ptr,
                  T* residuals_ptr) const {
    Eigen::Matrix<T, 3, 1> t_a{t_a_ptr[0], t_a_ptr[1], t_a_ptr[2]};
    Eigen::Quaternion<T> q_a{q_a_ptr[3], q_a_ptr[0], q_a_ptr[1], q_a_ptr[2]};

    Eigen::Matrix<T, 3, 1>  t_b{t_b_ptr[0], t_b_ptr[1], t_b_ptr[2]};
    Eigen::Quaternion<T> q_b{q_b_ptr[3], q_b_ptr[0], q_b_ptr[1], q_b_ptr[2]};

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> t_ab_estimated = q_a_inverse * (t_b - t_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        q_ab_measured_.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        t_ab_estimated - t_ab_measured_.template cast<T>();
    residuals.template block<3, 1>(3, 0) =  T(2)*delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Quaterniond& q_ab_measured,
      const Eigen::Vector3d& t_ab_measured,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 4, 3, 4, 3>(
        new PoseGraph3dErrorTerm(q_ab_measured, t_ab_measured, sqrt_information));
  }

  

 private:
  const Eigen::Quaterniond q_ab_measured_;
  // The measurement for the position of B relative to A in the A frame.
  const Eigen::Vector3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};


class PoseGraph3dErrorTermWorld {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseGraph3dErrorTermWorld(const Eigen::Quaterniond& q_ab_measured,
                       const Eigen::Vector3d& t_ab_measured,
                       const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : q_ab_measured_(q_ab_measured), t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()( const T* const q_a_ptr, const T* const t_a_ptr,
                   const T* const q_wa_ptr, const T* const t_wa_ptr, 
                   const T* const q_b_ptr, const T* const t_b_ptr,
                   const T* const q_wb_ptr, const T* const t_wb_ptr,
                  T* residuals_ptr) const {
    Eigen::Matrix<T, 3, 1> t_a{t_a_ptr[0], t_a_ptr[1], t_a_ptr[2]};
    Eigen::Quaternion<T> q_a{q_a_ptr[3], q_a_ptr[0], q_a_ptr[1], q_a_ptr[2]};
    Eigen::Matrix<T, 3, 1> t_wa{t_wa_ptr[0], t_wa_ptr[1], t_wa_ptr[2]};
    Eigen::Quaternion<T> q_wa{q_wa_ptr[3], q_wa_ptr[0], q_wa_ptr[1], q_wa_ptr[2]};
    t_a = t_wa + q_wa*t_a;
    q_a = q_wa * q_a;

    Eigen::Matrix<T, 3, 1>  t_b{t_b_ptr[0], t_b_ptr[1], t_b_ptr[2]};
    Eigen::Quaternion<T> q_b{q_b_ptr[3], q_b_ptr[0], q_b_ptr[1], q_b_ptr[2]};
    Eigen::Matrix<T, 3, 1>  t_wb{t_wb_ptr[0], t_wb_ptr[1], t_wb_ptr[2]};
    Eigen::Quaternion<T> q_wb{q_wb_ptr[3], q_wb_ptr[0], q_wb_ptr[1], q_wb_ptr[2]};
    t_b = t_wb + q_wb*t_b;
    q_b = q_wb * q_b;

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> t_ab_estimated = q_a_inverse * (t_b - t_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        q_ab_measured_.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        t_ab_estimated - t_ab_measured_.template cast<T>();
    residuals.template block<3, 1>(3, 0) =  T(2)*delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Quaterniond& q_ab_measured,
      const Eigen::Vector3d& t_ab_measured,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTermWorld, 6, 4, 3, 4, 3, 4, 3, 4, 3>(
        new PoseGraph3dErrorTermWorld(q_ab_measured, t_ab_measured, sqrt_information));
  }

  

 private:
  const Eigen::Quaterniond q_ab_measured_;
  // The measurement for the position of B relative to A in the A frame.
  const Eigen::Vector3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};



// optimize (q|t)
class PoseGraph3dErrorTerm2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseGraph3dErrorTerm2(
                       const Eigen::Matrix<double,7,1>& rel_ab_measured,
                       const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : rel_ab_measured_(rel_ab_measured), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()( const T* pose_a_ptr ,
                   const T* pose_b_ptr,
                  T* residuals_ptr) const {
    Eigen::Quaternion<T> q_a{pose_a_ptr[3], pose_a_ptr[0], pose_a_ptr[1], pose_a_ptr[2]};
    Eigen::Matrix<T, 3, 1> t_a{pose_a_ptr[4], pose_a_ptr[5], pose_a_ptr[6]};

    Eigen::Quaternion<T> q_b{pose_b_ptr[3], pose_b_ptr[0], pose_b_ptr[1], pose_b_ptr[2]};
    Eigen::Matrix<T, 3, 1> t_b{pose_b_ptr[4], pose_b_ptr[5], pose_b_ptr[6]};

    Eigen::Quaternion<double> q_ab_measured_{rel_ab_measured_[3], rel_ab_measured_[0], rel_ab_measured_[1], rel_ab_measured_[2]};
    Eigen::Matrix<double, 3, 1> t_ab_measured_{rel_ab_measured_[4], rel_ab_measured_[5], rel_ab_measured_[6]};

    

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> t_ab_estimated = q_a_inverse * (t_b - t_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        q_ab_measured_.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        t_ab_estimated - t_ab_measured_.template cast<T>();
    residuals.template block<3, 1>(3, 0) =  T(2)*delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix<double, 7, 1>& pose_ab_measured,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm2, 6, 7, 7>(
        new PoseGraph3dErrorTerm2(pose_ab_measured, sqrt_information));
  }

  

 private:
  const Eigen::Matrix<double, 7, 1> rel_ab_measured_;
 
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};
#endif
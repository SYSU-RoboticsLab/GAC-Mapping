

#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename T>
void templateSpaceToPlane(const std::vector<T>& intrinsic_params, const Eigen::Matrix<T, 3, 1>& P_c, Eigen::Matrix<T, 2, 1>& p) {
	// TODO: undistort the input image at the beginning of the system to ignore distortion in the projection phase
	T k1 = intrinsic_params[0];
	T k2 = intrinsic_params[1];
	T p1 = intrinsic_params[2];
	T p2 = intrinsic_params[3];
	T fx = intrinsic_params[4];
	T fy = intrinsic_params[5];
	T alpha = T(0); //cameraParams.alpha();
	T cx = intrinsic_params[6];
	T cy = intrinsic_params[7];

	// Transform to model plane
	T u = P_c[0] / P_c[2];
	T v = P_c[1] / P_c[2];

	T rho_sqr = u * u + v * v;
	T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
	T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
	T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

	T u_corrected = u*(L) + du;
	T v_corrected = v*(L) + dv;
	
	p(0) = fx * (u_corrected + alpha * v) + cx;
	p(1) = fy * v_corrected + cy;
}

template <typename T>
void templateLiftProjective(const std::vector<T>& intrinsic_params, const Eigen::Matrix<T,2,1>&p , Eigen::Matrix<T,3,1>& P) {
	T k1 = intrinsic_params[0];
	T k2 = intrinsic_params[1];
	T p1 = intrinsic_params[2];
	T p2 = intrinsic_params[3];
	T fx = intrinsic_params[4];
	T fy = intrinsic_params[5];
	T alpha = T(0); //cameraParams.alpha();
	T cx = intrinsic_params[6];
	T cy = intrinsic_params[7];

	Eigen::Matrix<T,3,3> K;
	K << fx, (T)0, cx,
		(T)0, fy, cy,
		(T)0, (T)0, (T)1;
	P << p, (T)1;
	// ROS_ERROR_STREAM("Before \n" << P.transpose() << "\n");
	P = K.inverse()*P;
	// ROS_ERROR_STREAM("After \n" << P.transpose() << "\n");
}
// point to line error
// optimize relative pose (q,t)
struct LidarEdgeFactor
{
	LidarEdgeFactor(const Eigen::Vector3d &curr_point_, const Eigen::Vector3d & last_point_a_,
					const Eigen::Vector3d &last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d & curr_point_, const Eigen::Vector3d & last_point_a_,
									   const Eigen::Vector3d & last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

// point to line error
// optimize relative pose (q|t)
struct LidarEdgeFactor1
{
	LidarEdgeFactor1(const Eigen::Vector3d &curr_point_, const Eigen::Vector3d & last_point_a_,
					const Eigen::Vector3d &last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *pose, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{pose[3], pose[0], pose[1], pose[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * pose[4], T(s) * pose[5], T(s) * pose[6]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d & curr_point_, const Eigen::Vector3d & last_point_a_,
									   const Eigen::Vector3d & last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor1, 3, 7>(
			new LidarEdgeFactor1(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

// optimize relative pose (q,t)
struct LidarPlaneFactor
{
	LidarPlaneFactor(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d & last_point_j_,
					 const Eigen::Vector3d& last_point_l_, const Eigen::Vector3d & last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d& last_point_j_,
									   const Eigen::Vector3d& last_point_l_, const Eigen::Vector3d& last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

// optimize relative pose (q|t)
struct LidarPlaneFactor1
{
	LidarPlaneFactor1(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d & last_point_j_,
					 const Eigen::Vector3d& last_point_l_, const Eigen::Vector3d & last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *pose, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{pose[3], pose[0], pose[1], pose[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * pose[4], T(s) * pose[5], T(s) * pose[6]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d& last_point_j_,
									   const Eigen::Vector3d& last_point_l_, const Eigen::Vector3d& last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor1, 1, 7>(
			new LidarPlaneFactor1(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d& plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d & curr_point_, const Eigen::Vector3d& plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


struct LidarDistanceFactor
{

	LidarDistanceFactor(const Eigen::Vector3d & curr_point_,const Eigen::Vector3d & closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;


		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d & curr_point_, const Eigen::Vector3d& closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};




// 3d-3d point distance error
// optimize relative pose (q,t)
struct PointPosition33FactorQT
{	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PointPosition33FactorQT(const Eigen::Vector3d& map_point_, const Eigen::Vector3d& observed_point_, const float& weight_)
								: map_point(map_point_), observed_point(observed_point_), weight(weight_) {}
	
	
	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// Eigen::Quaterniond qrcl(RCL2);
		// Eigen::Quaternion<T> qrcl2{T(qrcl.w()), T(qrcl.x()) , T(qrcl.y()), T(qrcl.z())};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		// q_last_curr = qrcl2*q_last_curr;
		Eigen::Matrix<T, 3, 1> t_last_curr{ t[0], t[1], t[2]};
		// Eigen::Matrix<T, 3, 1> t_last_curr{ -t[1], -t[2], t[0]};
		// t_last_curr = RCL2*t_last_curr;

		// Eigen::Matrix<T, 3, 1> last_point{T(map_point(2)), T(-1*map_point(0)),T(-1*map_point(1))}; // transform to lidar coordinate
		Eigen::Matrix<T, 3, 1> last_point{T(map_point(0)), T(map_point(1)),T(map_point(2))}; // transform to lidar coordinate
		Eigen::Matrix<T, 3, 1> P_cl;
		// Eigen::Matrix<T, 2, 1> p;
		// transform last map point to cur frame (under lidar cooridinate)
		P_cl = q_last_curr.inverse()*(last_point-t_last_curr); // sensor coordinate moving forward , point coordinate moving backward
		// P_cl = q_last_curr*last_point+t_last_curr;
		// transform the point back to camera sensor coordinate
		// Eigen::Matrix<T, 3, 1> P_c{T(-1)*T(P_cl(1,0)), T(-1)*T(P_cl(2,0)), T(P_cl(0,0))};
		Eigen::Matrix<T, 3, 1> P_c{T(P_cl(0,0)), T(P_cl(1,0)), T(P_cl(2,0))};
		Eigen::Matrix<T, 3, 1> A {T(observed_point(0)),T(observed_point(1)), T(observed_point(2))};
		
		Eigen::Matrix<T, 3, 1> PA = A-P_c;//{T(observed_point(0)-P_c(0), T(observed_point(1)-P_c(1), T(observed_point(2)-P_c(2)};
		Eigen::Matrix<T, 3, 1> PO = T(-1)*P_c;
		Eigen::Matrix<T, 3, 1> area = PA.cross(PO);

		// residual[0] = area.norm()/A.norm()* T(weight);
		residual[0] = area.x()/A.norm()* T(weight);
		residual[1] = area.y()/A.norm()* T(weight);
		residual[2] = area.z()/A.norm()* T(weight);
		// residual[0] = (P_c(0) - T(observed_point(0)));
		// residual[1] = (P_c(1) - T(observed_point(1)));
		// residual[2] = (P_c(2) - T(observed_point(2)));
		
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d& map_point_, const Eigen::Vector3d& observed_point_, const float& weight_)
	{
		return (new ceres::AutoDiffCostFunction<
				PointPosition33FactorQT, 3, 4,3>(
			new PointPosition33FactorQT(map_point_, observed_point_, weight_)));
	}
	
	
	Eigen::Vector3d map_point;
	Eigen::Vector3d observed_point;
	float weight;
};




// 3d-2d reprojection error
// optimize relative pose(q|t)
struct ReprojectionPoint32Factor1
{	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ReprojectionPoint32Factor1(std::vector<double> m_intrinsic_params_, const Eigen::Vector3d& map_point_, const Eigen::Vector2d& observed_point_)
								:m_intrinsic_params(m_intrinsic_params_), map_point(map_point_), observed_point(observed_point_) {}
	
	
	template <typename T>
	bool operator()(const T *pose, T *residual) const
	{
		// Eigen::Quaterniond qrcl(RCL2);
		// Eigen::Quaternion<T> qrcl2{T(qrcl.w()), T(qrcl.x()) , T(qrcl.y()), T(qrcl.z())};
		Eigen::Quaternion<T> q_last_curr{pose[3], pose[0], pose[1], pose[2]};
		// q_last_curr = qrcl2*q_last_curr;
		Eigen::Matrix<T, 3, 1> t_last_curr{ pose[4], pose[5], pose[6]};
		// Eigen::Matrix<T, 3, 1> t_last_curr{ -t[1], -t[2], t[0]};
		// t_last_curr = RCL2*t_last_curr;

		// Eigen::Matrix<T, 3, 1> last_point{T(map_point(2)), T(-1*map_point(0)),T(-1*map_point(1))}; // transform to lidar coordinate
		Eigen::Matrix<T, 3, 1> last_point{T(map_point(0)), T(map_point(1)),T(map_point(2))}; // transform to lidar coordinate
		Eigen::Matrix<T, 3, 1> P_cl;
		Eigen::Matrix<T, 2, 1> p;
		// transform last map point to cur frame (under lidar cooridinate)
		P_cl = q_last_curr.inverse()*(last_point-t_last_curr); // sensor coordinate moving forward , point coordinate moving backward
		// P_cl = q_last_curr*last_point+t_last_curr;
		// transform the point back to camera sensor coordinate
		// Eigen::Matrix<T, 3, 1> P_c{T(-1)*T(P_cl(1,0)), T(-1)*T(P_cl(2,0)), T(P_cl(0,0))};
		Eigen::Matrix<T, 3, 1> P_c{T(P_cl(0,0)), T(P_cl(1,0)), T(P_cl(2,0))};


		// project 3D object point to the image plane
		std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
		templateSpaceToPlane(intrinsic_params, P_c, p);

		// T k1 = intrinsic_params[0];
		// T k2 = intrinsic_params[1];
		// T p1 = intrinsic_params[2];
		// T p2 = intrinsic_params[3];
		// T fx = intrinsic_params[4];
		// T fy = intrinsic_params[5];
		// T alpha = T(0); //cameraParams.alpha();
		// T cx = intrinsic_params[6];
		// T cy = intrinsic_params[7];

		// // Transform to model plane
		// T u = P_c[0] / P_c[2];
		// T v = P_c[1] / P_c[2];

		// T rho_sqr = u * u + v * v;
		// T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
		// T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
		// T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

		// u = L * u + du;
		// v = L * v + dv;
		// p(0) = fx * (u + alpha * v) + cx;
		// p(1) = fy * v + cy;

		residual[0] = (p(0) - T(observed_point(0)));
		residual[1] = (p(1) - T(observed_point(1)));
		return true;
	}

	static ceres::CostFunction *Create(const std::vector<double> m_intrinsic_params_, const Eigen::Vector3d& map_point_, const Eigen::Vector2d& observed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				ReprojectionPoint32Factor1, 2, 7>(
			new ReprojectionPoint32Factor1(m_intrinsic_params_, map_point_, observed_point_)));
	}
	
	std::vector<double> m_intrinsic_params;
	Eigen::Vector3d map_point;
	Eigen::Vector2d observed_point;
};




// 3d-2d reprojection error
// optimize first depth pose
// optimize: pose(q|t), feature inverse depth(1/d)
struct ReprojectionPoint32Factor3new
{	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ReprojectionPoint32Factor3new(std::vector<double> m_intrinsic_params_, const Eigen::Vector2d& observed_point_,
							 const Eigen::Vector3d& first_depth_point_/*, const Eigen::Matrix<double, 7,1> first_depth_pose_*//*, const Eigen::Quaterniond& first_depth_q_, const Eigen::Vector3d& first_depth_t_*/)
								: m_intrinsic_params(m_intrinsic_params_), observed_point(observed_point_),
									 first_depth_point(first_depth_point_)/*,pose_fd(first_depth_pose_)*//*, first_depth_q(first_depth_q_), first_depth_t(first_depth_t_)*/ {}
	
	
	template <typename T>
	bool operator()(const T *pose_fd, const T *pose, const T *d, T *residual) const
	{

		// Eigen::Map<const Eigen::Quaternion<T> > q_w_curr(q);
		// Eigen::Map<const Eigen::Matrix<T, 3, 1> > t_w_curr(t);
		// Eigen::Quaternion<T> fdq{(T)pose_fd[3], (T)pose_fd[0], (T)pose_fd[1], (T)pose_fd[2]};
		// Eigen::Matrix<T, 3, 1> fdt{ (T)pose_fd[4], (T)pose_fd[5], (T)pose_fd[6]};
		Eigen::Quaternion<T> fdq{(T)pose_fd[3], (T)pose_fd[0], (T)pose_fd[1], (T)pose_fd[2]};
		Eigen::Matrix<T, 3, 1> fdt{ (T)pose_fd[4], (T)pose_fd[5], (T)pose_fd[6]};
		Eigen::Quaternion<T> q_w_curr{pose[3], pose[0], pose[1], pose[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{ pose[4], pose[5], pose[6]};
		T depth = (T)1.0/d[0];

		Eigen::Matrix<T, 3, 1> P_w;
		Eigen::Matrix<T, 3, 1> P_c_fd;
		P_c_fd(0) = depth*(T)first_depth_point(0);
		P_c_fd(1) = depth*(T)first_depth_point(1);
		P_c_fd(2) = depth*(T)first_depth_point(2);
		// Eigen::Quaternion<T> fdq{(T)first_depth_q.w(), (T)first_depth_q.x(), (T)first_depth_q.y(), (T)first_depth_q.z()};
		// Eigen::Matrix<T, 3, 1> fdt{(T)first_depth_t.x(),(T)first_depth_t.y(),(T)first_depth_t.z()};
		P_w = fdq*P_c_fd+fdt; // transform to world frame

		Eigen::Matrix<T, 3, 1> P_c;
		Eigen::Matrix<T, 2, 1> p;
		P_c = q_w_curr.inverse()*(P_w-t_w_curr); // transform to current frame


		// project 3D object point to the image plane
		std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());

		T k1 = intrinsic_params[0];
		T k2 = intrinsic_params[1];
		T p1 = intrinsic_params[2];
		T p2 = intrinsic_params[3];
		T fx = intrinsic_params[4];
		T fy = intrinsic_params[5];
		T alpha = T(0); //cameraParams.alpha();
		T cx = intrinsic_params[6];
		T cy = intrinsic_params[7];
		// if ((double)P_c[2] <= 0 ) return false;
		// Transform to model plane
		T u = P_c[0] / P_c[2];
		T v = P_c[1] / P_c[2];

		T rho_sqr = u * u + v * v;
		T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
		T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
		T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

		u = L * u + du;
		v = L * v + dv;
		p(0) = fx * (u + alpha * v) + cx;
		p(1) = fy * v + cy;

		residual[0] = p(0) - T(observed_point(0));
		residual[1] = p(1) - T(observed_point(1));
		
		return true;
	}

	static ceres::CostFunction *Create(const std::vector<double> m_intrinsic_params_, const Eigen::Vector2d& observed_point_,
					 const Eigen::Vector3d& first_track_point_/*, const Eigen::Matrix<double, 7,1>& first_depth_pose_*//*,const Eigen::Quaterniond& first_depth_q_, const Eigen::Vector3d& first_depth_t_*/) {
		return (new ceres::AutoDiffCostFunction<
				ReprojectionPoint32Factor3new, 2,7, 7, 1>(
			new ReprojectionPoint32Factor3new(m_intrinsic_params_, observed_point_,first_track_point_ /*, first_depth_pose_*//*, first_depth_q_, first_depth_t_*/)));
	}
	
	std::vector<double> m_intrinsic_params;
	Eigen::Vector2d observed_point; // observation u,v
	Eigen::Vector3d first_depth_point; // observation (x0, y0, 1)
	// Eigen::Matrix<double,7,1> pose_fd;
	// Eigen::Quaterniond first_depth_q;
	// Eigen::Vector3d first_depth_t;
	
};



struct Pose3DFactor
{	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Pose3DFactor(std::vector<double> m_intrinsic_params_, const Eigen::Vector2d& observed_point_)
								: m_intrinsic_params(m_intrinsic_params_), observed_point(observed_point_) {}
	
	
	template <typename T>
	bool operator()(const T *q, const T *t, const T *point, T *residual) const
	{
		// Eigen::Quaterniond qrcl(RCL2);
		// Eigen::Quaternion<T> qrcl2{T(qrcl.w()), T(qrcl.x()) , T(qrcl.y()), T(qrcl.z())};
		// Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Map<const Eigen::Quaternion<T> > q_w_curr(q);
		// q_last_curr = qrcl2*q_last_curr;
		// Eigen::Matrix<T, 3, 1> t_w_curr{ t[0], t[1], t[2]};
		Eigen::Map<const Eigen::Matrix<T, 3, 1> > t_w_curr(t);
		
		// Eigen::Matrix<T, 3, 1> t_last_curr{ -t[1], -t[2], t[0]};
		
		// t_last_curr = RCL2*t_last_curr;

		// Eigen::Matrix<T, 3, 1> last_point{T(map_point(2)), T(-1*map_point(0)),T(-1*map_point(1))}; // transform to lidar coordinate
		Eigen::Map<const Eigen::Matrix<T, 3, 1> > point_(point);
		Eigen::Matrix<T, 3, 1> P_c = point_;// transform to lidar coordinate
		
		// Eigen::Matrix<T, 3, 1> P_c{T(map_point[0]), T(map_point[1]), T(map_point[2])};
		Eigen::Matrix<T, 2, 1> p;
		// transform last map point to cur frame (under lidar cooridinate)
		// P_cl = q_last_curr.inverse()*(last_point-t_last_curr);
		P_c = q_w_curr.inverse()*(P_c-t_w_curr);
		// P_c = P_c/P_c(2)*(depth[0]);


		// project 3D object point to the image plane
		std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());

		T k1 = intrinsic_params[0];
		T k2 = intrinsic_params[1];
		T p1 = intrinsic_params[2];
		T p2 = intrinsic_params[3];
		T fx = intrinsic_params[4];
		T fy = intrinsic_params[5];
		T alpha = T(0); //cameraParams.alpha();
		T cx = intrinsic_params[6];
		T cy = intrinsic_params[7];
		// if ((double)P_c[2] <= 0 ) return false;
		// Transform to model plane
		T u = P_c[0] / P_c[2];
		T v = P_c[1] / P_c[2];

		T rho_sqr = u * u + v * v;
		T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
		T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
		T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

		u = L * u + du;
		v = L * v + dv;
		p(0) = fx * (u + alpha * v) + cx;
		p(1) = fy * v + cy;

		residual[0] = p(0) - T(observed_point(0));
		residual[1] = p(1) - T(observed_point(1));
		
		return true;
	}

	static ceres::CostFunction *Create(const std::vector<double> m_intrinsic_params_, const Eigen::Vector2d& observed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				Pose3DFactor, 2, 4, 3, 3>(
			new Pose3DFactor(m_intrinsic_params_, observed_point_)));
	}
	
	std::vector<double> m_intrinsic_params;
	Eigen::Vector2d observed_point;
};







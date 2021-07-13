/*
 * MotionModel.hpp
 *
 *  Created on: Dec 25, 2018
 *      Author: KaiJin Ji
 */
#include "Eigen/Core"
#include "Eigen/Geometry"
#ifndef COVGRID_SLAM_MOTIONMODEL_HPP_
#define COVGRID_SLAM_MOTIONMODEL_HPP_
namespace MotionModel{
/// \brief 四元数转角轴向量
///
/// 四元数转角轴向量
/// \param quaternion 四元数
/// \return 角轴向量
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
  const Eigen::Quaternion<T>& quaternion) {
	Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
	// We choose the quaternion with positive 'w', i.e., the one with a smaller
	// angle that represents this orientation.
	if (normalized_quaternion.w() < 0.) {
	  // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
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

/// \brief 角轴向量转四元数
///
/// 角轴向量转四元数
/// \param angle_axis 角轴向量
/// \return 四元数
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
  const Eigen::Matrix<T, 3, 1>& angle_axis) {
	T scale = T(0.5);
	T w = T(1.);
	constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
	if (angle_axis.squaredNorm() > kCutoffAngle) {
	  const T norm = angle_axis.norm();
	  scale = sin(norm / 2.) / norm;
	  w = cos(norm / 2.);
	}
	const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
	return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
								quaternion_xyz.z());
}

/// \brief 匀速模型
///
/// 匀速模型
/// \param first_rot 上上帧姿态
/// \param second_rot 上一帧姿态
/// \param second_trans 上一帧位置
/// \param timediff1 上上帧与上一帧之间的时间差
/// \param timediff2 上一帧与当前帧之间的时间差
/// \param linear_velocity 车体坐标下车辆速度
/// \param out_rot out预测车辆姿态
/// \param out_trans out预测车辆位置
void MotionModelConstVelocity(const Eigen::Quaterniond first_rot,const Eigen::Quaterniond second_rot,const Eigen::Vector3d second_trans,
		const double timediff1,const double timediff2,const Eigen::Vector3d linear_velocity/*vehicle frame*/,
		Eigen::Quaterniond& out_rot,Eigen::Vector3d& out_trans)
{
	Eigen::Vector3d angular_velocity = RotationQuaternionToAngleAxisVector(first_rot.inverse()*second_rot)/timediff1;
	Eigen::Vector3d angular_changed = angular_velocity * timediff2;
	Eigen::Vector3d linear_changed = linear_velocity * timediff2;
	Eigen::Quaterniond rotation = AngleAxisVectorToRotationQuaternion(angular_changed);
	out_rot = second_rot * rotation;
	out_trans = second_trans + second_rot * linear_changed;
}


}//namespace MotionModel
#endif /* COVGRID_SLAM_MOTIONMODEL_HPP_ */

/*
 * distortioncorrection.hpp
 *
 *  Created on: Dec 7, 2018
 *      Author: KaiJin Ji
 */

#ifndef COVGRID_SLAM_DISTORTIONCORRECTION_HPP_
#define COVGRID_SLAM_DISTORTIONCORRECTION_HPP_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace DistortionCorrection{
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
/// \brief 矫正点云运动畸变
///
/// 矫正点云运动畸变
/// \param pcl_point_cloud 输入点云
/// \param linearvelocity 线速度 0 1 2分别对应x y z轴的线速度
/// \param angularvelocity 角轴向量形式的速度 模代表速度大小，nomalize后代表角轴放心
/// \return 矫正运动畸变后的点云
template <class PointT>
  pcl::PointCloud<PointT>
  distortionCorrection(const pcl::PointCloud<PointT>& pcl_point_cloud,
		  const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity)
  {
	  auto result = pcl_point_cloud;

	  double starttime = result.front().timestamp;
	  double backtime = result.back().timestamp;
	  double scale_threshold = 0.9;
	  double period_time = 0.1;
	  if((backtime - starttime) < period_time*scale_threshold)
	  {
		  double maxtime = backtime;
		  for(auto& point:result)
		  {
			  if(maxtime < point.timestamp)
				  maxtime = point.timestamp;
		  }
		  backtime = maxtime;
	  }
	  for(auto& point:result)
	  {
		  double deltat = backtime - point.timestamp;
//		  CHECK_GE(deltat,0);

		  Eigen::Vector3d translation = linearvelocity * deltat;
		  Eigen::Vector3d angle = angularvelocity * deltat;

		  auto rotation = AngleAxisVectorToRotationQuaternion<double>(angle);
		  point.getVector3fMap() = ( rotation.inverse().cast<float>()*(point.getVector3fMap() - translation.cast<float>()));
	  }

	  return result;
  }
/// \brief 矫正点云运动畸变
///
/// 矫正点云运动畸变
/// \param pcl_point_cloud 输入点云
/// \param linearvelocity 线速度 0 1 2分别对应x y z轴的线速度
/// \param angularvelocity 角轴向量形式的速度 模代表速度大小，nomalize后代表角轴放心
/// \return 矫正运动畸变后的点云
template <class PointT>
  pcl::PointCloud<PointT>
  distortionCorrection64(const pcl::PointCloud<PointT>& pcl_point_cloud,
		  const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity,const float period)
  {

	  auto result = pcl_point_cloud;
	  LOG(INFO)<<linearvelocity;
	  int count = 0;
	  for(auto& point:result)
	  {
	      double deg = atan2(point.x, point.y) * 180 / M_PI; // -p.y, p.x
	      if(deg < 0){
	    	  deg += 360;
	      }
	      float scale = deg / 360.0;
		  double deltat = period * (1 - scale);
		  if(count < result.size()/2&&deltat<period*0.1)
			  deltat += period;
		  else if(count > result.size()/2&&deltat>period*0.9)
			  deltat -= period;

//		  CHECK_GE(deltat,0);
//		  if(point.ring==10)
//		  std::cout<<point.ring<<" "<<deltat<<std::endl;
		  Eigen::Vector3d translation = linearvelocity * deltat;
		  Eigen::Vector3d angle = angularvelocity * deltat;

		  auto rotation = AngleAxisVectorToRotationQuaternion<double>(angle);
		  point.getVector3fMap() = ( rotation.inverse().cast<float>()*(point.getVector3fMap() - translation.cast<float>()));
		  count++;
	  }

	  return result;
  }

}

#endif /* COVGRID_SLAM_DISTORTIONCORRECTION_HPP_ */

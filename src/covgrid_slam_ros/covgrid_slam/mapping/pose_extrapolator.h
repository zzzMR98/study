

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "ivcommon/common/time.h"
#include "ivcommon/transform/rigid_transform.h"
#include "covgrid_slam/mapping/imu_tracker.h"
#include "covgrid_slam/sensor/data_type.h"
#include "covgrid_slam/sensor/odometry_data.h"
#include "ivcommon/common/blocking_queue.h"
namespace mapping {
// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
/// \brief 利用运动模型及其他传感器信息进行位姿初估计
///
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(::ivcommon::Duration pose_queue_duration,
                            double imu_gravity_time_constant,bool withoutimu = false);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      ::ivcommon::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  static std::unique_ptr<PoseExtrapolator> InitializeWithoutImu(
      const ::ivcommon::Duration pose_queue_duration,const double imu_gravity_time_constant,
      const ivcommon::transform::posestamped) ;
  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  ::ivcommon::Time GetLastPoseTime() const;

  void AddPose(::ivcommon::Time time, const ivcommon::transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void UpdateImuData();
  void UpdateInsvelocityData();
  void AddEcuData(const sensor::EcuData& ecu_data);
  void AddInsVelocityData(const sensor::InsVelocityData& insvelocity_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  ivcommon::transform::Rigid3d ExtrapolatePose(::ivcommon::Time time);
  ivcommon::transform::Rigid3d ExtrapolatePoseAckermann(::ivcommon::Time time);
  Eigen::Quaterniond ExtrapolateRotation(::ivcommon::Time time);
  Eigen::Vector3d ExtrapolateTranslation(::ivcommon::Time time);
  // Gravity alignment estimate.
  Eigen::Quaterniond gravity_orientation() const {
    return imu_tracker_->orientation();
  }

  void reset(const ivcommon::transform::posestamped& posetime, bool resetvelocity);
  void setvelocity(const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity);
  void getvelocity(Eigen::Vector3d& linearvelocity,Eigen::Vector3d& angularvelocity);
  std::deque<sensor::ImuData>& GetIMUdata(){ return imu_data_; }

  bool withoutimu()
  {
    return withoutimu_;
  }
 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimInsvelocityData();
  void TrimOdometryData();
  void AdvanceImuTracker(::ivcommon::Time time, ImuTracker* imu_tracker);

  const ::ivcommon::Duration pose_queue_duration_;
  using TimedPose = ivcommon::transform::posestamped;
//  struct TimedPose {
//    ::ivcommon::Time time;
//    ivcommon::transform::Rigid3d pose;
//  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::ImuData>> imudata_cache_;
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::InsVelocityData>> insvelocity_cache_;
  std::deque<sensor::EcuData> ecu_data_;
  std::deque<sensor::InsVelocityData> insvelocity_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  bool withoutimu_;
  bool withecu_;
};

}
#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

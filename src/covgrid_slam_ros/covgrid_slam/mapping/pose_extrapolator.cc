

#include <algorithm>

#include "mapping/pose_extrapolator.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/transform/transform.h"
#include "glog/logging.h"

namespace mapping {

PoseExtrapolator::PoseExtrapolator(const ::ivcommon::Duration pose_queue_duration,
                                   double gravity_time_constant,bool withoutimu)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(gravity_time_constant),
      withoutimu_(withoutimu),
	  withecu_(false){}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const ::ivcommon::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
//  LOG(INFO)<<"the imu_data got are"<<imu_data.time<<'\t'<<imu_data.angular_velocity<<'\t'<<imu_data.linear_acceleration;
  auto extrapolator = ::ivcommon::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant,false);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      ::ivcommon::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);//generate the gravity_vector_ and the orientation_ lzz
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);//update the gravity_vector_ and the orientation_ lzz
//  LOG(INFO)<<extrapolator->imu_tracker_->orientation().matrix();

  extrapolator->AddPose(
      imu_data.time,
      ivcommon::transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithoutImu(
    const ::ivcommon::Duration pose_queue_duration,
    const double imu_gravity_time_constant,const ivcommon::transform::posestamped posetime) {
//  LOG(INFO)<<"the imu_data got are"<<imu_data.time<<'\t'<<imu_data.angular_velocity<<'\t'<<imu_data.linear_acceleration;
  auto extrapolator = ::ivcommon::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant,true);
  extrapolator->AddPose(
      posetime.time,
      posetime.pose);
  return extrapolator;
}

void PoseExtrapolator::reset(const ivcommon::transform::posestamped& posetime, bool resetvelocity)
{
  if(resetvelocity)
    {
      linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
      angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
    }
  else if(timed_pose_queue_.size()>0)
    linear_velocity_from_poses_ = posetime.pose.rotation() * timed_pose_queue_.back().pose.rotation().inverse()
						* linear_velocity_from_poses_;
  timed_pose_queue_.clear();
  timed_pose_queue_.push_back(TimedPose{posetime.time, posetime.pose});
  imu_tracker_ =
      ::ivcommon::make_unique<ImuTracker>(gravity_time_constant_, posetime.time);
  TrimImuData();
  TrimOdometryData();
  TrimInsvelocityData();
}

void PoseExtrapolator::setvelocity(const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity)
{
	linear_velocity_from_poses_ = linearvelocity;
	angular_velocity_from_poses_ = angularvelocity;
}

void PoseExtrapolator::getvelocity(Eigen::Vector3d& linearvelocity,Eigen::Vector3d& angularvelocity)
{
	TimedPose newest_timed_pose;
	if(!timed_pose_queue_.empty())
		newest_timed_pose = timed_pose_queue_.back();
	linearvelocity = newest_timed_pose.pose.rotation().inverse() * linear_velocity_from_poses_;
	angularvelocity = angular_velocity_from_poses_;
}

::ivcommon::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return ::ivcommon::Time::min();
  }
  return timed_pose_queue_.back().time;
}

void PoseExtrapolator::AddPose(const ::ivcommon::Time time,
                               const ivcommon::transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    ::ivcommon::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        ::ivcommon::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});//"timed_pose_queue_" recorded the pose relative to the start position for each scan lzz
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  //AdvanceImuTracker(time, imu_tracker_.get());
  UpdateImuData();
  //TrimImuData();
  TrimOdometryData();
  UpdateInsvelocityData();
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);

  imudata_cache_.Push(::ivcommon::make_unique<sensor::ImuData>(imu_data));
}

void PoseExtrapolator::UpdateImuData() {

  while(imudata_cache_.Size()>0)
  {
    imu_data_.push_back(*imudata_cache_.Pop());
  }

  TrimImuData();
}

void PoseExtrapolator::AddEcuData(const sensor::EcuData& ecu_data)
{
	withecu_ = true;
	ecu_data_.push_back(ecu_data);
}

void PoseExtrapolator::AddInsVelocityData(const sensor::InsVelocityData& insvelocity_data)
{
    CHECK(timed_pose_queue_.empty() ||
                  insvelocity_data.time >= timed_pose_queue_.back().time);
    insvelocity_cache_.Push(::ivcommon::make_unique<sensor::InsVelocityData>(insvelocity_data));
	//insvelocity_data_.push_back(insvelocity_data);
}

void PoseExtrapolator::UpdateInsvelocityData()
{
    while(insvelocity_cache_.Size()>0)
    {
        insvelocity_data_.push_back(*insvelocity_cache_.Pop());
    }

    TrimInsvelocityData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_older =
      odometry_data_[odometry_data_.size() - 2];
  const sensor::OdometryData& odometry_data_newer =
      odometry_data_[odometry_data_.size() - 1];
  const double odometry_time_delta =
      ::ivcommon::ToSeconds(odometry_data_older.time - odometry_data_newer.time);
  const ivcommon::transform::Rigid3d odometry_pose_delta =
      odometry_data_newer.pose.inverse() * odometry_data_older.pose;
  angular_velocity_from_odometry_ =
      ivcommon::transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newer_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newer_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newer.time);
  linear_velocity_from_odometry_ =
      orientation_at_newer_odometry_time *
      linear_velocity_in_tracking_frame_at_newer_odometry_time;
}

ivcommon::transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const ::ivcommon::Time time) {
  // TODO(whess): Keep the last extrapolated pose.
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);

//  LOG(INFO)<<time<<"\t"<<newest_timed_pose.time;
  return ivcommon::transform::Rigid3d::Translation(ExtrapolateTranslation(time)) *// left multiply because relative to general coordinate
         newest_timed_pose.pose *//*
         ivcommon::transform::Rigid3d::Rotation(ExtrapolateRotation(time));//right multiply because relative to  newest_timed_pose
}

ivcommon::transform::Rigid3d PoseExtrapolator::ExtrapolatePoseAckermann(const ::ivcommon::Time time) {
  // TODO(whess): Keep the last extrapolated pose.
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
//  if(time>=newest_timed_pose.time)
//  {
//	  LOG(ERROR)<<"time>newest_timed_pose.time";
//	  return newest_timed_pose.pose;
//  }

  CHECK_GE(time, newest_timed_pose.time);
  Eigen::Vector3d translation_vehicle = newest_timed_pose.pose.rotation().inverse() *
		  	  	  	  	  	  	  	  	  (ExtrapolateTranslation(time));

//  static auto start_time = ::ivcommon::now();
//  static double last_traslation = 0;
//  if(::ivcommon::ToSeconds(::ivcommon::now()-start_time)>100)
//  {
//	  if(translation_vehilce[1]<2.2)
//		  translation_vehilce[1]=2.2;
//	  translation_vehilce[1] = translation_vehilce[1]*0.2+last_traslation*0.8;
//  }

//  last_traslation = translation_vehilce[1];
  Eigen::Quaterniond rotation_vehicle = ExtrapolateRotation(time);

  UpdateInsvelocityData();
  //LOG(INFO) <<"insvelocity size " << insvelocity_data_.size();
  if(insvelocity_data_.size()>0)
  {
	  int insvelocitynum = 0;
	  Eigen::Vector3d linearvelocity_sum = Eigen::Vector3d::Zero();
	  Eigen::Vector3d angularvelocity_sum = Eigen::Vector3d::Zero();
	  for(const auto& insvelocity_data : insvelocity_data_)
	  {
		  if(insvelocity_data.time>=newest_timed_pose.time&&insvelocity_data.time<=time)
		  {
              linearvelocity_sum += insvelocity_data.linear_velocity;
              angularvelocity_sum += insvelocity_data.angular_velocity;
			  insvelocitynum++;
		  }
		  else if(insvelocity_data.time>time)
			  break;
	  }

	  while(insvelocity_data_.size()>0&&insvelocity_data_.front().time<newest_timed_pose.time)
		  insvelocity_data_.pop_front();


	  if(insvelocitynum>0)
	  {
//		  LOG(WARNING)<<"pose translation:"<<translation_vehilce;
		  Eigen::Vector3d linearvelocity = linearvelocity_sum / insvelocitynum;
		  auto angularVelocity = angularvelocity_sum / insvelocitynum;
		  double time_diff = ::ivcommon::ToSeconds(time - newest_timed_pose.time);
		  rotation_vehicle = ivcommon::transform::AngleAxisVectorToRotationQuaternion<double>(time_diff * angularVelocity);

		  // INS linear_velocity is not always reliable
//          translation_vehicle[0] = linearvelocity[0]*time_diff;
//          translation_vehicle[1] = linearvelocity[1]*time_diff;
//		  translation_vehilce[2] = insvelocity[2]*time_diff;
//		  LOG(WARNING)<<"ins translation:"<<insvelocity*time_diff<<" "<<insvelocitynum;
//		  if(withoutimu_)
//		  {
////			  LOG(WARNING)<<"pose velocity:"<<angular_velocity_from_poses_*180/M_PI*time_diff;
//			  auto insanglevelocity = insangularvelocity_sum / insvelocitynum;
//			  angular_velocity_from_poses_[2] = insanglevelocity[2];
////			  LOG(WARNING)<<"ins velocity:"<<insanglevelocity*180/M_PI*time_diff<<" "<<insvelocitynum;
//			  rotation_vehilce = ExtrapolateRotation(time);
//		  }
	  }


  }
  else if(ecu_data_.size()>0)
  {
	  int ecunum = 0;
	  double ecu_velocity_sum = 0;

	  for(const auto& ecu_data : ecu_data_)
	  {
		  if(ecu_data.time>=newest_timed_pose.time&&ecu_data.time<=time)
		  {
			  ecu_velocity_sum += ecu_data.fForwardVel;
			  ecunum++;
		  }
		  else if(ecu_data.time>time)
			  break;
	  }

	  while(ecu_data_.size()>0&&ecu_data_.front().time<newest_timed_pose.time)
		  ecu_data_.pop_front();

	  double ecu_velocity = ecu_velocity_sum / ecunum;
	  double time_diff = ::ivcommon::ToSeconds(time - newest_timed_pose.time);

	  if(ecunum>0&&translation_vehicle.norm()>0)
          translation_vehicle = translation_vehicle.normalized()*ecu_velocity*time_diff;
  }
//  translation_vehilce.z()=0;
//  LOG(INFO)<<time<<"\t"<<newest_timed_pose.time;
  return newest_timed_pose.pose *//*
         ivcommon::transform::Rigid3d(translation_vehicle,rotation_vehicle);//right multiply because relative to  newest_timed_pose
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {//estimate by the poses of last two time
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = ::ivcommon::ToSeconds(newest_time - oldest_time);
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " ms";
    return;
  }
  static Eigen::Vector3d last_velocity = Eigen::Vector3d(0,0,0);

  const ivcommon::transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const ivcommon::transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
//  linear_velocity_from_poses_[1] = linear_velocity_from_poses_[1]*0.3 + last_velocity[1]*0.7;
//  last_velocity = linear_velocity_from_poses_;
//  if(linear_velocity_from_poses_[2]>1)
//    linear_velocity_from_poses_[2] = 1;
//  if(linear_velocity_from_poses_[2]<-1)
//    linear_velocity_from_poses_[2] = -1;


  angular_velocity_from_poses_ =
      ivcommon::transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
//  if(angular_velocity_from_poses_[0]>0)
//	  angular_velocity_from_poses_[0] =-0.1;
//  LOG(INFO)<<angular_velocity_from_poses_[0]<<" "<<angular_velocity_from_poses_[1]<<" "<<angular_velocity_from_poses_[2];
}

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimInsvelocityData() {

    while (insvelocity_data_.size() > 1 && !timed_pose_queue_.empty() &&
            insvelocity_data_[1].time <= timed_pose_queue_.back().time) {
        insvelocity_data_.pop_front();
    }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolator::AdvanceImuTracker(const ::ivcommon::Time time,
                                         ImuTracker* const imu_tracker) {
  CHECK_GE(time, imu_tracker->time());
  UpdateImuData();
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const ::ivcommon::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time); //update pose utill this time
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(const ::ivcommon::Time time) {
//  if(withoutimu_)
//    {
      const TimedPose& newest_timed_pose = timed_pose_queue_.back();
      const double extrapolation_delta =
          ::ivcommon::ToSeconds(time - newest_timed_pose.time);
      Eigen::Vector3d pose = extrapolation_delta * angular_velocity_from_poses_;
      auto velocity = newest_timed_pose.pose.rotation().inverse()*linear_velocity_from_poses_;
//      pose.x()= 0 ;//velocity.y()*0.001;2//velocity.y()*0.00112;l
//      pose.y()= 0 ;//-velocity.y()*0.002;2//velocity.y()*0.00017;l
      return ivcommon::transform::AngleAxisVectorToRotationQuaternion<double>(
    		  pose);
//    }
//  else
//    {
//      ImuTracker imu_tracker = *imu_tracker_;
//      AdvanceImuTracker(time, &imu_tracker);
//      const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
//      return last_orientation.inverse() * imu_tracker.orientation();
//    }

}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(::ivcommon::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      ::ivcommon::ToSeconds(time - newest_timed_pose.time);

  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}
}

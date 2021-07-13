/*!
* \file local_trajectory_builder.cc
* \brief 雷达里程计入口类
*
*该文件是雷达里程计入口类，进行位姿估计、地图更新等。
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/07/27
*/

#include "covgrid_slam/mapping3d/local_trajectory_builder.h"

#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/time.h"
#include "covgrid_slam/mapping/scan_matching/proto/real_time_correlative_scan_matcher_options.pb.h"
#include "covgrid_slam/mapping3d/proto/local_trajectory_builder_options.pb.h"
#include "covgrid_slam/mapping3d/proto/submaps_options.pb.h"
#include "covgrid_slam/mapping3d/scan_matching/proto/scan_matcher_options.pb.h"
#include "glog/logging.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "covgrid_slam/mapping3d/scan_matching/fast_correlative_scan_matcher.h"
#include "covgrid_slam/mapping3d/scan_matching/low_resolution_matcher.h"


namespace mapping3d {
/// \brief 雷达里程计的实现类构造函数
///
/// \param options 配置
/// \param global_pose_init 全局初始位姿
/// \param time 初始化时间
LocalTrajectoryBuilder::LocalTrajectoryBuilder(
         proto::LocalTrajectoryBuilderOptions& options,mapping3d::PosewithGps global_pose_init, ::ivcommon::Time time, mapping3d::PosewithGps global_pose_init_only_yaw , bool without_imu)
    : options_(options),
	  submap_manager_(*options.mutable_submaps_options(), without_imu ? global_pose_init : global_pose_init_only_yaw),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          ::ivcommon::make_unique<scan_matching::RealTimeCorrelativeScanMatcher>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(::ivcommon::make_unique<scan_matching::CeresScanMatcher>(
          options_.scan_matcher_options())),
	  g2o_scan_matcher_(::ivcommon::make_unique<scan_matching::G2oScanMatcher>(
	          options_.scan_matcher_options())),
	  ceres_IMUscan_matcher_(
			  options_.scan_matcher_options(), time, global_pose_init),
      accumulated_range_data_{Eigen::Vector3d::Zero(), {}, {}},
      submap_insert_thread_(boost::bind(&LocalTrajectoryBuilder::SubmapInsertThread,this)),
//	  submap_process_thread_(boost::bind(&LocalTrajectoryBuilder::SubmapProcessThread,this)),
	  thread_pool_(1),
	  inspose_queue_(10),
      finished_(false){

  	init_time_ = time;
	//if(options_.without_imu())
	  {

	    setwithoutimu({init_time_
	      ,global_init_pose().pose.inverse()*global_pose_init.pose});
	  }

}
/// \brief 析构函数
///
LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {
  if(!finished_)
    Finish();
}
/// \brief 该类完成的后处理程序
///
/// 停止其他线程、队列等
void LocalTrajectoryBuilder::Finish() {
  while(thread_pool_.DequeSize()>0)
	  usleep(1000);
  submap_insert_queue_.Push(std::make_shared<SubmapSignal>(SignalType::Return));
  finished_ = true;
  inspose_queue_.stopQueue();
  fusepose_queue_.stopQueue();
//  submap_process_thread_.join();
  submap_insert_thread_.join();


}
/// \brief 没有imu的初始化处理
///
/// \param pose 初始位姿
void LocalTrajectoryBuilder::setwithoutimu(const ivcommon::transform::posestamped& pose) {
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithoutImu(
      ::ivcommon::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(),pose);
}

/// \brief 重置
///用于初始化及故障恢复
/// \param pose 重置位姿
/// \param resetvelocity 重置速度标志
void LocalTrajectoryBuilder::reset(const ivcommon::transform::posestamped& pose , bool resetvelocity) {
  extrapolator_->reset(pose,resetvelocity);

  reset_match_status();
  submap_manager_.reset();
  last_pose_estimate_ = {};
}

/// \brief 重置速度
///用于初始化及故障恢复
/// \param linearvelocity 线速度
/// \param angularvelocity 角速度
void LocalTrajectoryBuilder::setvelocity(const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity)
{
	if(extrapolator_)
		extrapolator_->setvelocity(linearvelocity,angularvelocity);
}
/// \brief 获得估计速度
///
/// \param linearvelocity 线速度
/// \param angularvelocity 角速度
void LocalTrajectoryBuilder::getvelocity(Eigen::Vector3d& linearvelocity,Eigen::Vector3d& angularvelocity)
{
	if(extrapolator_)
		extrapolator_->getvelocity(linearvelocity,angularvelocity);
	else
	{
		linearvelocity = Eigen::Vector3d::Zero();
		angularvelocity = Eigen::Vector3d::Zero();
	}
}
/// \brief 增加imu数据
///
/// \param imu_data 增加imu数据
void LocalTrajectoryBuilder::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  init_time_ = imu_data.time;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::ivcommon::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
}

/// \brief 增加ins数据
///
/// \param ins_data 惯导数据
void LocalTrajectoryBuilder::AddInsData(const ivcommon::transform::posestamped& ins_data)
{
	inspose_queue_.Push_Force(std::make_shared<::ivcommon::transform::posestamped>(ins_data));
}
/// \brief 增加融合定位结果数据
///
/// \param fusepose 融合位姿数据
void LocalTrajectoryBuilder::AddFusePoseData(const ivcommon::transform::posestamped& fusepose)
{
	fusepose_queue_.Push_Force(std::make_shared<::ivcommon::transform::posestamped>(fusepose));
}
/// \brief 增加惯导速度数据
///
/// \param insvelocity_data 惯导速度数据
void LocalTrajectoryBuilder::AddInsVelocityData(const sensor::InsVelocityData& insvelocity_data)
{
	  if (extrapolator_ != nullptr) {
	    extrapolator_->AddInsVelocityData(insvelocity_data);
	    return;
	  }
}

/// \brief 增加ecu数据
///
/// \param ecu_data ecu数据
void LocalTrajectoryBuilder::AddEcuData(const sensor::EcuData& ecu_data)
{

	  if (extrapolator_ != nullptr) {
	    extrapolator_->AddEcuData(ecu_data);
	    return;
	  }
}

/// \brief 增加雷达数据
/// 增加雷达数据，并触发雷达里程计的位姿估计等程序
/// \param time 雷达时间戳
/// \param origin 雷达发射器位置
/// \param ranges 雷达点云
/// \return 返回插入地图数据
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddRangefinderData(const ::ivcommon::Time time,
                                           const Eigen::Vector3d& origin,
                                           const sensor::PointCloud& ranges) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }

  if(time<init_time_)
  {
	LOG(ERROR) << "lidar time<init_time_.";
	return nullptr;
  }


	::ivcommon::Time lasttime = last_pose_estimate_.time;
	CHECK_GE(time, lasttime);
  if(options_.scans_per_accumulation()<=1)
  {
	  if (num_accumulated_ == 0) {
	      first_pose_estimate_ = extrapolator_->ExtrapolatePoseAckermann(time);
	      accumulated_range_data_ =
	          sensor::RangeData{Eigen::Vector3d{0,0,0},time, {}, {}};
	    }

	    const ivcommon::transform::Rigid3d tracking_delta =
	        first_pose_estimate_.inverse() *
	        extrapolator_->ExtrapolatePoseAckermann(time);//将差转到第一帧位姿的坐标上
	    const sensor::RangeData range_data_in_first_tracking =sensor::RangeData{Eigen::Vector3d{0,0,0},time, ranges, {}};

	    for (const auto& hit : range_data_in_first_tracking.returns) {
	      const Eigen::Vector3d delta = hit - range_data_in_first_tracking.origin;
	      const float range = delta.norm();
	      if (range >= options_.min_range()) {
	        if (range <= options_.max_range()) {
	          accumulated_range_data_.returns.push_back(hit);
	        } else {
	          // We insert a ray cropped to 'max_range' as a miss for hits beyond the
	          // maximum range. This way the free space up to the maximum range will
	          // be updated.
	          accumulated_range_data_.misses.emplace_back(
	              range_data_in_first_tracking.origin +
	              options_.max_range() / range * delta);
	        }
	      }
	    }
	    ++num_accumulated_;
//	  LOG(INFO)<<"The scans_per_accumulation is "<<options_.scans_per_accumulation();
	    if (num_accumulated_ >= options_.scans_per_accumulation()) {
	      num_accumulated_ = 0;
	      return AddAccumulatedRangeData(
	          time, accumulated_range_data_);
	    }
  }
  else
  {
	  if (num_accumulated_ == 0) {
	      first_pose_estimate_ = extrapolator_->ExtrapolatePoseAckermann(time);
	      accumulated_range_data_ =
	          sensor::RangeData{origin, time, {}, {}};
	    }

	    const ivcommon::transform::Rigid3d tracking_delta =
	        first_pose_estimate_.inverse() *
	        extrapolator_->ExtrapolatePoseAckermann(time);
	    const sensor::RangeData range_data_in_first_tracking =
	        sensor::TransformRangeData(sensor::RangeData{origin, time, ranges, {}},
	                                   tracking_delta);

	    for (const auto& hit : range_data_in_first_tracking.returns) {
	      const Eigen::Vector3d delta = hit - range_data_in_first_tracking.origin;
	      const float range = delta.norm();
	      if (range >= options_.min_range()) {
	        if (range <= options_.max_range()) {
	          accumulated_range_data_.returns.push_back(hit);
	        } else {
	          // We insert a ray cropped to 'max_range' as a miss for hits beyond the
	          // maximum range. This way the free space up to the maximum range will
	          // be updated.
	          accumulated_range_data_.misses.emplace_back(
	              range_data_in_first_tracking.origin +
	              options_.max_range() / range * delta);
	        }
	      }
	    }
	    ++num_accumulated_;
//	  LOG(INFO)<<"The scans_per_accumulation is "<<options_.scans_per_accumulation();
	    if (num_accumulated_ >= options_.scans_per_accumulation()) {
	      num_accumulated_ = 0;
	      return AddAccumulatedRangeData(
	          time, sensor::TransformRangeData(accumulated_range_data_,
	                                           tracking_delta.inverse()));
	    }
  }

  return nullptr;
}
/// \brief 增加多帧叠加的雷达数据，并进行雷达里程计位姿估计
/// 增加多帧叠加的雷达数据，并进行雷达里程计位姿估计。这是雷达里程计的主实现函数，进行位姿估计，离线地图切换、匹配等操作
/// \param time 雷达时间戳
/// \param range_data_in_tracking 雷达数据
/// \return 插入地图结果
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddAccumulatedRangeData(
    const ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking) {
	::ivcommon::Time lasttime = last_pose_estimate_.time;

  const sensor::RangeData filtered_range_data = {
      range_data_in_tracking.origin,
	  range_data_in_tracking.stamp,
      sensor::VoxelFiltered(range_data_in_tracking.returns,
                            options_.voxel_filter_size()),
      sensor::VoxelFiltered(range_data_in_tracking.misses,
                            options_.voxel_filter_size())};


  if (filtered_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  if(needtoreset_)
  {
	  UnNeedToReset();//set needtireset_=false
	  if(options_.submaps_options().readmap_flag())
	  {
		  ivcommon::transform::Rigid3d temp_pose_prediction =
		      extrapolator_->ExtrapolatePoseAckermann(time);

		  auto pose_estimate = temp_pose_prediction;
		  ivcommon::transform::Rigid3d global_pose_from_extern = global_init_pose().pose*temp_pose_prediction;
		  GetAndFuseNowInsData(time, global_pose_from_extern, temp_pose_prediction);
		  GetFuseData(time, global_pose_from_extern);
		  submap_manager_.SwitchToUpdateMode(global_init_pose().pose * pose_estimate,true);//global_pose_from_extern
		  extrapolator_->reset({time,pose_estimate},false);//global_init_pose().pose.inverse() * global_pose_from_extern
		  deadzones_.emplace_back(time,global_pose_from_extern);
		  matching_invalid_num_ = 0;
		  matching_status_ = MatchStatus::succeed;
	  }

  }

  ivcommon::transform::Rigid3d temp_pose_prediction =
      extrapolator_->ExtrapolatePoseAckermann(time);//local_pose

  ivcommon::transform::Rigid3d global_pose_from_extern = global_init_pose().pose*temp_pose_prediction;
  GetAndFuseNowInsData(time, global_pose_from_extern, temp_pose_prediction);
  GetFuseData(time, global_pose_from_extern);


  const ivcommon::transform::Rigid3d pose_prediction = temp_pose_prediction;
  std::shared_ptr<const Submap> matching_submap =
      submap_manager_.submaps().front();//活动中的子地图的第一个

  ivcommon::transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;//obtain pose relative to matching_submap，在子地图中的位置


  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data.returns);//it can automatically adjust filter voxel length




  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'intial_ceres_pose' as an output argument.
    const ivcommon::transform::Rigid3d initial_pose = initial_ceres_pose;
    real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
  }

  ivcommon::transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;

  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(filtered_range_data.returns);


//  sensor::AdaptiveVoxelFilter flat_feature_adaptive_voxel_filter(
//      options_.feature_adaptive_voxel_filter_options());
//  const sensor::PointCloud feature_pointcloud =
//      flat_feature_adaptive_voxel_filter.Filter(filtered_range_data.returns);

  sensor::PointCloud feature_pointcloud = high_resolution_point_cloud_in_tracking;

  feature_pointcloud.insert(feature_pointcloud.end(),low_resolution_point_cloud_in_tracking.begin(),
			    low_resolution_point_cloud_in_tracking.end());//feature_pointcloud中高分辨率在前，低分辨率在后

  sensor::PointCloud high_intensity_pointcloud;

  high_intensity_pointcloud.reserve(filtered_range_data.returns.size());
  for(auto point:filtered_range_data.returns)
  {
	  if(point.intensity>0.6)
		high_intensity_pointcloud.push_back(point);
  }

  sensor::AdaptiveVoxelFilter high_intensitypoint_adaptive_voxel_filter(
      options_.intensity_adaptive_voxel_filter_options());
  high_intensity_pointcloud =
		  high_intensitypoint_adaptive_voxel_filter.MaxIntensityFilter(high_intensity_pointcloud);//it can automatically adjust filter voxel length

  double finalcost = 0;
  extrapolator_->UpdateImuData();
  Eigen::Vector3d temp_linear_vel, temp_angular_vel;
  extrapolator_->getvelocity(temp_linear_vel, temp_angular_vel);//vel in body frame
  if(options_.scan_matcher_options().g2o_optimize())
	g2o_scan_matcher_->Match(
	  {time,matching_submap->local_pose().inverse() * pose_prediction},
	  {time,initial_ceres_pose},
	  {last_pose_estimate_.time,matching_submap->local_pose().inverse()*last_pose_estimate_.pose},
	  {{&high_resolution_point_cloud_in_tracking,
		&matching_submap->high_resolution_hybrid_grid()},
	   {&low_resolution_point_cloud_in_tracking,
		&matching_submap->low_resolution_hybrid_grid()}},
	{{&feature_pointcloud,
			&matching_submap->feature_hybrid_grid()}},
	  &pose_observation_in_submap, &finalcost);
  else if(options_.without_imu())
	ceres_scan_matcher_->Match(
	  {time,matching_submap->local_pose().inverse() * pose_prediction},
	  {time,initial_ceres_pose},
	  {last_pose_estimate_.time,matching_submap->local_pose().inverse()*last_pose_estimate_.pose},
	  {{&high_resolution_point_cloud_in_tracking,
		&matching_submap->high_resolution_hybrid_grid()},
	   {&low_resolution_point_cloud_in_tracking,
		&matching_submap->low_resolution_hybrid_grid()}},
		{{&feature_pointcloud,
			&matching_submap->feature_hybrid_grid()},
		{&high_intensity_pointcloud,
			&matching_submap->high_intensity_feature_hybrid_grid()}},
	{{&high_intensity_pointcloud,
			&matching_submap->intensity_hybrid_grid()}},
	  &pose_observation_in_submap, &finalcost);
  else
	ceres_IMUscan_matcher_.Match(
			{time,initial_ceres_pose},
            temp_linear_vel,
			{last_pose_estimate_.time,matching_submap->local_pose().inverse()*last_pose_estimate_.pose},
			extrapolator_->GetIMUdata(),
			{&feature_pointcloud, matching_submap},
			&pose_observation_in_submap,
			&finalcost
			);



  FailureProcess(finalcost);

  if(matching_status_ == MatchStatus::failed||matching_status_ == MatchStatus::final)
  {
	  submap_manager_.SwitchToUpdateMode(global_init_pose().pose * temp_pose_prediction,true);
	  extrapolator_->reset({time,temp_pose_prediction},false);
	  deadzones_.emplace_back(time,global_pose_from_extern);
	  if(deadzones_.size()>5)
		  deadzones_.pop_front();
	  matching_invalid_num_ = 0;
	  matching_status_ = MatchStatus::succeed;
  }
//  if(matching_status_ == MatchStatus::failed)//cc20191211注释
//  {
//	  LOG(WARNING)<<"MatchStatus::failed";
//	  return nullptr;
//  }
//
//  if(matching_status_ == MatchStatus::final)
//    {
//	  LOG(WARNING)<<"MatchStatus::final";
//      return nullptr;
//    }

  ivcommon::transform::Rigid3d pose_estimate =
      matching_submap->local_pose() * pose_observation_in_submap;



  extrapolator_->AddPose(time, pose_estimate);
  mapping::trajectoryPose trajectorypose;
  trajectorypose.time = time;
  trajectorypose.estimatepose = global_init_pose().pose*pose_estimate;//global_init_pose是submap的全局初始位姿，按照矩阵右乘旋转自身的思路不太对？？
  trajectorypose.gpspose = global_pose_from_extern;
  submap_manager_.AddPose(trajectorypose);
//  LOG(INFO)<<summary.FullReport().c_str();

  if(!DetectFlat(filtered_range_data))
  {
      MatchSubmapAndSwitchMode(time, pose_estimate,global_pose_from_extern,matching_submap,range_data_in_tracking.returns);
  } else
  {
      matchresults_.Clear();
      LOG(WARNING) << " Pointcloud is flat, stop matching offline map ";
  }



  std::shared_ptr<sensor::RangeData> transformed_data(new sensor::RangeData(
		  	  	  	  	  	  	  	  	  sensor::TransformRangeData(filtered_range_data, pose_estimate)));


  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (const std::shared_ptr<Submap>& submap : submap_manager_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  submap_manager_.SwitchSubmap(pose_estimate);
  pose_likegps_ = global_init_pose().pose*pose_estimate;
  if(options_.submaps_options().readmap_flag())
	  pose_likegps_ = submap_manager_.TransformMapGlobalPoseToGpsPose(pose_likegps_);

  last_pose_estimate_ = {
      time, pose_estimate,
      transformed_data->returns
                                  };
//  LOG(INFO)<<last_pose_estimate_.pose.DebugString();
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.



  LOG(INFO)<<"submap index:"<<insertion_submaps.front()->index();
  submap_insert_queue_.Push(std::make_shared<InsertSubmapSignal>(SignalType::Insert,transformed_data));

  return std::unique_ptr<InsertionResult>(
      new InsertionResult{time, range_data_in_tracking, pose_estimate,
                          std::move(insertion_submaps)});

//  return InsertIntoSubmap(time, filtered_range_data, pose_estimate,transformed_data);
}

/**
 *
 * @param rangedata pointcloud in vehicle frame
 * @return true if flat
 */
bool LocalTrajectoryBuilder::DetectFlat(const sensor::RangeData &rangedata)
{
    std::map<int, int> height_histogram;
    int totoal_num = 0;
    for(const sensor::Point& point : rangedata.returns)
    {
        if(point.norm() > 25)
            continue;
        int index = round(5*point.z());
        height_histogram[index]++;
        totoal_num++;
    }

    int max_num = 0;
    for(const auto& iter : height_histogram)
    {
        if(iter.second > max_num)
            max_num = iter.second;
    }
    LOG(WARNING) << "flat occupation rate : " << (double)max_num/totoal_num;
    return ((double)max_num/totoal_num > 0.8);
}

/// \brief 获取对应时间的惯导数据，并与雷达里程计预测结果打包
///
/// \param time 时间戳
/// \param global_pose_from_extern 惯导位姿
/// \param temp_pose_prediction 预测的位姿
void LocalTrajectoryBuilder::GetAndFuseNowInsData(const ::ivcommon::Time time,
		::ivcommon::transform::Rigid3d& global_pose_from_extern,ivcommon::transform::Rigid3d& temp_pose_prediction)
{
	  if(inspose_queue_.Size()>0)
	  {
		  auto inspose = inspose_queue_.Pop();
		  double timediff = ::ivcommon::ToSeconds(inspose->time-time);
		  while(timediff<-0.02)//时间太短了，需要重新pop一个数据
		  {
			  inspose = inspose_queue_.PopWithTimeout(::ivcommon::FromSeconds(0.05));
			  if(inspose==nullptr)
				  return;
			  timediff = ::ivcommon::ToSeconds(inspose->time-time);
		  }
		  auto lastpose = inspose;//上一时刻的Inspose
		  inspose = inspose_queue_.PopWithTimeout(::ivcommon::FromSeconds(0.05));//相邻的第二个数据,即此时的inspose
		  if(inspose==nullptr)
			  return;
		  if(::ivcommon::ToSeconds(inspose->time-time)>0&&::ivcommon::ToSeconds(lastpose->time-time)<0)
		  {
			  double rate = ::ivcommon::ToSeconds(time - lastpose->time)/ ::ivcommon::ToSeconds(inspose->time - lastpose->time);
			  global_pose_from_extern = lastpose->pose.slerp(rate,inspose->pose);

//			  LOG(INFO)<<"##########################################";
		  }
		  else
		  {
			  global_pose_from_extern = lastpose->pose;

//			  LOG(INFO)<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$";
		  }



		  if(options_.scan_matcher_options().fuse_ins())
		  {
			  auto pose_ins_local = global_init_pose().pose.inverse()*global_pose_from_extern;
			  temp_pose_prediction= ::ivcommon::transform::Rigid3d(temp_pose_prediction.translation()
					  ,temp_pose_prediction.rotation().slerp(1,pose_ins_local.rotation()));
		  }

	  }
}

/// \brief 获取对应时间的惯导数据，并与雷达里程计预测结果打包
///
/// \param time 时间戳
/// \param global_pose_from_extern 惯导位姿
/// \param temp_pose_prediction 预测的位姿
void LocalTrajectoryBuilder::GetFuseData(const ::ivcommon::Time time,ivcommon::transform::Rigid3d& global_pose_from_extern)
{

	if(fusepose_queue_.Size()==0)
		return;
	while(fusepose_queue_.Size()>1)
	{
	    submap_manager_.AddFusePose(*fusepose_queue_.Pop());
	}
	auto fusepose = fusepose_queue_.Peek<::ivcommon::transform::posestamped>();
	double timediff = ::ivcommon::ToSeconds(fusepose->time-time);

	if(fabs(timediff)<0.3)
	{
	  global_pose_from_extern = fusepose->pose;
	//			  LOG(INFO)<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$";
	}

}
/// \brief 离线地图匹配失败的判别及处理程序
///
/// \param finalcost 位姿估计时，优化算法最后得到的损失
void LocalTrajectoryBuilder::FailureProcess(const double finalcost)
{

	  const double featureweight =   options_.scan_matcher_options().feature_space_weight(0);
//	  LOG(INFO)<<"final cost"<<finalcost;

	  matched_probability_ = 1 - finalcost/(0.5*featureweight*featureweight);
	  if(options_.submaps_options().readmap_flag() &&
			  matched_probability_ < 0.25)
	    {
	      //extrapolator_->AddPose(time, last_pose_estimate_.pose);
	      LOG(WARNING)<<"cost:"<<finalcost<<" matched_probability:"<<matched_probability_;
	      if(matching_invalid_num_==0)
	    	  matching_invalid_pose_ = last_pose_estimate_;
	      matching_invalid_num_ ++;

	      if(matching_invalid_num_ > matchfinalthreshold_)
	    	  matching_status_ = MatchStatus::final;
	      else  if(matching_invalid_num_ > matchfailedthreshold_)
	    	  matching_status_ = MatchStatus::failed;
	      else
	    	  matching_status_ = MatchStatus::warning;

	//      return nullptr;
	    }
//	  else if(std::isnan(finalcost))
//		  matching_status_ = MatchStatus::failed;
	  else
	    {
	      matching_invalid_num_ = 0;
	      matching_status_ = MatchStatus::succeed;
	    }
}
/// \brief 离线地图匹配及模式切换
///
/// 进行初次搜索并匹配离线地图，更新模式与离线模式间进行切换
/// \param time 雷达时间戳
/// \param pose_estimate 雷达里程计估计位姿
/// \param global_pose_from_extern 惯导位姿，用于搜索子地图的初值位置
/// \param matching_submap 正在匹配的子地图
/// \param point_cloud 雷达点云
void LocalTrajectoryBuilder::MatchSubmapAndSwitchMode(const ::ivcommon::Time time,ivcommon::transform::Rigid3d& pose_estimate
		,const ivcommon::transform::Rigid3d& global_pose_from_extern,std::shared_ptr<const Submap>& matching_submap
		,const sensor::PointCloud& point_cloud)
{

	  if(options_.submaps_options().readmap_flag())
		  matchresults_.Clear();
	  if(options_.submaps_options().automode_flag()&&!options_.submaps_options().readmap_flag()&&matchresults_.Size()>0)
	  {

		  auto matchresult = *matchresults_.Peek<MatchResult>();
		  auto offline_pose_global = matchresult.offlinemap_pose*(matchresult.onlinemap_pose.inverse()*pose_estimate);

		  submap_manager_.UpdateNearestSubmap(offline_pose_global);
		  int nearest_index = submap_manager_.nearest_submap_index();

		  if(matching_submap->header().linkindexs.find(nearest_index)!=matching_submap->header().linkindexs.end()//防止匹配到刚刚保存的地图
				  ||!submap_manager_.IsNearSubmap(nearest_index,matchresult.offlinesubmap_index,2))
			  matchresults_.Pop();
		  else
		  {
			  auto submap_temp = submap_manager_.PullSubmap(nearest_index);

			  if(submap_temp)
			  {
	//			  this->reset({time,pose_estimate},false);

				  submap_manager_.SwitchToReadMode(nearest_index);

				  matching_submap = submap_manager_.submaps().front();

				  auto pose_offlinemap =
						  submap_manager_.TransformPoseInWorldToSubmap(offline_pose_global,nearest_index);

				  pose_estimate = matching_submap->local_pose()*pose_offlinemap;

				  extrapolator_->reset({time,pose_estimate},false);

				  reset_match_status();
	//			  set_global_pose({submap_manager_.submap_lists().global_pose,submap_manager_.submap_lists().gpsdata});
				  matchresults_.Clear();
			  }
			  else
				  submap_manager_.RequestSubmap(nearest_index);
		  }

	  }

	  if(options_.submaps_options().automode_flag()&&!options_.submaps_options().readmap_flag()
			  &&thread_pool_.DequeSize()==0&&matchresults_.Size()==0)
	  {
		  for(const auto& deadzone:deadzones_)
		  {
			  if(fabs(::ivcommon::ToSeconds(time-deadzone.time))<10
					  ||(deadzone.pose.inverse()*global_pose_from_extern).translation().head(2).norm()<10
					  ||(deadzone.pose.inverse()*global_init_pose().pose*pose_estimate).translation().norm()<10)
				  return;
		  }
		  thread_pool_.Schedule([=]() {
			  SubmapMatchOnce({time,global_pose_from_extern},{time,pose_estimate}
			  ,{::ivcommon::ToSeconds(time-last_pose_estimate_.time),last_pose_estimate_.pose.inverse()*pose_estimate}
			  ,point_cloud);
		  });
	  }

}
/// \brief 子地图一次匹配
///
/// 通过惯导位姿，搜索附近子地图，找到最近邻子地图，后利用分支界定法搜索确定车辆在子地图中的大致位置，然后里程计的位姿优化，
/// 获得较为精确的位置，并将该结果与雷达里程计打包,等待下次估计时切换到离线模式
/// \param global_pose_from_extern 惯导位姿
/// \param pose_onlinemap 在线匹配的车辆位姿
/// \param pose_last2now 上一帧到当前帧的位姿，用于估计车辆速度
/// \param point_cloud 点云
void LocalTrajectoryBuilder::SubmapMatchOnce(const ivcommon::transform::posestamped& global_pose_from_extern,const ivcommon::transform::posestamped& pose_onlinemap
		  ,const std::pair<double,ivcommon::transform::Rigid3d>& pose_last2now,const sensor::PointCloud& point_cloud)
{
	  int submap_index = submap_manager_.UpdateNearestSubmapGps(global_pose_from_extern.pose);
//	  matchPoseWithSubmap(init_pose_offlinemap);
		//LOG(WARNING)<<"submap_index:"<<submap_index<<" global_pose_from_extern.pose:"<<global_pose_from_extern.pose;
	  if(submap_index<0)
		  return;
	  const auto submap = submap_manager_.PullSubmap(submap_index);

	  if(submap==nullptr)
	  {
		  submap_manager_.RequestSubmap(submap_index);
		  return;
	  }
//	  LOG(WARNING)<<"submap_index:"<<submap_index;
	  const auto& submap_header = submap_manager_.submap_lists().lists.at(submap_index);
	  const int nearest_node = submap_header.FindNearstNodeGps(global_pose_from_extern.pose);

	  CHECK_GE(nearest_node,0);
	  if(nearest_node<0)
		  return;
	  const auto& node = submap_header.trajectory.at(nearest_node);
//	  auto node2now_vec = (node.gpspose.inverse()*global_pose_from_extern.pose.translation()).head(2);
//	  auto last2now_vec = (global_pose_from_extern.pose.rotation()*pose_onlinemap.pose.rotation().inverse()
//			  *pose_last2now.second.translation()).head(2);

	  float score,rotational_score;
//	  CHECK(submap!=nullptr);
//	  CHECK(submap_header.trajectory.size()>0);
	  auto ref_pose = node.gpspose;
	  if(node.withfusepose)
		  ref_pose = node.fusepose;
	  auto pose_trans = ref_pose.inverse() * global_pose_from_extern.pose;
	  double yaw = ivcommon::transform::GetYaw(pose_trans.rotation());
	  pose_trans = ivcommon::transform::Rigid3d(pose_trans.translation(),ivcommon::transform::RollPitchYaw(0, 0, yaw));//仍然是差值，修正量
	  const ivcommon::transform::Rigid3d init_pose_map = node.estimatepose * pose_trans;
	  ivcommon::transform::Rigid3d pose_offlinemap = submap_manager_.TransformPoseInWorldToSubmap(init_pose_map,submap_index);
	  auto translation = pose_offlinemap.translation();
	  Eigen::Vector3d angle = ivcommon::transform::toRollPitchYaw(pose_offlinemap.rotation());
	  LOG(WARNING)<<"!!!!!!!!!!!!!!!angle: "<<angle.x()*180/M_PI<<" "<<angle.y()*180/M_PI<<" "<<angle.z()*180/M_PI;
//	  auto rotation = ivcommon::transform::RollPitchYaw(0, 0, angle[2]);
	  translation[2] = 0;//altitude is not enough accurate.
	  pose_offlinemap = ivcommon::transform::Rigid3d(translation,pose_offlinemap.rotation());

//	  LOG(WARNING)<<"MakeSubmapScanMatcher";
	  MakeSubmapScanMatcher(point_cloud,submap
			  , {pose_onlinemap.time,pose_offlinemap},pose_last2now
			  , score,rotational_score,pose_offlinemap);

	  pose_offlinemap = submap_manager_.TransformPoseInSubmapToWorld(pose_offlinemap,submap_index);

	  if(score<options_.initmatch_options().min_score())
		  return;

	  MatchResult matchresult = {pose_onlinemap.time,pose_onlinemap.pose,pose_offlinemap,submap_index};
	  matchresults_.Push(std::make_shared<MatchResult>(matchresult));

//	  pose_init_successed_ = true;
//	  extrapolator_->reset({time,active_submaps_.submaps().front()->local_pose() * pose},false);
}

/// \brief 点云与子地图的扫描匹配
///
/// 利用分支界定法搜索确定车辆在子地图中的大致位置，然后里程计的位姿优化，
/// 获得较为精确的位置，并将该结果与雷达里程计打包,等待下次估计时切换到离线模式
/// \param point_cloud 点云
/// \param submap 离线子地图
/// \param init_pose 匹配初始位姿
/// \param pose_last2now 上一帧到当前帧的位姿变化
/// \param score 匹配度
/// \param rotational_score 旋转匹配度
/// \param pose_estimate 匹配后的位姿
void LocalTrajectoryBuilder::MakeSubmapScanMatcher(const sensor::PointCloud& point_cloud,const std::shared_ptr<Submap> submap
		,const ivcommon::transform::posestamped init_pose,const std::pair<double,ivcommon::transform::Rigid3d>& pose_last2now
		,float& score, float& rotational_score, ivcommon::transform::Rigid3d& pose_estimate)
{
	const sensor::AdaptiveVoxelFilter high_resolution_adaptive_voxel_filter_(
	          options_.high_resolution_adaptive_voxel_filter_options());
	const sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter_(
	          options_.low_resolution_adaptive_voxel_filter_options());
	const sensor::PointCloud high_resolution_point_cloud =
	  high_resolution_adaptive_voxel_filter_.Filter(point_cloud);
	const sensor::PointCloud low_resolution_point_cloud =
	  low_resolution_adaptive_voxel_filter_.Filter(point_cloud);
	LOG(WARNING)<<"creating low_resolution_matcher:"<<submap->low_resolution_hybrid_grid().resolution();
	const auto low_resolution_matcher = scan_matching::CreateLowResolutionMatcher(
		  &(submap->feature_hybrid_grid()),
	  &low_resolution_point_cloud, options_.initmatch_options().min_low_resolution_score());
	LOG(WARNING)<<"creating fast_correlative_scan_matcher:"<<submap->high_resolution_hybrid_grid().resolution()
			<<" "<<submap->high_resolution_hybrid_grid().grid_size()<<" "<<submap->histogram().size();

	auto fast_correlative_scan_matcher =
	  ::ivcommon::make_unique<scan_matching::FastCorrelativeScanMatcher>(
			  submap->feature_hybrid_grid(),
			  submap->histogram(),
			  options_.fastcorrelativescanmatcher_options());
	LOG(WARNING)<<"matching";

	fast_correlative_scan_matcher->Match(
			init_pose.pose, high_resolution_point_cloud, point_cloud,
			options_.initmatch_options().min_score(), low_resolution_matcher,
	              &score, &pose_estimate, &rotational_score);
	LOG(WARNING)<<"srore:"<<score<<" "<<rotational_score;


	  sensor::PointCloud feature_pointcloud = high_resolution_point_cloud;

	  feature_pointcloud.insert(feature_pointcloud.end(),low_resolution_point_cloud.begin(),
			  low_resolution_point_cloud.end());
	  double finalcost = 0;

	  sensor::PointCloud intensity_pointcloud;
	  intensity_pointcloud.reserve(point_cloud.size());
	  for(auto point:point_cloud)
	  {
		  if(fabs(point.intensity-0.1)<0.000001||fabs(point.intensity-0.9)<0.000001)
			  intensity_pointcloud.push_back(point);
	  }
	  sensor::AdaptiveVoxelFilter intensitypoint_adaptive_voxel_filter(
	      options_.intensity_adaptive_voxel_filter_options());
	  intensity_pointcloud =
			  intensitypoint_adaptive_voxel_filter.Filter(intensity_pointcloud);//it can automatically adjust filter voxel length
	  if(options_.scan_matcher_options().g2o_optimize())
	  {

		auto g2o_scan_matcher = (::ivcommon::make_unique<scan_matching::G2oScanMatcher>(
			  options_.scan_matcher_options()));
		g2o_scan_matcher->Match(
		  {init_pose.time,pose_estimate},
		  {init_pose.time,pose_estimate},
		  {init_pose.time-ivcommon::FromSeconds(pose_last2now.first),pose_estimate*pose_last2now.second.inverse()},
		  {{&high_resolution_point_cloud,
			&submap->high_resolution_hybrid_grid()},
		   {&low_resolution_point_cloud,
			&submap->low_resolution_hybrid_grid()}},
		  {{&feature_pointcloud,
				&submap->feature_hybrid_grid()}},
		  &pose_estimate, &finalcost);
	  }
	  else
	  {
		auto ceres_scan_matcher = (::ivcommon::make_unique<scan_matching::CeresScanMatcher>(
		  options_.scan_matcher_options()));
		ceres_scan_matcher->Match(
		  {init_pose.time,pose_estimate},
		  {init_pose.time,pose_estimate},
		  {init_pose.time-ivcommon::FromSeconds(pose_last2now.first),pose_estimate*pose_last2now.second.inverse()},
		  {{&high_resolution_point_cloud,
			&submap->high_resolution_hybrid_grid()},
		   {&low_resolution_point_cloud,
			&submap->low_resolution_hybrid_grid()}},
		  {{&feature_pointcloud,
				&submap->feature_hybrid_grid()}},
		  {{&intensity_pointcloud,
				&submap->intensity_hybrid_grid()}},
		  &pose_estimate, &finalcost);
	  }

	  LOG(WARNING)<<"offline map finalcost="<<finalcost;
	  const double featureweight =   options_.scan_matcher_options().feature_space_weight(0);
	  if(finalcost > 0.375*featureweight*featureweight)
		  score = 0;
	  LOG(WARNING)<<"srore:"<<score<<" "<<rotational_score;
}

/// \brief 子地图插入线程
///
/// 用于将点云插入到相应子地图中
void LocalTrajectoryBuilder::SubmapInsertThread()
{
  while(1)
    {
	  static int losenum = 0;
	  static int totalnum = 0;
      while(submap_insert_queue_.Size()>1)
      {
    	  losenum ++;
    	  submap_insert_queue_.Pop();
      }
      totalnum++;
      LOG(INFO)<<"submap lost rate:"<<losenum<<"/"<<totalnum;
      auto signalpair = submap_insert_queue_.Pop();

      if(signalpair->signaltype==SignalType::Insert)
	{
//	  LOG(INFO)<<"It is inserting the submap!";
	  auto time1 = ::ivcommon::now();
	  submap_manager_.InsertRangeData(
	      *(std::dynamic_pointer_cast<InsertSubmapSignal>(signalpair)->rangedata),
//	      extrapolator_->gravity_orientation());
	      pose_estimate().pose.rotation());
//	 LOG(INFO)<<"insert map time"<< ::ivcommon::ToSeconds(::ivcommon::now()-time1);
//	  ShowFeatureGrid(active_submaps_.submaps().front()->feature_hybrid_grid());
	}
      else if(signalpair->signaltype==SignalType::Return)
	{
      LOG(INFO)<<"SubmapInsertThread end";
	  return;
	}
  }
}

//void LocalTrajectoryBuilder::SubmapProcessThread()
//{
//  while(!finished_)
//    {
//	  active_submaps_.ProcessMatchingSubmap(pose_estimate().pose);
//    }
//  LOG(INFO)<<"SubmapProcessThread end";
//}

//void LocalTrajectoryBuilder::ShowFeatureGrid(const FeatureHybridGrid& featurngrid)
//{
//  static ::ros::NodeHandle node_handle_;
//  static ros::Publisher pubDynamic =  node_handle_.advertise<sensor_msgs::PointCloud2>
//  ("/laser_dynamic", 2);
//  pcl::PointCloud<pcl::PointXYZI> point_cloud;
//  for(const auto& point :featurngrid.addpoint_indices_)
//    {
//      pcl::PointXYZI temppoint;
//      temppoint.x = point[0];
//      temppoint.y = point[1];
//      temppoint.z = point[2];
//      temppoint.intensity = 0;
//      point_cloud.push_back(temppoint);
//    }
//
//  for(const auto& point :featurngrid.missgrid_indices_)
//    {
//      pcl::PointXYZI temppoint;
//      temppoint.x = point[0];
//      temppoint.y = point[1];
//      temppoint.z = point[2];
//      temppoint.intensity = 10;
//      point_cloud.push_back(temppoint);
//    }
//  sensor_msgs::PointCloud2 laserCloud;
//
//  pcl::toROSMsg(point_cloud, laserCloud);
//  laserCloud.header.frame_id = "/laser_Submap";
//  laserCloud.header.stamp =  ros::Time::now();
//  pubDynamic.publish(laserCloud);
//}

/// \brief 增加里程计数据
///
/// 增加其他里程计的数据，如车辆里程计
/// \param time 数据时间戳
/// \param odometer_pose 里程计位姿
void LocalTrajectoryBuilder::AddOdometerData(
    const ::ivcommon::Time time, const ivcommon::transform::Rigid3d& odometer_pose) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(sensor::OdometryData{time, odometer_pose});
}

/// \brief 位姿估计结果对外开放接口
///
/// \return 最新匹配位姿
const LocalTrajectoryBuilder::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}
/// \brief 点云插入子地图
///
/// \param time 雷达时间
/// \param range_data_in_tracking 雷达点云
/// \param pose_observation 估计的位姿
/// \param transformed_range_data 转换到子地图坐标系下的雷达点云
/// \return 插入点云的子地图
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::InsertIntoSubmap(
    const ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking,
    const ivcommon::transform::Rigid3d& pose_observation,std::shared_ptr<sensor::RangeData> transformed_range_data) {
  if (motion_filter_.IsSimilar(time, pose_observation)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.


  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (const std::shared_ptr<Submap>& submap : submap_manager_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  LOG(INFO)<<"submap index:"<<insertion_submaps.front()->index();
  submap_insert_queue_.Push(std::make_shared<InsertSubmapSignal>(SignalType::Insert,transformed_range_data));

  return std::unique_ptr<InsertionResult>(
      new InsertionResult{time, range_data_in_tracking, pose_observation,
                          std::move(insertion_submaps)});
}

}  // namespace mapping3d

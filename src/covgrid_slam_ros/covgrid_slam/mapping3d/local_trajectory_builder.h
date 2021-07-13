/*!
* \file local_trajectory_builder.cc
* \brief 雷达里程计入口类
*
*该文件是雷达里程计入口类，进行位姿估计、地图更新等。
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "ivcommon/common/time.h"
#include "covgrid_slam/mapping3d/proto/local_trajectory_builder_options.pb.h"
#include "ivcommon/transform/rigid_transform.h"
#include <fstream>
#include "ivcommon/common/blocking_queue.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ivcommon/common/thread_pool.h"

#include "covgrid_slam/mapping/global_trajectory_builder_interface.h"
#include "covgrid_slam/mapping/pose_extrapolator.h"
#include "covgrid_slam/mapping3d/motion_filter.h"
#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher.h"
#include "covgrid_slam/mapping3d/scan_matching/g2o_scan_matcher.h"
#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher_with_IMU.h"
#include "covgrid_slam/mapping3d/scan_matching/real_time_correlative_scan_matcher.h"
#include "covgrid_slam/mapping3d/submaps.h"
#include "covgrid_slam/mapping3d/submap_manager.h"

#include "covgrid_slam/sensor/range_data.h"
#include "covgrid_slam/sensor/voxel_filter.h"
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

namespace mapping3d {


  enum class MatchStatus{
    succeed =0 ,warning = 1 ,failed=2 , final = 3
  };

// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
// closure.
/// \brief 雷达里程计的实现类
///
/// 进行位姿估计、地图更新等。
class LocalTrajectoryBuilder {
 public:

  using PoseEstimate = mapping::GlobalTrajectoryBuilderInterface::PoseEstimate;

  struct InsertionResult {
    ::ivcommon::Time time;
    sensor::RangeData range_data_in_tracking;
    ivcommon::transform::Rigid3d pose_observation;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  struct MatchResult{
	  ::ivcommon::Time stamp;
	  ivcommon::transform::Rigid3d onlinemap_pose;
	  ivcommon::transform::Rigid3d offlinemap_pose;
	  int offlinesubmap_index;
  };


  explicit LocalTrajectoryBuilder(
      proto::LocalTrajectoryBuilderOptions& options,mapping3d::PosewithGps global_pose_init, ::ivcommon::Time time, mapping3d::PosewithGps global_pose_init_only_yaw = mapping3d::PosewithGps() , bool without_imu = true);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  void Finish();
  void AddImuData(const sensor::ImuData& imu_data);
  void AddInsData(const ivcommon::transform::posestamped& ins_data);
  void AddFusePoseData(const ivcommon::transform::posestamped& ins_data);
  void AddInsVelocityData(const sensor::InsVelocityData& insvelocity_data);
  void AddEcuData(const sensor::EcuData& ecu_data);
  void setwithoutimu(const ivcommon::transform::posestamped& pose);
  void reset(const ivcommon::transform::posestamped& pose,bool resetvelocity);
  void setvelocity(const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity);
  void getvelocity(Eigen::Vector3d& linearvelocity,Eigen::Vector3d& angularvelocity);
  std::unique_ptr<InsertionResult> AddRangefinderData(
      ::ivcommon::Time time, const Eigen::Vector3d& origin,
      const sensor::PointCloud& ranges);
  void AddOdometerData(::ivcommon::Time time,
                       const ivcommon::transform::Rigid3d& odometer_pose);
  void MatchSubmapAndSwitchMode(const ::ivcommon::Time time,ivcommon::transform::Rigid3d& pose_estimate
			,const ivcommon::transform::Rigid3d& temp_pose_ins,std::shared_ptr<const Submap>& matching_submap
			,const sensor::PointCloud& point_cloud);
  void GetAndFuseNowInsData(const ::ivcommon::Time time,ivcommon::transform::Rigid3d& temp_pose_ins,ivcommon::transform::Rigid3d& temp_pose_prediction);
  void GetFuseData(const ::ivcommon::Time time,ivcommon::transform::Rigid3d& global_pose_from_extern);
  void FailureProcess(const double finalcost);
  bool DetectFlat(const sensor::RangeData& rangedata);
  const PoseEstimate& pose_estimate() const;
//  void matchPoseWithSubmap(const ivcommon::transform::posestamped& pose)
//  {
//
//	int index;
//	submap_manager_.UpdateNearestSubmap(pose.pose);
//	Eigen::Vector3d posdiff = nowtranslation - localmapping_->map_entrance_global_pose().translation();
//	posdiff[2] = 0;
//
//	if(posdiff.norm() < 5)
//	  {
//
//		options_.mutable_submaps_options()->set_readmap_flag(true);
//		options_.mutable_submaps_options()->set_update_flag(false);
//		reset({pose.time
//		  ,localmapping_->map_global_pose().pose.inverse() * nowpose},false);
//		localmapping_->set_global_pose(localmapping_->map_global_pose());
//		LOG(WARNING)<<"enter into map";
//	  }
//  }


  inline const PosewithGps& global_init_pose() const
  {
    return submap_manager_.global_init_pose();
  }

  inline void set_global_pose(const PosewithGps& pose)
  {
    submap_manager_.set_global_pose(pose);
  }

  inline std::pair<MatchStatus,ivcommon::transform::posestamped> match_status()
  {
      return {matching_status_ , matching_invalid_pose_};
  }

  inline void reset_match_status()
  {
       matching_status_ = MatchStatus::succeed;
       matching_invalid_num_ = 0;
       submap_insert_queue_.Clear();
  }

  inline int matching_submap_index()
  {
    return submap_manager_.matching_index();
  }

  inline const ivcommon::transform::Rigid3d& PoseLikeGps()
  {
	  return pose_likegps_;
  }

  inline const double matched_probability()
  {
	  return matched_probability_;
  }

  inline void setNeedToReset(int mode)
  {
	  if(options_.submaps_options().readmap_flag() && mode==2)
		  needtoreset_ = true;
  }

  inline void UnNeedToReset()
  {
	  needtoreset_ = false;
  }

 private:
  std::unique_ptr<InsertionResult> AddAccumulatedRangeData(
      ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking,
      const ivcommon::transform::Rigid3d& pose_observation,std::shared_ptr<sensor::RangeData>  transformed_range_data);

  void SubmapInsertThread();
  void SubmapProcessThread();

  void SubmapMatchOnce(const ivcommon::transform::posestamped& global_pose_from_extern,const ivcommon::transform::posestamped& pose_onlinemap
		  ,const std::pair<double,ivcommon::transform::Rigid3d>& pose_last2now,const sensor::PointCloud& point_cloud);
  void ShowFeatureGrid(const FeatureHybridGrid& featurngrid);
  void MakeSubmapScanMatcher(const sensor::PointCloud& point_cloud,const std::shared_ptr<Submap> submap
		  ,const ivcommon::transform::posestamped init_pose,const std::pair<double,ivcommon::transform::Rigid3d>& pose_last2now
		  ,float& score, float& rotational_score, ivcommon::transform::Rigid3d& pose_estimate);
  proto::LocalTrajectoryBuilderOptions& options_; /**<程序所需的一些参数*/

  SubmapManager submap_manager_;/**< 用于管理子地图 */
  PoseEstimate last_pose_estimate_;/**< 上一帧的位姿估计 */
  ivcommon::transform::Rigid3d pose_likegps_;/**< 通过离线匹配并通过储存的GPS定位映射的GPS定位结果 */


  MotionFilter motion_filter_;/**< 根据车位移、时间等滤除部分点云 */
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher>
      real_time_correlative_scan_matcher_;/**< 扫描匹配 */
  std::unique_ptr<scan_matching::CeresScanMatcher> ceres_scan_matcher_;/**< ceres 点云匹配接口 */
  std::unique_ptr<scan_matching::G2oScanMatcher> g2o_scan_matcher_;/**< g2O点云匹配接口 */
  scan_matching::Ceres_IMUScan_Matcher ceres_IMUscan_matcher_;

  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;/**< 根据运动模型估计车辆位姿，用于匹配初值 */

  int num_accumulated_ = 0;/**< 每次使用的点云数量 */
  ivcommon::transform::Rigid3d first_pose_estimate_ = ivcommon::transform::Rigid3d::Identity();/**< 第一帧位姿 */
  sensor::RangeData accumulated_range_data_;/**< 多帧累积的点云数据 */

  ::ivcommon::BlockingQueue<std::shared_ptr<SubmapSignal>> submap_insert_queue_;/**< 用于存储存储子地图插入的信息 */

  ::ivcommon::BlockingQueue<std::shared_ptr<::ivcommon::transform::posestamped>> inspose_queue_;/**< ins数据队列 */
  ::ivcommon::BlockingQueue<std::shared_ptr<::ivcommon::transform::posestamped>> fusepose_queue_;/**< 融合后的位姿队列 */
  ::ivcommon::ThreadPool thread_pool_;/**< 线程池 */
  ::ivcommon::BlockingQueue<std::shared_ptr<MatchResult>> matchresults_;/**< 离线地图初匹配的结果 */
  std::deque<::ivcommon::transform::posestamped> deadzones_;/**< 不进行离线地图匹配区域 */
  ::ivcommon::Time init_time_;/**< 初始时间 */
  int matching_invalid_num_ = 0;/**< 匹配失败的次数 */
  MatchStatus matching_status_ = MatchStatus::succeed;/**< 匹配状态 */
  double matched_probability_;/**< 匹配成功的概率 */
  static constexpr int matchfailedthreshold_ = 5;/**< 匹配失败的阈值 */
  ivcommon::transform::posestamped matching_invalid_pose_;/**< 匹配失败的位姿 */
  static constexpr int matchfinalthreshold_ = 10;/**< 最终匹配失败的阈值 */
  bool finished_;/**< 该类结束的标志 */
  bool needtoreset_;/**< 匹配失败需要重置的标志 */
  boost::thread submap_insert_thread_;/**< 用于插入点云的线程 */
  static constexpr double kExtrapolationEstimationTimeSec = 0.01;
};

}  // namespace mapping3d

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_

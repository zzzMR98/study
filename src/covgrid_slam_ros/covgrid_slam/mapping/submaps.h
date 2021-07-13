#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>
#include <set>
#include "Eigen/Geometry"
#include "ivcommon/common/math.h"
#include "ivcommon/common/port.h"
#include "glog/logging.h"
#include "covgrid_slam/mapping/id.h"
#include "covgrid_slam/mapping/probability_grid.h"
#include "covgrid_slam/mapping/probability_values.h"

namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = ::ivcommon::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local SLAM frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
struct trajectoryPose{
	trajectoryPose():withfusepose(false)
	{

	}
	::ivcommon::Time time;
	::ivcommon::transform::Rigid3d estimatepose;// lidar_odometry estimated global pose
	::ivcommon::transform::Rigid3d gpspose;// global pose from INS
	::ivcommon::transform::Rigid3d fusepose;// global pose from sensor_fusion
	bool withfusepose;
};

/// \brief 子地图头信息类
///
/// 子地图的索引、链接子地图索引，车辆在其中行驶轨迹，寻找最近邻存储轨迹点
class SubmapHeader{

public:
/// \brief 找到最近邻GPS轨迹点
///
/// \param pose gps位姿
/// \return 轨迹点索引
  const int FindNearstNodeGps(ivcommon::transform::Rigid3d pose) const
  {
	  double mindis = 50;
	  double minnode = -1;
	  for(int i=0;i< trajectory.size();i++)
	  {
		  auto ref_pose = trajectory[i].gpspose;
		  if(trajectory[i].withfusepose)
			  ref_pose = trajectory[i].fusepose;
		  auto diff = (ref_pose.inverse()*pose.translation()).head(2).norm();
		  if(diff<mindis)
		  {
			  mindis = diff;
			  minnode = i;
		  }
	  }
	  return minnode;
  }
  /// \brief 找到最近邻里程计轨迹点
  ///
  /// \param pose 里程计位姿
  /// \return 轨迹点索引
  int FindNearstNode(ivcommon::transform::Rigid3d pose)
  {
	  double mindis = 50;
	  double minnode = -1;
	  for(int i=0;i< trajectory.size();i++)
	  {
		  auto diff = (trajectory[i].estimatepose.inverse()*pose.translation()).norm();
		  if(diff<mindis)
		  {
			  mindis = diff;
			  minnode = i;
		  }
	  }
	  return minnode;
  }

  int index;//!<子地图索引
  std::set<int> linkindexs;//!<链接子地图索引
  ::ivcommon::transform::Rigid3d local_pose;//!<子地图位姿
  ::ivcommon::transform::Rigid3d global_pose_adjusted;//!<经过优化的子地图位姿
  std::vector<mapping::trajectoryPose> trajectory;//!<车辆轨迹
};

/// \brief 二维子地图类
///
/// 进行二维子地图的新建、管理活动子地图、插入点云
class Submap {
 public:

  Submap(const ::ivcommon::transform::Rigid3d& local_pose) : local_pose_(local_pose)
 	 {
	  header_.local_pose =  local_pose_;
	  SetIndex(0);
 	 }
  virtual ~Submap() {}


  // Local SLAM pose of this submap.
  ::ivcommon::transform::Rigid3d local_pose() const { return local_pose_; }

  ::ivcommon::transform::Rigid3d global_pose_adjusted() const { return header_.global_pose_adjusted; }
  void set_global_pose_adjusted(const ::ivcommon::transform::Rigid3d& pose)
  {
	  header_.global_pose_adjusted=pose;
  }
  // Number of RangeData inserted.
  int num_range_data() const { return num_range_data_; }

  const std::vector<trajectoryPose>& trajectory()
  {
	  return header_.trajectory;
  }

  void AddPose(const trajectoryPose& pose)
  {
	  if(header_.trajectory.size()==0||(header_.trajectory.back().estimatepose.inverse()*pose.estimatepose.translation()).norm()>kSampleStep)
		  header_.trajectory.push_back(pose);
  }
  void AddFusePose(const ::ivcommon::transform::posestamped& fusepose)
  {
	  for(auto& trajectorynode:header_.trajectory)
	  {
		  if(trajectorynode.time == fusepose.time)
		  {
			  trajectorynode.fusepose = fusepose.pose;
			  trajectorynode.withfusepose = true;
		  }
	  }

  }
  void SetIndex(const int& index)
  {
	  header_.index = index;
  }

  const int& index() const
  {
    return header_.index ;
  }

  void AddLinkIndex(const int& index)
  {
	  if(index>=0)
		  header_.linkindexs.insert(index);
  }

  const std::set<int>& LinkIndexs() const
  {
    return header_.linkindexs ;
  }
  const SubmapHeader& header() const
  {
	  return header_;
  }

  void SetHeader(const SubmapHeader& header)
  {
	  header_ = header;
  }

  void SetLocalpose(const ::ivcommon::transform::Rigid3d& local_pose)
  {
      local_pose_ = local_pose;
  }

  void reset_num_range_data()
  {
      num_range_data_ = 0;
  }

 protected:
  void SetNumRangeData(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

 private:
  ::ivcommon::transform::Rigid3d local_pose_;//!<子地图位姿

  int num_range_data_ = 0;//!<点云帧数
  static constexpr float kSampleStep = 1.;//!<位姿采样间隔
//  std::vector<trajectoryPose> trajectory_;
  SubmapHeader header_;//!<子地图头信息
};
}  // namespace mapping

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_

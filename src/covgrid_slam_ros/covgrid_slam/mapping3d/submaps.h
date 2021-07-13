/*!
* \file submaps.h
* \brief 子地图相关程序
*
* 子地图类，活动子地图类，主要负责点云的插入，活动子地图的管理
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#ifndef CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_


#include "covgrid_slam/mapping/submaps.h"
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "Eigen/Geometry"
#include "ivcommon/common/port.h"
#include "covgrid_slam/mapping3d/proto/submaps_options.pb.h"
#include "covgrid_slam/mapping3d/proto/submap.pb.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include <sstream>
#include <mutex>
#include "covgrid_slam/mapping/id.h"
#include "covgrid_slam/mapping/range_data_inserter.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/range_data_inserter.h"
#include "covgrid_slam/mapping3d/scan_matching/rotational_histogram.h"
#include "covgrid_slam/sensor/data_type.h"
#include "covgrid_slam/sensor/range_data.h"
#include "io/proto_stream.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
namespace mapping3d {

proto::SubmapsOptions CreateSubmapsOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary);

/// \brief 子地图信息列表
///
struct SubmapLists{
  ivcommon::transform::Rigid3d global_pose;
  ivcommon::transform::Rigid3d entrance_pose;
  sensor::GpsInsData gpsdata;
  std::map<int,mapping::SubmapHeader> lists;
};
/// \brief 带gps信息的位姿
///
struct PosewithGps {
ivcommon::transform::Rigid3d pose;
sensor::GpsInsData gps;
};

/// \brief 三维子地图类
///
/// 进行子地图的读取、储存、更新等
class Submap : public mapping::Submap {
 public:
  Submap(float high_resolution, float low_resolution,float feature_resolution,const float intensity_resolution,const int histogram_size,
         const ivcommon::transform::Rigid3d& local_pose);
  Submap(const proto::Submap3D& proto);
  void ToProto(proto::Submap3D* const proto) const;

  void set_point_cloud_fromPCD(const std::string& PCD_dir)
  {
      pcl::io::loadPCDFile(PCD_dir, point_cloud_);
  }

  const HybridGrid& high_resolution_hybrid_grid() const {
    return high_resolution_hybrid_grid_;
  }
  const HybridGrid& low_resolution_hybrid_grid() const {
    return low_resolution_hybrid_grid_;
  }
  const FeatureHybridGrid& feature_hybrid_grid() const {
    return feature_hybrid_grid_;
  }
  const FeatureHybridGrid& high_intensity_feature_hybrid_grid() const {
    return high_intensity_feature_hybrid_grid_;
  }
  const IntensityHybridGrid& intensity_hybrid_grid() const {
    return intensity_hybrid_grid_;
  }

  const Eigen::VectorXf& histogram() const {
    return histogram_;
  }
  bool finished() const { return finished_; }


  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserter& range_data_inserter,
                       int high_resolution_max_range,bool update_common_map=true);
  void Finish();
  void save_To_2d_Grid(const HybridGrid& hybrid_grid,const std::string map_name_index);
  void covert_To_2d_Grid(const HybridGrid& hybrid_grid,const ivcommon::transform::Rigid3d& pose_estimate);
  void covert_To_2d_NowGrid(const HybridGrid& hybrid_grid,const ivcommon::transform::Rigid3d& pose_estimate);


  void save_histogram(const Eigen::VectorXf& histogram,const std::string filename);
  void load_histogram(Eigen::VectorXf& histogram,const std::string filename);
  void save(const std::string& mapdir);
  void saveToProto(const std::string& mapdir);
  void load(int index,const std::string& mapdir);

  void resetstate()
  {
	  finished_ = false;
	  SetNumRangeData(0);
  }


 private:
  HybridGrid high_resolution_hybrid_grid_;//!<高精度概率栅格
  HybridGrid low_resolution_hybrid_grid_;//!<低精度概率栅格
  FeatureHybridGrid feature_hybrid_grid_;//!<特征栅格
  FeatureHybridGrid high_intensity_feature_hybrid_grid_;//!<特征栅格
  IntensityHybridGrid intensity_hybrid_grid_;//!<回波强度栅格
  Eigen::VectorXf histogram_;//!<直方图
  pcl::PointCloud<pcl::PointXYZI> point_cloud_;
  bool finished_ = false;//!<子地图完成标志
  bool updated_ = false;//!<子地图更新标志
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.

/// \brief 活动子地图类
///
/// 进行子地图的新建、管理活动子地图、插入点云
class ActiveSubmaps {
 public:
  explicit ActiveSubmaps(const proto::SubmapsOptions& options);

  ActiveSubmaps(const ActiveSubmaps&) = delete;
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete;
  virtual ~ActiveSubmaps(){};
  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

//  void set_matching_index(int index);

  // Inserts 'range_data' into the Submap collection. 'gravity_alignment' is
  // used for the orientation of new submaps so that the z axis approximately
  // aligns with gravity.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const Eigen::Quaterniond& gravity_alignment);

  void ProcessMatchingSubmap(const ivcommon::transform::Rigid3d pose_estimate);


  inline void PopFront()
  {
	mutex_.lock();
    submaps_.erase(submaps_.begin());
    mutex_.unlock();
  }

  inline void PushFront(std::shared_ptr<Submap> submap)
  {
	mutex_.lock();
    submaps_.push_front(submap);
    matching_submap_index_ = submap->index();
    if(submaps_.size()>2)
    	submaps_.pop_back();
    mutex_.unlock();
  }

  void AddPose(const mapping::trajectoryPose& pose);
  void AddFusePose(const ivcommon::transform::posestamped& fusepose);
  inline void resetsubmap(int index)
  {
    mutex_.lock();
    submaps_.clear();
    mutex_.unlock();
    if(index>=0)
    	matching_submap_index_ = index;


  }
  std::deque<std::shared_ptr<Submap>> submaps() const;
  inline const PosewithGps& global_init_pose() const
  {
//	  LOG(WARNING)<<"global_init_pose_:"<<global_init_pose_.pose.DebugString();
	return global_init_pose_;
  }
  inline void set_global_pose(const PosewithGps& pose)
  {
//	LOG(WARNING)<<"set_global_pose:"<<pose.pose.DebugString();
	global_init_pose_ = pose;
  }


 protected:
  void AddSubmap(const ivcommon::transform::Rigid3d& local_pose);
 private:
  const proto::SubmapsOptions& options_;//!<子地图配置
  int matching_submap_index_ = 0;//!<正在匹配的地图索引
  std::deque<std::shared_ptr<Submap>> submaps_;//!<活动的子地图
  RangeDataInserter range_data_inserter_;//!<子地图插入接口
  PosewithGps global_init_pose_;//!<全局初始位姿
  mutable std::mutex mutex_;//!<地图同步锁
  bool updated_;//!<更新标志
};

}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_

#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
#include <memory>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "ivcommon/common/time.h"
#include "ivcommon/common/mutex.h"
#include "ivcommon/common/file_directory_generation.h"
#include "ivcommon/common/time_conversion.h"
#include "ivcommon/transform/rigid_transform.h"
#include "iv_slam_mapping/mapping/global_trajectory_builder_interface.h"
#include "iv_slam_mapping/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "iv_slam_mapping/mapping_3d/submaps.h"
#include "iv_slam_mapping/sensor/range_data.h"
#include "iv_slam_mapping/sensor/voxel_filter.h"
#include <time.h> 
#include "iv_slam_mapping/sensor/range_data.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
namespace iv_slam_mapping {
namespace mapping_3d {
///
/// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
/// closure.
  ///
class LocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    ::ivcommon::Time time;///当前帧激光雷达点云时间戳
    sensor::RangeData range_data_in_tracking;///当前帧激光雷达点云
    ::ivcommon::transform::Rigid3d pose_observation;///当前激光雷达里程计位姿
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;///地图创建结果
  };
///
  ///局部地图创建着构造函数，主要完成变量初始化
  ///
  explicit LocalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options, const ::ros::NodeHandle & local_trajectory_builder_nh);
  ~LocalTrajectoryBuilder();///析构函数

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;///不允许复制对象
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;///禁止复制对象
  ///
  ///数据预处理，距离滤波，滤去车身周围的点，太近的点以及太远的点，因为其噪声比较多
  ///
  std::unique_ptr<InsertionResult> AddRangefinderData( ::ivcommon::Time time, const Eigen::Vector3f& origin,const sensor::PointCloud& ranges);
  void AddOdometerData(::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& odometer_pose);///添加激光雷达里程计位姿
  void AddLocationModuleData(::ivcommon::Time time,const ::ivcommon::transform::Rigid3d& location_module_data);///添加融合定位信息
   ///
  ///根据需要保存当前点云数据，主要服务与全局位姿优化中的闭环检测
  ///
void RangeDataWriter(const iv_slam_mapping::sensor::RangeData & range_data_,int & inserted_rangedata_index_);
 ///
  ///滤除动态障碍物
  ///
  void ProcessDynamicObjectData(::ivcommon::Time time,const sensor::Data::DynamicObject dynamic_objects);
  ///
  ///点云稀疏化以及获取点云位姿
  ///
  std::unique_ptr<InsertionResult> AddAccumulatedRangeData( ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking);
 ///
  ///进行三维概率栅格地图创建以及准可通行区域提取
  ///
  std::unique_ptr<InsertionResult> InsertIntoSubmap( ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking, const ::ivcommon::transform::Rigid3d& pose_observation);

  const proto::LocalTrajectoryBuilderOptions options_;///地图配置信息
  ActiveSubmaps active_submaps_;///当前活跃的地图
  std::vector<int> current_mapping_indexs;///在线建图模式索引号
  int mapping_mode;  ///建图模式
   int active_submap_initial_index;
  
private: 

std::deque<sensor::OdometryData> odometry_data_;  ///激光雷达里程计位姿
  sensor::RangeData accumulated_range_data_;
   ::ivcommon::Mutex high_resolution_grid_mutex; ///三维地图互斥锁
   ::ivcommon::Mutex odometry_data_mutex; ///里程计位姿互斥锁 
   int last_mapping_mode;///记录之前建图模式
   bool processdynamicobjectdata_running;///障碍物滤除模块运行标示
};

}  /// 命名空间 mapping_3d
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_

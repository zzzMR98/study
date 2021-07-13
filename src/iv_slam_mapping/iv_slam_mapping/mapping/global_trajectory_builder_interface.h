
#ifndef CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ivcommon/common/time.h"
#include "ivcommon/transform/rigid_transform.h"
#include "iv_slam_mapping/mapping/submaps.h"
#include "iv_slam_mapping/mapping/trajectory_builder.h"
#include "iv_slam_mapping/sensor/imu_data.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/range_data.h"
#include "iv_slam_mapping/sensor/data.h"
///
///这是一个虚基类，主要用于实现２Ｄｓｌａｍ和３Ｄｓｌａｍ不同实现时的动态绑定，
///但由于现在去掉了２Ｄｓｌａｍ，因此也就丧失了其虚基类的意义，出于保守，
///当前先给予保存，以防止后人确实需要扩展，如果后人确定不需要扩展,
///删除此类以及修改collated_trajectory_builder.cc中关于对象建立的语句即可．
///
namespace iv_slam_mapping {
namespace mapping {
///
/// This interface is used for both 2D and 3D SLAM. Implementations wire up a
/// global SLAM stack.
  ///
class GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilderInterface() {}
  virtual ~GlobalTrajectoryBuilderInterface() {}

  GlobalTrajectoryBuilderInterface(const GlobalTrajectoryBuilderInterface&) = delete;
  GlobalTrajectoryBuilderInterface& operator=(  const GlobalTrajectoryBuilderInterface&) = delete;

  virtual void AddRangefinderData(::ivcommon::Time time, const Eigen::Vector3f& origin, const sensor::PointCloud& ranges) = 0;///处理点云数据
  virtual void AddImuData(const sensor::ImuData& imu_data) = 0;///处理ＩＭＵ数据
    ///
  ///处理现有激光雷达里程计信息，即先验地图模式以及在线建图模式．
  ///
virtual void AddLidarOdometry(::ivcommon::Time time, const ivcommon::transform::Rigid3d& pose, const Eigen::Vector3d& gps,const std::vector<int>& indexs, const short int & mode ) = 0;
virtual void AddDynamicObjectData(::ivcommon::Time time,  const sensor::Data::DynamicObject dynamic_objects) = 0;///处理动态障碍物信息
virtual void AddLocationModuleData(::ivcommon::Time time, const sensor::Data::OdometrydData location_module) = 0;///处理融合定位模块信息，主要用于为地图提供相应的全局位姿
			       
};

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_

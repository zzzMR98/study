#ifndef CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "iv_slam_mapping/mapping/global_trajectory_builder_interface.h"
#include "iv_slam_mapping/mapping_3d/local_trajectory_builder.h"
#include "iv_slam_mapping/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "iv_slam_ros_msgs/OptimizationImu.h"
#include "iv_slam_ros_msgs/OptimizationInsertResult.h"
#include <fstream>

namespace iv_slam_mapping {
namespace mapping_3d {

class GlobalTrajectoryBuilder: public mapping::GlobalTrajectoryBuilderInterface {
 public:
   ///
   ///构造函数
   ///
  GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options, int trajectory_id,const ::ros::NodeHandle & GlobalTrajectoryBuilder_nh);
  ~GlobalTrajectoryBuilder() override;///析构函数
  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  void AddImuData(const sensor::ImuData& imu_data) override;///处理ＩＭＵ数据
  void AddRangefinderData(::ivcommon::Time time, const Eigen::Vector3f& origin,const sensor::PointCloud& ranges) override;///处理点云数据
  void SendInsertResult(const ::ivcommon::Time& time,LocalTrajectoryBuilder::InsertionResult* insertion_result);///发送建图结果信息
    ///
  ///处理现有激光雷达里程计信息，即先验地图模式以及在线建图模式．
  ///
void AddLidarOdometry(::ivcommon::Time time,const ::ivcommon::transform::Rigid3d& pose, const Eigen::Vector3d& gps,const std::vector<int>& indexs, const short int & mode ) override;
void AddDynamicObjectData(::ivcommon::Time time, const sensor::Data::DynamicObject dynamic_objects) override;///处理动态障碍物信息
void AddLocationModuleData(::ivcommon::Time time,const sensor::Data::OdometrydData location_module) override;///处理融合定位模块信息，主要用于为地图提供相应的全局位姿
  const int trajectory_id_; ///轨迹ｉｄ
   LocalTrajectoryBuilder local_trajectory_builder_;///局部地图创建者
};

}  /// 命名空间 mapping_3d
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_

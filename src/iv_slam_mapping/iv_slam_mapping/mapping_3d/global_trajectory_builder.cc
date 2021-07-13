
#include "iv_slam_mapping/mapping_3d/global_trajectory_builder.h"
#include <iomanip>
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "ivcommon/transform/utm/utm.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace iv_slam_mapping
{
  namespace mapping_3d
  {
    ///
    ///构造函数
    ///
    GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions &options,
                                                     const int trajectory_id, const ::ros::NodeHandle &GlobalTrajectoryBuilder_nh)
        : trajectory_id_(trajectory_id), local_trajectory_builder_(options, GlobalTrajectoryBuilder_nh) {}
    GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {} ///析构函数

    void GlobalTrajectoryBuilder::AddImuData(const sensor::ImuData &imu_data) {} //无用
    ///
    ///处理点云数据
    ///
    void GlobalTrajectoryBuilder::AddRangefinderData(const ::ivcommon::Time time, const Eigen::Vector3f &origin, const sensor::PointCloud &ranges)
    {
      auto insertion_result = local_trajectory_builder_.AddRangefinderData(time, origin, ranges); ///处理点云信息，即将进行三维地图创建
    }

    void GlobalTrajectoryBuilder::AddLidarOdometry(::ivcommon::Time time, const ::ivcommon::transform::Rigid3d &pose, const Eigen::Vector3d &gps, const std::vector<int> &indexs, const short int &mode)
    {
      static int receive_odometry_index = 0;
      if (receive_odometry_index == 0)
      {
        double a = 6378137;
        double e2 = 0.0818192 * 0.0818192; //e的平方
        ::ivcommon::transform::GridZone zone = ::ivcommon::transform::UTM_ZONE_AUTO;
        ::ivcommon::transform::Hemisphere hemi = ::ivcommon::transform::HEMI_NORTH;
        double N, E;
        ::ivcommon::transform::geographic_to_grid(a, e2, gps.y() * M_PI / 180, gps.x() * M_PI / 180, &zone, &hemi, &N, &E);
        local_trajectory_builder_.active_submaps_.activemap_constant.gps_zone = zone; ///获得ｚｏｎｅ
        receive_odometry_index++;                                                     ///保证只运行一次，因为此语句段是为了获得一个准确的ｚｏｎｅ
      }
      local_trajectory_builder_.AddOdometerData(time, pose);     ///添加里程计数据
      local_trajectory_builder_.current_mapping_indexs = indexs; ///每次刚开始在线建图的时候只有一个值，后面会有前后两个地图的值
      local_trajectory_builder_.mapping_mode = mode;             ///０为正在创建地图，１为一个地图创建完成，２为先验地图模式
    }
    ///
    ///滤除动态障碍物信息
    ///
    void GlobalTrajectoryBuilder::AddDynamicObjectData(::ivcommon::Time time, const sensor::Data::DynamicObject dynamic_objects)
    {
      boost::thread process_dynamicobject_thread(boost::bind(&LocalTrajectoryBuilder::ProcessDynamicObjectData, &local_trajectory_builder_, time, dynamic_objects));
      process_dynamicobject_thread.detach(); ///释放本线程资源
    }

    void GlobalTrajectoryBuilder::AddLocationModuleData(::ivcommon::Time time, const sensor::Data::OdometrydData location_module)
    {
      double a = 6378137;
      double e2 = 0.0818192 * 0.0818192; //e的平方
      ::ivcommon::transform::GridZone zone = ::ivcommon::transform::UTM_ZONE_AUTO;
      ::ivcommon::transform::Hemisphere hemi = ::ivcommon::transform::HEMI_NORTH;
      double meter_x, meter_y;
      ::ivcommon::transform::geographic_to_grid(
        a, e2, location_module.pose.translation().y() * M_PI / 180,
        location_module.pose.translation().x() * M_PI / 180, &zone, 
        &hemi, &meter_y, &meter_x); ///经纬度坐标转为米制坐标
      // 添加全局定位信息
      local_trajectory_builder_.AddLocationModuleData(time, ::ivcommon::transform::Rigid3d(Eigen::Vector3d(meter_x - 500000, meter_y, location_module.pose.translation().z()), location_module.pose.rotation()));
    }

  } // namespace mapping_3d
} // namespace iv_slam_mapping

#include "ivcommon/common/time.h"
#include "iv_slam_mapping/mapping/collated_trajectory_builder.h"
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace
///
///这里比较绕，所以说明一下构造函数功能以及后面可能涉及到的调用逻辑：
///该类负责整理各传感器数据，将其传送到sensor_collator_中用于数据调度．
///同时在构造函数中利用ｌａｍｄａ表达式注册了传感器数据对应的回调函数，用于在数据调度快结束时调用．
///wrapped_trajectory_builder_是一个指向GlobalTrajectoryBuilderInterface的ｕｎｉｑｕｅ指针，
///其会在回调函数HandleCollatedSensorData中调用GlobalTrajectoryBuilderInterface的各成员函数，
///GlobalTrajectoryBuilderInterface作为一个虚基类被ｍａｐｐｉｎｇ_3d中的GlobalTrajectoryBuilder所继承，
///然后通过wrapped_trajectory_builder_这一基类指针调用继承类的函数，实现对GlobalTrajectoryBuilder中函数的调用，
///对GlobalTrajectoryBuilder中函数的调用又将触发ｌｏｃａｌ＿ｔｒａｊｅｃｔｏｒｙ＿ｂｕｉｌｄｅｒ中相应函数的调用．
///以上就是函数简单的调用逻辑
///
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder( sensor::Collator* const sensor_collator, const int trajectory_id,
    const std::unordered_set<string>& expected_sensor_ids, std::unique_ptr<GlobalTrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),trajectory_id_(trajectory_id),wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()){
  sensor_collator_->AddTrajectory( trajectory_id, expected_sensor_ids,[this](const string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));});///HandleCollatedSensorData为传感器回调函数
}
///
///析构函数
///
CollatedTrajectoryBuilder::~CollatedTrajectoryBuilder() {}
///
///向传感数据整理器中添加传感器数据
///
void CollatedTrajectoryBuilder::AddSensorData(const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, sensor_id, std::move(data));
}

void CollatedTrajectoryBuilder::HandleCollatedSensorData( const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);///找到sensor_id传感器对应的ｒａｔｅｔｉｍｅｒ．
  if (it == rate_timers_.end()) {
    it = rate_timers_.emplace( std::piecewise_construct, std::forward_as_tuple(sensor_id),
		       std::forward_as_tuple(  ::ivcommon::FromSeconds(kSensorDataRatesLoggingPeriodSeconds))).first;
  }///如果没有找到sensor_id对应的ｒａｔｅｔｉｍｅｒ则添加一个对应的ｒａｔｅ＿ｔｉｍｅｒ
  it->second.Pulse(data->time);///向ｒａｔｅｔｉｍｅｒ中添加一个事件
///
  ///每隔１５秒输出一次传感器数据频率信息
  ///
  if (std::chrono::steady_clock::now() - last_logging_time_ > ::ivcommon::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();///更新日志记录事件．
  }
///
    ///根据数据类型调用不同的函数．
    ///
  switch (data->type) {    
///
    ///调用GlobalTrajectoryBuilder::AddRangefinderData
    ///
    case sensor::Data::Type::kRangefinder:
      wrapped_trajectory_builder_->AddRangefinderData( data->time, data->rangefinder.origin, data->rangefinder.ranges);
      return;

      
         ///
    ///调用GlobalTrajectoryBuilder::AddLidarOdometry
    ///
      case sensor::Data::Type::KLidarOdometry:
      wrapped_trajectory_builder_->AddLidarOdometry(data->time, data->lidar_odometry.odometer_pose,data->lidar_odometry.GPS,data->lidar_odometry.indexs,data->lidar_odometry.mode);
      return;
           ///
    ///调用GlobalTrajectoryBuilder::AddDynamicObjectData
    ///    
      case sensor::Data::Type::KDynamicObject:
      wrapped_trajectory_builder_->AddDynamicObjectData(data->time,data->dynamic_object);
      return;
         ///
    ///调用GlobalTrajectoryBuilder::AddLocationModuleData
    ///
       case sensor::Data::Type::KLocationModule:
      wrapped_trajectory_builder_->AddLocationModuleData(data->time,data->location_module);
      return;
  }
  LOG(FATAL);
}

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

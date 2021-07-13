
#ifndef CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include "ivcommon/common/port.h"
#include "ivcommon/common/rate_timer.h"
#include "iv_slam_mapping/mapping/global_trajectory_builder_interface.h"
#include "iv_slam_mapping/mapping/submaps.h"
#include "iv_slam_mapping/mapping/trajectory_builder.h"
#include "iv_slam_mapping/sensor/collator.h"
#include "iv_slam_mapping/sensor/data.h"

namespace iv_slam_mapping {
namespace mapping {
///
/// Handles collating sensor data using a sensor::Collator, then passing it on to
/// a mapping::GlobalTrajectoryBuilderInterface .
  ///
class CollatedTrajectoryBuilder : public TrajectoryBuilder {
 public:
  CollatedTrajectoryBuilder( sensor::Collator* sensor_collator, int trajectory_id,
      const std::unordered_set<string>& expected_sensor_ids, std::unique_ptr<GlobalTrajectoryBuilderInterface> wrapped_trajectory_builder);///构造函数
  ~CollatedTrajectoryBuilder() override;///析构函数
  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;///禁止复制对象
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) = delete;///禁止复制对象
  void AddSensorData(const string& sensor_id,std::unique_ptr<sensor::Data> data) override;///向传感数据整理器中添加传感器数据
  void HandleCollatedSensorData(const string& sensor_id,std::unique_ptr<sensor::Data> data);///传感整理器回调函数
  sensor::Collator* const sensor_collator_;///传感整理器
  const int trajectory_id_;///路线ＩＤ
   std::unique_ptr<GlobalTrajectoryBuilderInterface> wrapped_trajectory_builder_;
///
  /// Time at which we last logged the rates of incoming sensor data.
   ///
  std::chrono::steady_clock::time_point last_logging_time_;///上一次的日志记录时间，主要用于统计当前时间到上次日志时间之间的传感器接收频率
  std::map<string, ::ivcommon::RateTimer<>> rate_timers_;///与每种数据对应的频率计时器
};

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_

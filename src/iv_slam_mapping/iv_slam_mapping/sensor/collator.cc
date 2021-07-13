
#include "iv_slam_mapping/sensor/collator.h"

namespace iv_slam_mapping {
namespace sensor {
 Collator::Collator(bool wiping_dynamic_object):wiping_dynamic_object_(wiping_dynamic_object),queue_(wiping_dynamic_object){}///构造函数
///
  /// Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  /// for each collated sensor data.
  ///
void Collator::AddTrajectory(   const int trajectory_id,const std::unordered_set<string>& expected_sensor_ids,const Callback& callback) {
 
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}
///
  /// Marks 'trajectory_id' as finished.
  ///
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}
///
  /// Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  /// sensor data. Sensor packets with matching 'sensor_id' must be added in time
  /// order.
  ///
void Collator::AddSensorData(const int trajectory_id, const string& sensor_id, std::unique_ptr<Data> data) {
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}
///
  /// Dispatches all queued sensor packets. May only be called once.
  /// AddSensorData may not be called after Flush.
  ///
void Collator::Flush() { queue_.Flush(); }
///
  /// Must only be called if at least one unfinished trajectory exists. Returns
  /// the ID of the trajectory that needs more data before the Collator is
  /// unblocked.
  ///
int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // 命名空间 sensor
}  // 命名空间 iv_slam_mapping

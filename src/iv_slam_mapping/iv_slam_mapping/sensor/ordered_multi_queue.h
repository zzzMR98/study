#ifndef CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include "ivcommon/common/blocking_queue.h"
#include "ivcommon/common/port.h"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping/sensor/data.h"

namespace iv_slam_mapping {
namespace sensor {

struct QueueKey {
  int trajectory_id;
  string sensor_id;
///
  ///重载运算符，用于比较QueueKey
  ///
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) < std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
  ///
  ///重载运算符，用于比较QueueKey
  ///
  bool operator==(const QueueKey& other)const {
    return (trajectory_id == other.trajectory_id) && (sensor_id == other.sensor_id);    
  }
  ///
  ///重载运算符，用于比较QueueKey
  ///
    bool operator!=(const QueueKey& other)const {
    return (trajectory_id != other.trajectory_id) || (sensor_id != other.sensor_id);    
  }
};
///
/// Maintains multiple queues of sorted sensor data and dispatches it in merge
/// sorted order. It will wait to see at least one value for each unfinished
/// queue before dispatching the next time ordered value across all queues.
///
/// This class is thread-compatible.
///
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue(bool wiping_dynamic_object);
  ~OrderedMultiQueue();
  ///
  /// Adds a new queue with key 'queue_key' which must not already exist.
  /// 'callback' will be called whenever data from this queue can be dispatched.
  ///
  void AddQueue(const QueueKey& queue_key, Callback callback);
///
  /// Marks a queue as finished, i.e. no further data can be added. The queue
  /// will be removed once the last piece of data from it has been dispatched.
  ///
  void MarkQueueAsFinished(const QueueKey& queue_key);
///
  /// Adds 'data' to a queue with the given 'queue_key'. Data must be added
  /// sorted per queue.
  ///
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);
///
  /// Dispatches all remaining values in sorted order and removes the underlying
  /// queues.
  ///
  void Flush();
///
  /// Must only be called if at least one unfinished queue exists. Returns the
  /// key of a queue that needs more data before the OrderedMultiQueue can
  /// dispatch data.
  ///
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    ::ivcommon::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;///每一种数据对应的毁掉函数，在CollatedTrajectoryBuilder的构造函数中注册了这一毁掉函数
    bool finished = false;
  };

  void Dispatch();///进行数据调度
  void CannotMakeProgress(const QueueKey& queue_key);
  ::ivcommon::Time GetCommonStartTime(int trajectory_id);///用于获得接收到所有种类数据时的时间

  // Used to verify that values are dispatched in sorted order.
  ::ivcommon::Time last_dispatched_time_ = ::ivcommon::Time::min();///上次调度的时间
  ::ivcommon::Time last_lidar_odometry_time_ = ::ivcommon::Time::min();///上次调度雷达里程计的时间
  ::ivcommon::Time last_lidar_sensor_time_ = ::ivcommon::Time::min();///上次调度雷达点云的时间
bool find_lidar_odometry;///是否寻找激光雷达里程计数据
bool find_lidar_sensor;///是否寻找点云数据
  const QueueKey lidar_odometry_quekey;///雷达里程计ｑｕｅｋｅｙ
  const QueueKey lidar_sensor_quekey;///三维点云ｑｕｅｋｅｙ
  std::map<int, ::ivcommon::Time> common_start_time_per_trajectory_;///接收到所有种类数据时的时间
  std::map<QueueKey, Queue> queues_;///每一种ｑｕｅｋｅｙ对应的ｑｕｅｕｅ
  QueueKey blocker_;
 bool  wiping_dynamic_object_; 
};

}  // namespace sensor
}  // namespace iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

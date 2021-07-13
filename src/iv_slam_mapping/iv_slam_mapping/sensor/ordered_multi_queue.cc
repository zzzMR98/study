
#include "iv_slam_mapping/sensor/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>


#include "ivcommon/common/make_unique.h"
#include "glog/logging.h"
#include "ros/ros.h"

namespace iv_slam_mapping {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace
///
///重载运算符，用于输出quekey
///
inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}
///
///构造函数，并进行成员变量初始化
///
OrderedMultiQueue::OrderedMultiQueue(bool wiping_dynamic_object):
lidar_odometry_quekey(QueueKey{0,/*"lidar_odometry_to_earth"*/"lidar_odometry_for_mapping" }),
lidar_sensor_quekey(QueueKey{0,"points2"}),
wiping_dynamic_object_(wiping_dynamic_object){
   find_lidar_odometry = true;
 find_lidar_sensor = false;
}
///
///析构函数
///
OrderedMultiQueue::~OrderedMultiQueue() {
}
  ///
  /// Adds a new queue with key 'queue_key' which must not already exist.
  /// 'callback' will be called whenever data from this queue can be dispatched.
  ///
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}
///
  /// Marks a queue as finished, i.e. no further data can be added. The queue
  /// will be removed once the last piece of data from it has been dispatched.
  ///
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);
  queue.finished = true;
  Dispatch();
}
///
  /// Adds 'data' to a queue with the given 'queue_key'. Data must be added
  /// sorted per queue.
  ///
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
 
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  it->second.queue.Push(std::move(data));//add the data into deque_;
  Dispatch();
			      
			    }
///
  /// Dispatches all remaining values in sorted order and removes the underlying
  /// queues.
  ///
void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}
///
  /// Must only be called if at least one unfinished queue exists. Returns the
  /// key of a queue that needs more data before the OrderedMultiQueue can
  /// dispatch data.
  ///
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}
///
///进行数据调度
///
void OrderedMultiQueue::Dispatch() {

  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    for (auto it = queues_.begin(); it != queues_.end();) {     
      const auto* data = it->second.queue.Peek<Data>();///return deque_.front()
      if (data == nullptr) {
        if (it->second.finished) {
	
          queues_.erase(it++);
          continue;
        }
LOG(WARNING)<<"data null:"<<it->first;
        CannotMakeProgress(it->first);
        return;
      }
 ///     
/// 寻找指定的数据
/// 
      if(find_lidar_odometry){
	  if((it->first != lidar_odometry_quekey) /*&& (it->first !=imu_data_quekey)&&(it->first !=location_module_data_quekey)&&(it->first != dynamic_object_quekey)*/){
	     ++it;
	    continue;
	  }
      if(it->first == lidar_odometry_quekey){
	
	    while(it->second.queue.Size()>2){
	      it->second.queue.Pop();
	       data = it->second.queue.Peek<Data>();
	    if (data == nullptr) {
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
        CannotMakeProgress(it->first);
        return;
            }
	    }	 
            }
	}
	
	 if(find_lidar_sensor){
	  if((it->first != lidar_sensor_quekey)/* && (it->first !=imu_data_quekey)&& (it->first !=location_module_data_quekey)&&(it->first != dynamic_object_quekey)*/){
	    ++it;
	    continue;
	  }	  
	}

      if (next_data == nullptr || data->time <= next_data->time) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;

      }    

      ++it;       	
    }
     
    if (next_data == nullptr) {    
      CHECK(queues_.empty());
      return;
    }    
///
    /// If we haven't dispatched any data for this trajectory yet, fast forward
    /// all queues of this trajectory until a common start time has been reached.
    ///
    const ::ivcommon::Time common_start_time =  GetCommonStartTime(next_queue_key.trajectory_id);///return the time when we got the sensor data 

    if (next_data->time >= common_start_time) {
      /// Happy case, we are beyond the 'common_start_time' already.        
        if(next_queue_key == lidar_sensor_quekey){
	while(next_data->time<last_lidar_odometry_time_&&next_data!=nullptr){///丢弃旧的雷达点云数据
	  next_queue->queue.Pop();	  
	  next_data = next_queue->queue.Peek<Data>();  
	    if(next_data==nullptr) break;	
	}
	if(next_data==nullptr){
	  LOG(WARNING)<<"the lidar sensor data is null!";
	  return;
	}
	if(next_data->time == last_lidar_odometry_time_ ){///如果当前点云数据时间戳和激光雷达里程计对应上，则执行毁掉函数

 last_lidar_sensor_time_ = last_lidar_odometry_time_;
	  last_dispatched_time_ = last_lidar_sensor_time_;	 
	    find_lidar_odometry = true;///下一次开始寻找激光雷达里程计数据
	  find_lidar_sensor = false;	 
	  next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数
	 
	  return;
	}else if (next_data->time>last_lidar_odometry_time_){
	  LOG(INFO)<<"It`s waiting for the lidarodometry data!";
	   find_lidar_sensor = false;
	  find_lidar_odometry = true;
	  return;  
	}
	
      }
     
         if(next_queue_key == lidar_odometry_quekey){	    
	   while(next_data->time<=last_lidar_sensor_time_&&next_data!=nullptr){///丢弃旧的激光雷达里程计消息
	     next_queue->queue.Pop();
	     next_data = next_queue->queue.Peek<Data>();
	       if(next_data==nullptr) break;	
	  }
	  if(next_data==nullptr){
	  LOG(WARNING)<<"the lidar odometry data is null!";
	  return;
	}	
	if(next_data->time>last_lidar_sensor_time_){///如果当前里程计数据比上次处理的点云数据要信，则执行里程计数据对应的毁掉函数
	  last_lidar_odometry_time_ = next_data->time;
	last_dispatched_time_ = last_lidar_odometry_time_;
	   find_lidar_sensor = true;///开始寻找激光雷达点云数据
	  find_lidar_odometry = false;	
	  next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数
	return;
	}else{
	  return;}   
       
      
    }	
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数 
    } else if (next_queue->queue.Size() < 2) {
      if (!next_queue->finished) {
        CannotMakeProgress(next_queue_key);
        return;
      }        
        if(next_queue_key == lidar_sensor_quekey){
	while(next_data->time<last_lidar_odometry_time_&&next_data!=nullptr){
	  next_queue->queue.Pop();	  
	  next_data = next_queue->queue.Peek<Data>();  
	    if(next_data==nullptr) break;	
	}
	if(next_data==nullptr){
	  LOG(WARNING)<<"the lidar sensor data is null!";
	  return;
	}
	if(next_data->time == last_lidar_odometry_time_ ){
 last_lidar_sensor_time_ = last_lidar_odometry_time_;
	  last_dispatched_time_ = last_lidar_sensor_time_;
{
	    find_lidar_odometry = true;
	  find_lidar_sensor = false;
	  }
	  next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数
	 
	  return;
	}else if (next_data->time>last_lidar_odometry_time_){
	  LOG(INFO)<<"It`s waiting for the lidarodometry data!";
	   find_lidar_sensor = false;
	  find_lidar_odometry = true;
	  return;  
	}	
      }     
         if(next_queue_key == lidar_odometry_quekey){
{	    
	   while(next_data->time<=last_lidar_sensor_time_&&next_data!=nullptr){
	     next_queue->queue.Pop();
	     next_data = next_queue->queue.Peek<Data>();
	       if(next_data==nullptr) break;	
	  }
	  if(next_data==nullptr){
	  LOG(WARNING)<<"the lidar odometry data is null!";
	  return;
	}	
	if(next_data->time>last_lidar_sensor_time_){
	  last_lidar_odometry_time_ = next_data->time;
	last_dispatched_time_ = last_lidar_odometry_time_;
	   find_lidar_sensor = true;
	  find_lidar_odometry = false;
	  next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数	 	  
	return;
	}else{
	  return;}	    
	  }
      }    

      
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop());///执行该数据类型对应的回调函数
    } else {
      ///
      /// We take a peek at the time after next data. If it also is not beyond
      /// 'common_start_time' we drop 'next_data', otherwise we just found the
      /// first packet to dispatch from this queue.
      ///
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->time > common_start_time) {
        last_dispatched_time_ = next_data->time;
        next_queue->callback(std::move(next_data_owner));///执行该数据类型对应的回调函数
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    if (entry.second.queue.Size() > kMaxQueueSize) {
        LOG_EVERY_N(WARNING, 500) << "Queue waiting for data: " << queue_key;///当收不到数据时，每５００次提醒一下
      return;
    }
  }
}
///
///用于获得接收到所有种类数据时的时间
///
::ivcommon::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, ::ivcommon::Time::min());
  ::ivcommon::Time& common_start_time = emplace_result.first->second;
  if (emplace_result.second) {

    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time =
            std::max(common_start_time, entry.second.queue.Peek<Data>()->time);
      }
    }

    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace iv_slam_mapping

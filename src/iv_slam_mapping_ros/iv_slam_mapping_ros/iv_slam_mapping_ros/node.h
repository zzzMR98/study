
#ifndef CARTOGRAPHER_ROS_NODE_H_
#define CARTOGRAPHER_ROS_NODE_H_
///
///本节点主要用于接收各消息，将各消息数据加入到ｓｌａｍ系统中来并进行初步的处理。
///sensor_id为话题名；
///trajectory_id为当前轨迹名，在现有功能下，我们只有一条轨迹，因此，trajectory_id一直是０；
///
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// #include "iv_slam_mapping/common/mutex.h"
// #include "iv_slam_mapping/common/time.h"
#include "ivcommon/common/mutex.h"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping_ros/node_constants.h"
#include "iv_slam_mapping_ros/node_options.h"
#include "iv_slam_mapping_ros/trajectory_options.h"
#include "common/port.h"
#include "iv_slam_ros_msgs/SensorTopics.h"
#include "iv_slam_ros_msgs/TraversableArea.h"
#include "iv_slam_ros_msgs/TrajectoryOptions.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "covgrid_slam_msgs/LidarOdometryForMapping.h"
#include "compressed_pointcloud_msgs/Compressed_PointCloud.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include <time.h>
#include "iv_slam_mapping/sensor/odometry_data.h"
#include "ivcommon/transform/rigid_transform.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/mapping/map_builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "iv_slam_mapping/sensor/data.h"
#include "iv_dynamicobject_msgs/moving_target_send.h"

namespace iv_slam_mapping_ros
{
  class Node
  {
  public:
    Node(const NodeOptions &node_options, const TrajectoryOptions &trajectory_options, tf2_ros::Buffer *tf_buffer, ros::NodeHandle &nh_); ///构造函数
    ~Node();                                                                                                                              ///析构函数
    Node(const Node &) = delete;                                                                                                          ///不允许对象复制
    Node &operator=(const Node &) = delete;                                                                                               ///不允许对象复制

    void StartMappingWithDefaultTopics(); ///以默认话题设置开始建图
    ///
    ///返回当前frame_id和tracking_frame_之间的transform
    ///
    std::unique_ptr<ivcommon::transform::Rigid3d> LookupToTracking(::ivcommon::Time time, const string &frame_id) const;
    ///
    ///将nav_msgs::Odometry类型的里程计消息转为本系统格式的里程计消息，当前已弃用，之所以保留是减少后人再用到此消息格式时的编程量
    ///将covgrid_slam_msgs::LidarOdometryForMapping类型的里程计消息转为本系统格式的里程计消息，服务于接收激光雷达里程计消息
    ///
    std::unique_ptr<::iv_slam_mapping::sensor::LidarOdometryData> ToLidarOdometryData(const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr &msg);
    ///
    ///将sensor_driver_msgs::GpswithHeading类型的融合定位模块消息转为本系统格式的消息，用于为地图提供较为准确的全局位姿
    ///
    std::unique_ptr<::iv_slam_mapping::sensor::OdometryData> ToLocationModuleData(const sensor_driver_msgs::GpswithHeading::ConstPtr &msg);
    ///
    ///接下来的一组函数用于将各个传感器的消息加入到当前的路线上来。
    ///
    ///用于接收并处理covgrid_slam_msgs::LidarOdometryForMapping类型的激光雷达里程计消息，正在使用
    ///
    void HandleLidarOdometryMessage(const int trajectory_id, const string &sensor_id, const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr &msg);
    ///
    ///用于接收并处理融合定位模块数据，正在使用
    ///
    void HandleLocationModuleMessage(const int trajectory_id, const string &sensor_id, const sensor_driver_msgs::GpswithHeading::ConstPtr &msg);
    ///
    ///用于接收并处理激光雷达点云数据
    ///
    void HandlePointCloud2Message(int trajectory_id, const string &sensor_id, const sensor_msgs::PointCloud2::ConstPtr &msg);
    ///
    ///用于接收并处理压缩了的激光雷达点云数据
    ///
    void HandleCompressedPointCloud2Message(int trajectory_id, const string &sensor_id, const compressed_pointcloud_msgs::Compressed_PointCloud::ConstPtr &msg);
    ///
    ///将激光雷达数据传递到trajectory_builder_中去。
    ///
    void HandleRangefinder(const string &sensor_id, const ::ivcommon::Time time, const string &frame_id, const iv_slam_mapping::sensor::PointCloud &ranges);
    ///
    ///用于接收并处理动态障碍物数据
    ///
    void HandleDynamicObjectMessage(int trajectory_id, const string &sensor_id, const iv_dynamicobject_msgs::moving_target_send::ConstPtr &msg);

  private:
    int AddTrajectory(const TrajectoryOptions &options, const iv_slam_ros_msgs::SensorTopics &topics); ///添加路线
        ///
    ///返回我们我想要订阅的话题的集合
    ///
    std::unordered_set<string> ComputeExpectedTopics(const TrajectoryOptions &options, const iv_slam_ros_msgs::SensorTopics &topics);
    ///
    ///启动话题订阅
    ///
    void LaunchSubscribers(const iv_slam_ros_msgs::SensorTopics &topics, int trajectory_id);

    const NodeOptions node_options_;               ///外边传入的值
    const TrajectoryOptions trajectory_options_;   ///外边传入的值
    const tf2_ros::Buffer *tf_buffer_;             ///外边传入的值
    tf2_ros::TransformBroadcaster tf_broadcaster_; ///外边传入的值
    ::ros::NodeHandle node_handle_;                ///外边传入的值
    ::ivcommon::Mutex mutex_;         ///互斥锁
    iv_slam_mapping::mapping::MapBuilder map_builder_;
    std::unordered_map<int, std::vector<::ros::Subscriber>> subscribers_;
    ::iv_slam_mapping::mapping::TrajectoryBuilder *trajectory_builder_;
    ::ivcommon::Time last_dynamic_objects_time = ::ivcommon::Time::min();
  };
} // namespace iv_slam_mapping_ros
#endif // CARTOGRAPHER_ROS_NODE_H_

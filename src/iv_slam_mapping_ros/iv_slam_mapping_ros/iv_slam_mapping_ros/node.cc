
#include "iv_slam_mapping_ros/node.h"
#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "Eigen/Core"
// #include "iv_slam_mapping/common/configuration_file_resolver.h"
// #include "iv_slam_mapping/common/lua_parameter_dictionary.h"
// #include "iv_slam_mapping/common/make_unique.h"
// #include "iv_slam_mapping/common/port.h"
// #include "iv_slam_mapping/common/time.h"
// #include "iv_slam_mapping/common/time_conversion.h"
// #include "iv_slam_mapping/common/file_directory_generation.h"
#include "ivcommon/common/configuration_file_resolver.h"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/port.h"
#include "ivcommon/common/time.h"
#include "ivcommon/common/time_conversion.h"
#include "ivcommon/common/file_directory_generation.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "iv_slam_mapping_ros/msg_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Image.h"
#include "iv_slam_mapping/mapping_3d/hybrid_grid.h"
#include "iv_slam_mapping/mapping/probability_values.h"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <ros/package.h>
#include <unistd.h>
#include "iv_slam_mapping/mapping/collated_trajectory_builder.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
namespace iv_slam_mapping_ros
{
  namespace
  {
    ///
    ///检验个话题名字中是否存在斜杠“/”
    ///
    const string &CheckNoLeadingSlash(const string &frame_id)
    {
      if (frame_id.size() > 0)
      {
        CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id << " should not start with a /. See 1.7 in http://wiki.ros.org/tf2/Migration.";
      }
      return frame_id;
    }

    ///
    ///建立默认话题
    ///其中ｉｍｕ被注释掉了，如果想用，取消注释即可
    ///
    iv_slam_ros_msgs::SensorTopics DefaultSensorTopics()
    {
      iv_slam_ros_msgs::SensorTopics topics;
      topics.point_cloud2_topic = kPointCloud2Topic;
      //   topics.imu_topic = kImuTopic;
      topics.odometry_topic = kOdometryTopic;
      topics.lidar_odometry_topic = kLidarOdometryTopic;
      topics.dynamicobject_topic = kDynamicObjectTopic;
      topics.locationmodule_topic = kLocationModuleTopic;
      return topics;
    }
    ///
    ///使用 lamda 表达式注册消息订阅者，顺便也注册了消息处理函数，ｌａｍｄａ表达式具体用法请自行学习
    ///
    template <typename MessageType>
    ::ros::Subscriber SubscribeWithHandler(void (Node::*handler)(int, const string &, const typename MessageType::ConstPtr &), const int trajectory_id, const string &topic, ::ros::NodeHandle *const node_handle, Node *const node)
    {
      return node_handle->subscribe<MessageType>(topic, 2, boost::function<void(const typename MessageType::ConstPtr &)>([node, handler, trajectory_id, topic](const typename MessageType::ConstPtr &msg) { (node->*handler)(trajectory_id, topic, msg); }));
    }
  } // namespace

  namespace carto = ::iv_slam_mapping; ///定义命名空间
  using ::ivcommon::transform::Rigid3d;     ///使用命名空间

  ///
  ///构造函数，进行成员初始化
  ///
  Node::Node(const NodeOptions &node_options, const TrajectoryOptions &trajectory_options, tf2_ros::Buffer *const tf_buffer, ros::NodeHandle &nh_)
      : node_options_(node_options), trajectory_options_(trajectory_options), tf_buffer_(tf_buffer), node_handle_(nh_), map_builder_(node_options.map_builder_options, nh_, trajectory_options_.wiping_movingtaget)
  {
  }

  Node::~Node() {}
  ///
  ///按需加入话题，返回我们我想要订阅的话题的集合
  ///
  std::unordered_set<string> Node::ComputeExpectedTopics(const TrajectoryOptions &options, const iv_slam_ros_msgs::SensorTopics &topics)
  {
    std::unordered_set<string> expected_topics;
    expected_topics.insert(topics.point_cloud2_topic);
    //     expected_topics.insert(topics.imu_topic);
    if (options.wiping_movingtaget)
    {
      expected_topics.insert(topics.dynamicobject_topic);
    }
    if (options.use_lidar_odometry)
    {
      expected_topics.insert(topics.lidar_odometry_topic);
    }
    if (options.use_location_module)
    {
      expected_topics.insert(topics.locationmodule_topic);
    }
    return expected_topics;
  }

  ///
  ///以默认话题设置开始建图
  ///
  void Node::StartMappingWithDefaultTopics()
  {
    ::ivcommon::MutexLocker lock(&mutex_);

    iv_slam_ros_msgs::SensorTopics topics = DefaultSensorTopics();                                                                    ///获得默认话题配置
    const std::unordered_set<string> expected_sensor_ids = ComputeExpectedTopics(trajectory_options_, topics);                        ///根据配置文档加入需要的话题
    const int trajectory_id = map_builder_.AddTrajectoryBuilder(expected_sensor_ids, trajectory_options_.trajectory_builder_options); ///添加ｓｌａｍ路线
    trajectory_builder_ = map_builder_.GetTrajectoryBuilder(trajectory_id);                                                           ///获得路线创建者对象
    LaunchSubscribers(topics, trajectory_id);                                                                                         ///启动话题订阅
  }
  ///
  ///启动话题订阅
  ///
  void Node::LaunchSubscribers(const iv_slam_ros_msgs::SensorTopics &topics, const int trajectory_id)
  {
    ///
    ///亚索点云或者未压缩的普通点云二选一
    ///
    if (trajectory_options_.use_compressed_pointcloud)
    {
      ///
      ///注册订阅者以及相应的回调函数
      ///
      subscribers_[trajectory_id].push_back(SubscribeWithHandler<compressed_pointcloud_msgs::Compressed_PointCloud>(
          &Node::HandleCompressedPointCloud2Message, trajectory_id, topics.point_cloud2_topic, &node_handle_, this));
    }
    else
    {
      ///
      ///注册订阅者以及相应的回调函数
      ///
      subscribers_[trajectory_id].push_back(SubscribeWithHandler<sensor_msgs::PointCloud2>(&Node::HandlePointCloud2Message,
                                                                                           trajectory_id, topics.point_cloud2_topic, &node_handle_, this));
    }

    ///
    ///选择具有先验地图模式的里程计消息
    ///
    if (trajectory_options_.use_lidar_odometry)
    {
      string topic = topics.lidar_odometry_topic;
      ///
      ///注册订阅者以及相应的回调函数
      ///
      subscribers_[trajectory_id].push_back(SubscribeWithHandler<covgrid_slam_msgs::LidarOdometryForMapping>(&Node::HandleLidarOdometryMessage, trajectory_id, topic, &node_handle_, this));
    }
    ///
    ///选择动态障碍物消息
    ///
    if (trajectory_options_.wiping_movingtaget)
    {
      string topic = topics.dynamicobject_topic;
      ///
      ///注册订阅者以及相应的回调函数
      ///
      subscribers_[trajectory_id].push_back(SubscribeWithHandler<iv_dynamicobject_msgs::moving_target_send>(&Node::HandleDynamicObjectMessage, trajectory_id, topic, &node_handle_, this));
    }
    ///
    ///选择普融合定位模块消息
    ///
    if (trajectory_options_.use_location_module)
    {
      string topic = topics.locationmodule_topic;
      ///
      ///注册订阅者以及相应的回调函数
      ///
      subscribers_[trajectory_id].push_back(SubscribeWithHandler<sensor_driver_msgs::GpswithHeading>(&Node::HandleLocationModuleMessage, trajectory_id, topic, &node_handle_, this));
    }
  }
  ///
  ///用于接收并处理covgrid_slam_msgs::LidarOdometryForMapping类型的激光雷达里程计消息，正在使用
  ///
  void Node::HandleLidarOdometryMessage(const int trajectory_id, const string &sensor_id, const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr &msg)
  {
    ::ivcommon::MutexLocker lock(&mutex_);
    LOG(INFO) << "odometry time delay:" << (ros::Time::now() - msg->odometry.header.stamp).toSec();
    ///
    ///消息类型转换
    ///
    std::unique_ptr<::iv_slam_mapping::sensor::LidarOdometryData> lidar_odometry_data = ToLidarOdometryData(msg);
    if (lidar_odometry_data != nullptr)
    {
      ///
      ///将数据加入到trajectory_builder_中
      ///
      trajectory_builder_->AddLidarOdometerData(sensor_id, lidar_odometry_data->time, lidar_odometry_data->pose, lidar_odometry_data->GPS, lidar_odometry_data->indexs, lidar_odometry_data->mode);
    }
  }
  ///
  ///用于接收并处理融合定位模块数据，正在使用
  ///
  void Node::HandleLocationModuleMessage(const int trajectory_id, const string &sensor_id, const sensor_driver_msgs::GpswithHeading::ConstPtr &msg)
  {
    ::ivcommon::MutexLocker lock(&mutex_);
    LOG(INFO) << "LocationModuleMessage msg delay:" << (ros::Time::now() - msg->gps.header.stamp).toSec();
    if (msg->gps.longitude <= 0 || msg->gps.latitude <= 0)
    {
      LOG(WARNING) << "LocationModuleMessage msg invlide";
      return;
    }
    ///
    ///消息类型转换
    ///
    std::unique_ptr<::iv_slam_mapping::sensor::OdometryData> location_module_data = ToLocationModuleData(msg);
    if (location_module_data != nullptr)
    {
      ///
      ///将数据加入到wrapped_trajectory_builder_，即GlobalTrajectoryBuilder中
      ///
      map_builder_.trajectory_builders_.back()->wrapped_trajectory_builder_->AddLocationModuleData(location_module_data->time, iv_slam_mapping::sensor::Data::OdometrydData{location_module_data->pose});
    }
  }
  ///
  ///用于接收并处理动态障碍物数据
  ///
  void Node::HandleDynamicObjectMessage(int trajectory_id, const string &sensor_id, const iv_dynamicobject_msgs::moving_target_send::ConstPtr &msg)
  {
    ::ivcommon::MutexLocker lock(&mutex_);
    LOG(INFO) << "HandleDynamicObjectMessage msg delay:" << (ros::Time::now() - ros::Time(msg->time_stamp)).toSec();
    iv_slam_mapping::sensor::Data::DynamicObject dynamic_object;
    ros::Time tem_time;
    tem_time.fromSec(msg->time_stamp);
    ///
    ///时间格式转换
    ///
    dynamic_object.time = ::ivcommon::FromRos(tem_time);
    ///
    ///过滤旧的动态障碍物信息
    ///
    if (dynamic_object.time <= last_dynamic_objects_time)
    {
      return;
    }
    ///
    ///解析动态障碍物数据
    ///
    ///每个动态障碍物数据都包括：
    ///动态障碍物数目
    ///每一个动态障碍物是够被更新（is_updated），以及每一个动态障碍物的历史跟踪次数（history_num）
    ///然后解析每一个动态障碍物的每一个历史位置的四个点
    ///最终将解析结果传送到wrapped_trajectory_builder_，即GlobalTrajectoryBuilder中
    ///
    last_dynamic_objects_time = dynamic_object.time;
    dynamic_object.target_num = msg->target_num;

    for (int i = 0; i < dynamic_object.target_num; i++)
    {
      iv_slam_mapping::sensor::Data::DynamicObject::MovingTarget tem_moving_target;

      tem_moving_target.is_updated = msg->target.at(i).is_updated;
      tem_moving_target.history_num = msg->target.at(i).history_num;

      for (int j = 0; j < tem_moving_target.history_num; j++)
      {
        iv_slam_mapping::sensor::Data::DynamicObject::MovingTarget::HistoryTraj tem_history_traj;
        for (int k = 0; k < 4; k++)
        {
          tem_history_traj.points.push_back(Eigen::Vector3d(msg->target.at(i).history_traj.at(j).line_point.at(k).x,
                                                            msg->target.at(i).history_traj.at(j).line_point.at(k).y, msg->target.at(i).history_traj.at(j).line_point.at(k).z));
        }
        tem_moving_target.history_traj.push_back(tem_history_traj);
      }
      dynamic_object.moving_target.push_back(tem_moving_target);
    }
    map_builder_.trajectory_builders_.back()->wrapped_trajectory_builder_->AddDynamicObjectData(dynamic_object.time, dynamic_object);
  }
  ///
  ///用于接收并处理压缩了的激光雷达点云数据
  ///
  void Node::HandleCompressedPointCloud2Message(int trajectory_id, const string &sensor_id, const compressed_pointcloud_msgs::Compressed_PointCloud::ConstPtr &msg)
  {
    ::ivcommon::MutexLocker lock(&mutex_);
    LOG(INFO) << "range data time delay: " << (ros::Time::now() - msg->header.stamp).toSec();
    iv_slam_mapping::sensor::proto::CompressedRangeData tem_CompressedRangeData_proto;
    ///
    ///从字符串类型的数据中解析成ｐｒｏｔｏ格式的压缩点云
    ///
    tem_CompressedRangeData_proto.ParseFromString(msg->string.data);
    ///
    ///从ｐｒｏｔｏ格式的压缩点云解析成ｓｅｎｓｏｒ格式的压缩点云
    ///
    iv_slam_mapping::sensor::CompressedRangeData temCompressedRangeData = iv_slam_mapping::sensor::FromProto(tem_CompressedRangeData_proto);
    ///
    ///解析压缩点云为普通点云
    ///
    iv_slam_mapping::sensor::RangeData rangedata = iv_slam_mapping::sensor::Decompress(temCompressedRangeData);
    HandleRangefinder(sensor_id, ::ivcommon::FromRos(msg->header.stamp), msg->header.frame_id, rangedata.returns);
  }
  ///
  ///解析点云数据为各角度的点云，并将点云属性点(range = -0.5;)区分出来
  ///
  void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &outputclouds, std::vector<pcl::PointXYZI> &lidarpropertys)
  {
    //////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////
    int cloudnum = inputcloud->size() % 16; ///包含的点云包数目
    std::vector<int> startnum;
    for (int i = 0; i < cloudnum; i++)
    {
      pcl::PointXYZI originpoint;
      int flag = (*inputcloud)[inputcloud->size() - cloudnum + i].range; ///每一包点云的第一个点的位置
      (*inputcloud)[inputcloud->size() - cloudnum + i].range = -0.5;
      originpoint.x = (*inputcloud)[inputcloud->size() - cloudnum + i].x;               ///每一包点云中对应的雷达在车体坐标系的x
      originpoint.y = (*inputcloud)[inputcloud->size() - cloudnum + i].y;               ///每一包点云中对应的雷达在车体坐标系的y
      originpoint.z = (*inputcloud)[inputcloud->size() - cloudnum + i].z;               ///每一包点云中对应的雷达在车体坐标系的z
      originpoint.intensity = (*inputcloud)[inputcloud->size() - cloudnum + i].azimuth; ///每一包点云中对应的雷达线束
      startnum.push_back(flag);
      lidarpropertys.push_back(originpoint);
    }

    for (int i = 0; i < startnum.size(); i++)
    {
      int length;
      pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloudptr(new pcl::PointCloud<pcl::PointXYZI>); ///每一包点云

      if (i == startnum.size() - 1)
      {
        length = inputcloud->size() - cloudnum - startnum.at(i);
      }
      else
      {
        length = startnum.at(i + 1) - startnum.at(i);
      }
      lasercloudptr->insert(lasercloudptr->begin(), inputcloud->begin() + startnum.at(i), inputcloud->begin() + startnum.at(i) + length);
      outputclouds.push_back(lasercloudptr);
    }
  }
  ///
  ///用于接收并处理激光雷达点云数据
  ///
  void Node::HandlePointCloud2Message(const int trajectory_id, const string &sensor_id, const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    // clock_t start_time = clock();
    ::ivcommon::MutexLocker lock(&mutex_);
    LOG(INFO) << "range data time delay: " << (ros::Time::now() - msg->header.stamp).toSec();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ///
    ///点云格式转换
    ///
    pcl::fromROSMsg(*msg, *pcl_point_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds; ///用于存储各角度的点云
    std::vector<pcl::PointXYZI> lidarpropertys;                     ///用于存储每一个角度点云的起始位置及其所在线束
                                                                    ///
    ///解析点云数据为各角度的点云，并将点云属性点区分出来
    ///
    analysisCloud(pcl_point_cloud, outputclouds, lidarpropertys);
    iv_slam_mapping::sensor::PointCloud point_cloud;
    ///
    ///如果没有点云则返回
    ///
    if (outputclouds.size() == 0)
      return;

    // std::string tem_filename = "/home/zack-liu/test/liukai.txt";
    // std::fstream tem_filestream;
    // tem_filestream.open(tem_filename,std::ios::out|std::ios::app);
    // for(int i =  0;i<outputclouds.size();i++){
    //   for(const auto& point : *outputclouds[i]){
    //     if(point.range < 0.5)continue;
    //     tem_filestream<<std::fixed<<std::setprecision(10)<<msg->header.stamp
    // <<'\t'<< point.x<<'\t'<< point.y<<'\t'<< point.z<<'\n';
    //   }
    // }
    // tem_filestream.close();
    ///
    ///滤除点云属性点(range = -0.5;)
    ///
    for (int i = 0; i < outputclouds.size(); i++)
    {
      for (const auto &point : *outputclouds[i])
      {
        if (point.range < 0.5 || point.z > 2.5 || point.y < -20)
          continue;
        point_cloud.emplace_back(point.x, point.y, point.z);
      }
    }
    //   for (const auto& point : *outputclouds[0]) {
    //     if(point.range < 0.5||point.z>2.5||point.y<-20)continue;
    //     point_cloud.emplace_back(point.x, point.y, point.z);
    //   }

    HandleRangefinder(sensor_id, ::ivcommon::FromRos(msg->header.stamp), msg->header.frame_id, point_cloud);
    // clock_t end_time = clock();
    // ROS_INFO("iv_slam_mapping_ros time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
  }
  ///
  ///将激光雷达数据传递到trajectory_builder_中去。
  ///
  void Node::HandleRangefinder(const string &sensor_id, const ::ivcommon::Time time, const string &frame_id, const iv_slam_mapping::sensor::PointCloud &ranges)
  {
    trajectory_builder_->AddRangefinderData(sensor_id, time, Eigen::Vector3f::Zero(), ranges);
  }
  ///
  ///返回当前frame_id和tracking_frame_之间的transform
  ///
  std::unique_ptr<ivcommon::transform::Rigid3d> Node::LookupToTracking(const ::ivcommon::Time time, const string &frame_id) const
  {
    ::ros::Duration timeout(node_options_.lookup_transform_timeout_sec);
    std::unique_ptr<ivcommon::transform::Rigid3d> frame_id_to_tracking;
    try
    {
      const ::ros::Time latest_tf_time =
          tf_buffer_
              ->lookupTransform(trajectory_options_.tracking_frame, frame_id, ::ros::Time(0.),
                                timeout)
              .header.stamp;
      const ::ros::Time requested_time = ::ivcommon::ToRos(time);
      if (latest_tf_time >= requested_time)
      {
        // We already have newer data, so we do not wait. Otherwise, we would wait
        // for the full 'timeout' even if we ask for data that is too old.
        timeout = ::ros::Duration(0.);
      }
      return ::ivcommon::make_unique<ivcommon::transform::Rigid3d>(ToRigid3d(tf_buffer_->lookupTransform(
          trajectory_options_.tracking_frame, frame_id, requested_time, timeout)));
    }
    ///
    ///捕获异常信息
    ///
    catch (const tf2::TransformException &ex)
    {
      LOG(WARNING) << ex.what();
    }
    return nullptr;
  }

  std::unique_ptr<::iv_slam_mapping::sensor::LidarOdometryData> Node::ToLidarOdometryData(const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr &msg)
  {
    const ::ivcommon::Time time = ::ivcommon::FromRos(msg->odometry.header.stamp);
    ///
    ///返回当前frame_id和tracking_frame_之间的transform
    ///
    const auto sensor_to_tracking = LookupToTracking(time, CheckNoLeadingSlash(msg->odometry.child_frame_id));
    if (sensor_to_tracking == nullptr)
    {
      return nullptr;
    }
    std::vector<int> indexs;
    indexs.clear();
    for (int i = 0; i < msg->indexs.size(); i++)
    {
      indexs.push_back(msg->indexs[i]);
    }

    return ::ivcommon::make_unique<::iv_slam_mapping::sensor::LidarOdometryData>(::iv_slam_mapping::sensor::LidarOdometryData{time, ToRigid3d(msg->odometry.pose.pose),
                                                                                                                                             Eigen::Vector3d(msg->gps.longitude, msg->gps.latitude, msg->gps.altitude), indexs, msg->mode});
  }

  std::unique_ptr<iv_slam_mapping::sensor::OdometryData> Node::ToLocationModuleData(const sensor_driver_msgs::GpswithHeading::ConstPtr &msg)
  {
    const ::ivcommon::Time time = ::ivcommon::FromRos(msg->gps.header.stamp);
    Eigen::Vector3d tem_pose = Eigen::Vector3f(msg->pitch, msg->roll, msg->heading).cast<double>();
    Eigen::Vector3d tem_location = Eigen::Vector3f(msg->gps.longitude, msg->gps.latitude, msg->gps.altitude).cast<double>();
    return ::ivcommon::make_unique<::iv_slam_mapping::sensor::OdometryData>(
        ::iv_slam_mapping::sensor::OdometryData{time, ivcommon::transform::Rigid3d(Eigen::Vector3d(tem_location.x(), tem_location.y(), tem_location.z()),
                                                                                            ivcommon::transform::PitchRollYaw(tem_pose.x(), tem_pose.y(), tem_pose.z()))});
  }
} // namespace iv_slam_mapping_ros

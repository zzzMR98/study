///
///此为准可通行区域提取的主节点，主要作用是加载配置文档
///１、初始化ｇｌｏｇ
///２、初始化ｒｏｓ节点
///３、加载配置文档
///４、初始化ｔf_buffer
///５、初始化节点　Ｎｏｄｅ对象
///６、使用多线程订阅各消息
///
///
#include "iv_slam_mapping_ros/node.h"
#include "iv_slam_mapping_ros/node_options.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <linux/input.h>
#include <fcntl.h>
#include "opencv2/core.hpp"

#if CV_VERSION_EPOCH == 2
#define OPENCV2
#elif CV_VERSION_MAJOR == 3
#define OPENCV3
#else
#error Not support this OpenCV version
#endif

///
///利用 gflag 提供的宏定义参数slam_configuration_directory,
///该宏的 3 个参数分别为命令行参数名，参数默认值，参数的帮助信息。
///
DEFINE_string(slam_configuration_directory, "", "First directory in which configuration files are searched, second is always the Cartographer installation to allow including files from there.");
///
///利用 gflag 提供的宏定义参数slam_configuration_basename,
///该宏的 3 个参数分别为命令行参数名，参数默认值，参数的帮助信息。
///
DEFINE_string(slam_configuration_basename, "", "Basename, i.e. not containing any directory prefix, of the configuration file.");

namespace iv_slam_mapping_ros {
namespace {
///
///主函数入口
///
void Run() {
  ///
  ///初始化tf2_ros::Buffer：tf_buffer
  ///tf_buffer 保存时间为１０^６ｓ
  ///
  constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  tf2_ros::Buffer tf_buffer{ ::ros::Duration(kTfBufferCacheTimeInSeconds) };
  ///
  ///初始化tf2_ros::TransformListener： tf
  ///
  tf2_ros::TransformListener tf(tf_buffer);
  ///
  ///建立对象　node_options
  ///
  NodeOptions node_options;
  ///
  ///建立对象　trajectory_options
  ///
  TrajectoryOptions trajectory_options;
  ///
  ///加载配置文件参数到　node_options, trajectory_options；
  ///
  std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_slam_configuration_directory, FLAGS_slam_configuration_basename);
  ///
  ///建立NodeHandle对象　nh，用于调用ｒｏｓ资源
  ///
  ros::NodeHandle nh;
  ///
  ///建立Node对象　node
  ///
  Node node(node_options, trajectory_options, &tf_buffer, nh);
  node.StartMappingWithDefaultTopics();
  ///
  ///使用多线程订阅各消息
  ///
  ros::AsyncSpinner spinner(6);
  ///
  ///使用默认话题开始建图及准可通行区域提取，话题配置可以再配置文件中配置
  ///开始多线程处理各消息
  spinner.start();
  ///
  ///等待ｒｏｓ关闭
  ///
  ros::waitForShutdown();
}
} // namespace
} // namespace iv_slam_mapping_ros
///
///主程序main函数
///
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);                ///初始化ｇｌｏｇ
  google::ParseCommandLineFlags(&argc, &argv, true); ///解析命令行参数
                                                     ///
  ///检查是否提供了配置文件所在文件夹
  ///
  CHECK(!FLAGS_slam_configuration_directory.empty())
    << "-configuration_directory is missing.";
  ///
  ///检查是否提供了配置文件名
  ///
  CHECK(!FLAGS_slam_configuration_basename.empty())
    << "-configuration_basename is missing.";
  ///
  ///初始化ｒｏｓ节点，节点名为“iv_slam_mapping_node”
  ///
  ::ros::init(argc, argv, "iv_slam_mapping_node");
  ::ros::start(); ///开启ｒｏｓ节点

  iv_slam_mapping_ros::Run(); ///主程序入口
  ::ros::shutdown();          ///关闭ｒｏｓ节点
}

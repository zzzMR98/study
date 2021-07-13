#include "node.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2_ros/transform_listener.h"
#include <linux/input.h>
#include <fcntl.h>
#include "node_options.h"

#if CV_VERSION_EPOCH == 2
#define OPENCV2
#elif CV_VERSION_MAJOR == 3
#define OPENCV3
#else
#error Not support this OpenCV version
#endif

///
///此为可通行区域提取程序.主要技术方案为接收准可通行区域提取模块的准可通行区域信息作为可通行区域基础，
///然后综合负障碍／正障碍／斜坡／悬崖／非平台区域检测等各模块的检测信息，经过优化处理形成可通行区域．本程序
///包括在线可通行区域提取以及先验信息利用两种模式．
///

DEFINE_string(traversable_area_extraction_configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(traversable_area_extraction_configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
namespace traversable_area_extraction
{
  void Run()
  {
    TraversableAreaOption traversablearea_option;
    traversablearea_option =
        LoadOptions(FLAGS_traversable_area_extraction_configuration_directory, FLAGS_traversable_area_extraction_configuration_basename);//加载配置文件
    ros::NodeHandle nh;
    LOG(INFO) << "traversablearea_option loaded";
    Node node(nh, traversablearea_option);
    // ros::MultiThreadedSpinner spinner(5);
    // spinner.spin();
    // ros::spin();
    ros::AsyncSpinner spinner(6);//多线程订阅消息
    spinner.start();
    ros::waitForShutdown();
  }
} //namespace traversable_area_extraction

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);//初始化glog
  google::ParseCommandLineFlags(&argc, &argv, true);//初始化gflags 处理命令行传入的参数，使得定义的FLAG参数得到正确赋值

  google::InstallFailureSignalHandler();//在程序出现严重错误时将详细的错误信息打印出来
  CHECK(!FLAGS_traversable_area_extraction_configuration_directory.empty())//flag的默认值是空的 所以从什么地方传入呢 zmr?
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_traversable_area_extraction_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "traversibal_area_extraction_main");
  ::ros::start();
  LOG(INFO) << "started";
  //   iv_slam_mapping_ros::ScopedRosLogSink ros_log_sink;
  traversable_area_extraction::Run();
  google::ShutdownGoogleLogging();
  ::ros::shutdown();
}

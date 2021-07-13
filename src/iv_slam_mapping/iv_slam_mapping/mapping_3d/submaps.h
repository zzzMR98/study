#ifndef CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
#include <memory>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include "Eigen/Geometry"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/image_encodings.h>
#include "ivcommon/common/port.h"
#include "ivcommon/common/time_conversion.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "ivcommon/transform/utm/utm.h"
#include "ivcommon/io/proto_stream.h"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "iv_slam_mapping/mapping/proto/serialization.pb.h"
#include "iv_slam_mapping/mapping/submaps.h"
#include "iv_slam_mapping/sensor/odometry_data.h"
#include "iv_slam_mapping/mapping_3d/hybrid_grid.h"
#include "iv_slam_mapping/mapping_3d/proto/submaps_options.pb.h"
#include "iv_slam_mapping/mapping_3d/proto/submap.pb.h"
#include "iv_slam_mapping/mapping_3d/range_data_inserter.h"
#include "iv_slam_mapping/sensor/range_data.h"
#include "iv_slam_ros_msgs/PrimarytraversableArea.h"
#include "iv_slam_ros_msgs/Traversablevehiclepose.h"

namespace iv_slam_mapping {
namespace mapping_3d {

///
///主要用于从外面传入参数
///
struct ActivemapConstant {
  ros::Publisher vehicle_global_pose_pub;
  ros::Publisher submap_traversable_area_pub;
  iv_slam_ros_msgs::PrimarytraversableArea traversable_area_msg;
  ::ivcommon::transform::Rigid3d origin_position_pose;
  ::ivcommon::transform::GridZone gps_zone;
  std::vector<iv_slam_mapping::sensor::OdometryData> location_module_data;
  bool use_gps_location_module;
  bool priormapmode;
  int priormapindex;
  bool finished;
  bool traversable_area_display;             ///来自ｌｕａ配置文件，是否显示图片
  float kXrayObstructedCellProbabilityLimit; ///来自ｌｕａ配置文件，三维地图滤波概率
  bool priormap_save;                        ///来自ｌｕａ配置文件，是否保存先验地图
  int rough_intensity;                       ///来自ｌｕａ配置文件，用于斜坡检测，栅格粗化程度
  float kMinZDifference;                     ///来自ｌｕａ配置文件，普通高度差阈值
  float kMinZDifference_beyond;              ///来自ｌｕａ配置文件，远处高度差阈值
  int ZDifference_change_thresh;             ///来自ｌｕａ配置文件，改变高度差的地方距离当前车辆位置的距离范围
  float obstacle_emptythresh;                ///来自ｌｕａ配置文件，悬空检测阈值
  int extension_index;                       ///来自ｌｕａ配置文件，地图扩大阈值
  ///
  ///进行参数初始化
  ///
  ActivemapConstant() {
    use_gps_location_module = false;
    traversable_area_display = false;
    priormapmode = false;
    finished = false;
  }
};

struct TwidMapData {
  float resolution;                   ///分辨率
  int width;                          ///宽度
  int height;                         ///高度
  int twid_submap_pose_image_index_x; ///ｘ向索引数
  int twid_submap_pose_image_index_y; ///ｙ向索引书
  ivcommon::transform::Rigid3d twidmap_pose;    ///二维地图
  ivcommon::transform::Rigid3d gps_global_pose; ///全局位置
};

struct PixelData {
  int min_z = INT_MAX;
  int max_z = INT_MIN;
  int count = 0;                ///体素数
  float probability_sum = 0.f;  ///总概率
  float max_probability = 0.5f; ///最大概率
  int x_in_index = -1000;       ///ｘ向索引数
  int y_in_index = -1000;       ///ｙ向索引数
};
///
///从 lua 创建 SubmapsOptions
///
proto::SubmapsOptions CreateSubmapsOptions(::ivcommon::LuaParameterDictionary* parameter_dictionary);

class Submap : public mapping::Submap {
public:
  // 构造函数
  Submap(float high_resolution, float low_resolution, const ivcommon::transform::Rigid3d& local_pose);
  explicit Submap(const mapping::proto::Submap3D& proto); ///构造函数

  void ToProto(mapping_3d::proto::Submap3D* proto) const override; ///转为ｐｒｏｔｏ

  const HybridGrid& high_resolution_hybrid_grid() const { return high_resolution_hybrid_grid_; } ///返回高精度地图
  bool finished() const { return finished_; }                                                    ///地图是否完成标志
                                                                                                 ///
  /// Insert 'range_data' into this submap using 'range_data_inserter'. The
  /// submap must not be finished yet.
  ///进行三维概率栅格地图创建
  ///
  void InsertRangeData(const sensor::RangeData& range_data, const RangeDataInserter& range_data_inserter, int high_resolution_max_range);
  ///
  ///地图创建是否完成
  ///
  void Finish();

  ///
  ///保存三维地图
  ///
  void Trid_Submap_Write(std::string file_time_name_);
  ///
  ///保存可视化的三维地图
  ///
  void Visualization_Trid_Submap_Write(std::string file_time_name_);
  ///进行准可通行区域提取
  void Traversablearea_Extraction(const ::ivcommon::Time& time_, ActivemapConstant activemap_constant_,
    ivcommon::transform::Rigid3d range_pose_observation_, std::map<int, TwidMapData>* twid_map_data_, std::string file_time_name_, bool twid_map_write_flag_);
  HybridGrid high_resolution_hybrid_grid_;              ///高精度地图
  bool twid_display_thread_created;                     ///是否已经创建线程
  bool trid_write_thread_created;                       ///是否已经创建线程
  bool visualization_trid_write_thread_created;         ///是否已经创建线程
  Eigen::Vector3f current_vehicle_position_vframe;      ///当前车辆位置
  Eigen::Array3i current_vehicle_position_vframe_index; ///当前车辆位置索引数
  int identifier_index;                                 ///地图索引号
  cv::Mat Display_Image;
private:
  bool finished_ = false;
  // cv::Mat TwiDmapImage_;
};
///
/// Except during initialization when only a single submap exists, there are
/// always two submaps into which scans are inserted: an old submap that is used
/// for matching, and a new one, which will be used for matching next, that is
/// being initialized.
///
/// Once a certain number of scans have been inserted, the new submap is
/// considered initialized: the old submap is no longer changed, the "new" submap
/// is now the "old" submap and is used for scan-to-map matching. Moreover, a
/// "new" submap gets created. The "old" submap is forgotten by this object.
///
class ActiveSubmaps {
public:
  explicit ActiveSubmaps(const proto::SubmapsOptions& options, const ::ros::NodeHandle& activesubmap_nh); ///构造函数
  ActiveSubmaps(const ActiveSubmaps&) = delete; ///禁止复制对象
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete; ///禁止复制对象
  ~ActiveSubmaps(); ///析构函数
  ///
  /// Returns the index of the newest initialized Submap which can be
  /// used for scan-to-map matching.
  ///
  int matching_index() const;
  ///
  /// Inserts 'range_data' into the Submap collection.
  ///
  void InsertRangeData(const ::ivcommon::Time& time_, const sensor::RangeData& range_data,
    const ::ivcommon::transform::Rigid3d& activesubmap_pose_observation, const std::vector<int> current_mapping_indexs_, const int& mode);
  void visualizeSubmap();
  std::vector<std::shared_ptr<Submap>> submaps() const;                          ///获得当前正在维护的地图
  std::string file_time_name;                                                    ///先验地图文件名
  void AddSubmap(const ::ivcommon::transform::Rigid3d& local_pose, const int& submap_index); ///添加地图，同时删除上一个地图
  std::vector<std::shared_ptr<Submap>> submaps_;                                 ///当前正在维护的地图
  int inserted_rangedata_index;
  std::map<int, TwidMapData> twid_map_data;
  ActivemapConstant activemap_constant; ///配置参数常量
  // IplImage *Display_Image_;             ///显示图片
  int autonomousmapping_index;          ///自主建图索引号
private:
  const proto::SubmapsOptions options_; ///配置选项
  int matching_submap_index_ = 0;
  RangeDataInserter range_data_inserter_; ///点云插入器
  ::ros::NodeHandle activesubmap_nh_;     ///node_handle
  int last_mappingmode;                   ///上次建图模式
  int last_invoked_mapindex;              ///上次先验地图调用索引
  std::shared_ptr<boost::thread> visualize_thread; // OpenCV 可视化线程
  bool keep_visualize_thread_running;
};

} // namespace mapping_3d
} // namespace iv_slam_mapping

#endif // CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_

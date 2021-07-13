#include "iv_slam_mapping/mapping_3d/submaps.h"

#include <unistd.h>
#include <cmath>
#include <limits>
#include <time.h>
#include "glog/logging.h"
#include <fstream> //lzz
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <octomap/octomap.h>

#include "ivcommon/common/math.h"
#include "ivcommon/common/file_directory_generation.h"

bool traversable_area_extracted = false;
bool traversable_area_transfered = true;

namespace iv_slam_mapping {
namespace mapping_3d {

namespace {
// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses and reflectivity
// information.
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
  const float max_range) {
  sensor::RangeData result{ range_data.origin, {}, {} };
  for (const Eigen::Vector3f& hit : range_data.returns) {
    if ((hit - range_data.origin).norm() <= max_range) {
      result.returns.push_back(hit);
    }
  }
  return result;
}

std::vector<PixelData> AccumulatePixelData(
  const int width, const int height, const Eigen::Array2i& min_index,
  const Eigen::Array2i& max_index,
  const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
    voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability =
      mapping::ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  return accumulated_pixel_data;
}

// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
std::vector<Eigen::Array4i> ExtractVoxelData(
  const HybridGrid& hybrid_grid, const ::ivcommon::transform::Rigid3f& transform,
  Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
      hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    const Eigen::Array4i voxel_index_and_probability(
      ::ivcommon::RoundToInt(cell_center_global.x() * resolution_inverse),
      ::ivcommon::RoundToInt(cell_center_global.y() * resolution_inverse),
      ::ivcommon::RoundToInt(cell_center_global.z() * resolution_inverse),
      probability_value);

    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  return voxel_indices_and_probabilities;
}

// Builds texture data containing interleaved value and alpha for the
// visualization from 'accumulated_pixel_data'.
string ComputePixelValues(
  const std::vector<PixelData>& accumulated_pixel_data) {
  string cell_data;
  cell_data.reserve(2 * accumulated_pixel_data.size());
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      cell_data.push_back(0); // value
      cell_data.push_back(0); // alpha
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = mapping::ClampProbability(
      (pixel.probability_sum + free_space_probability * free_space_weight) /
      total_weight);
    // delta<0 IS hit;==0 (unkown);>0 miss
    const int delta =
      128 - mapping::ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cell_data.push_back(value);                        // value is the miss
    cell_data.push_back((value || alpha) ? alpha : 1); // alpha is the hit
  }
  return cell_data;
}
} // namespace

///
///从ｌｕａ创建SubmapsOptions
///
proto::SubmapsOptions CreateSubmapsOptions(::ivcommon::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_high_resolution(parameter_dictionary->GetDouble("high_resolution"));                     ///三维地图分辨率
  options.set_high_resolution_max_range(parameter_dictionary->GetDouble("high_resolution_max_range")); ///距离滤波范围
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(parameter_dictionary->GetNonNegativeInt("num_range_data")); ///帧数
  *options.mutable_range_data_inserter_options() =
    CreateRangeDataInserterOptions(parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  options.set_twid_submap_write(parameter_dictionary->GetBool("twid_submap_write"));                                       ///弃用
  options.set_trid_submap_write(parameter_dictionary->GetBool("trid_submap_write"));                                       ///是否保存三维地图
  options.set_visualization_trid_submap_write(parameter_dictionary->GetBool("visualization_trid_submap_write"));           ///是否保存可视化三维地图
  options.set_twid_submap_display(parameter_dictionary->GetBool("twid_submap_display"));                                   ///是否显示
  options.set_trid_submap_display(parameter_dictionary->GetBool("trid_submap_display"));                                   ///弃用
  options.set_enable_traversablearea_extraction(parameter_dictionary->GetBool("enable_traversablearea_extraction"));       ///是否提取准可通行区域
  options.set_kxrayobstructedcellprobabilitylimit(parameter_dictionary->GetDouble("kxrayobstructedcellprobabilitylimit")); ///三维地图概率绿宝阈值
  options.set_rough_intensity(parameter_dictionary->GetInt("rough_intensity"));                                            ///用于斜坡检测，栅格粗化程度
  options.set_kminzdifference(parameter_dictionary->GetDouble("kminzdifference"));                                         ///普通高度差阈值
  options.set_kminzdifference_beyond(parameter_dictionary->GetDouble("kminzdifference_beyond"));                           ///远处高度差阈值
  options.set_zdifference_change_thresh(parameter_dictionary->GetDouble("zdifference_change_thresh"));                     ///改变高度差的地方距离当前车辆位置的距离范围
  options.set_obstacle_emptythresh(parameter_dictionary->GetDouble("obstacle_emptythresh"));                               ///悬空检测阈值
  options.set_extension_index(parameter_dictionary->GetInt("extension_index"));                                            ///地图扩大阈值
  return options;
}

///
///构造函数
//////
Submap::Submap(const float high_resolution, const float low_resolution, const ivcommon::transform::Rigid3d& local_pose)
  : mapping::Submap(local_pose), high_resolution_hybrid_grid_(high_resolution) {
  twid_display_thread_created = false;                            ///参数初始化
  trid_write_thread_created = false;                              ///参数初始化
  visualization_trid_write_thread_created = false;                ///参数初始化
  current_vehicle_position_vframe = Eigen::Vector3f::Zero();      ///参数初始化
  current_vehicle_position_vframe_index = Eigen::Array3i::Zero(); ///参数初始化
}
///
///构造函数
//////
Submap::Submap(const mapping::proto::Submap3D& proto) : mapping::Submap(ivcommon::transform::ToRigid3(proto.local_pose())),
high_resolution_hybrid_grid_(proto.high_resolution_hybrid_grid()) {
  SetNumRangeData(proto.num_range_data());
  finished_ = proto.finished();
  twid_display_thread_created = false;                            ///参数初始化
  trid_write_thread_created = false;                              ///参数初始化
  visualization_trid_write_thread_created = false;                ///参数初始化
  current_vehicle_position_vframe = Eigen::Vector3f::Zero();      ///参数初始化
  current_vehicle_position_vframe_index = Eigen::Array3i::Zero(); ///参数初始化
}
///
///转为ｐｒｏｔｏ
///
void Submap::ToProto(mapping_3d::proto::Submap3D* const proto) const {
  *proto->mutable_local_pose() = ::ivcommon::transform::ToProto(local_pose());
  proto->set_num_range_data(num_range_data());
  proto->set_finished(finished_);
  *proto->mutable_high_resolution_hybrid_grid() = high_resolution_hybrid_grid().ToProto();
  //   proto->mutable_low_resolution_hybrid_grid() =      low_resolution_hybrid_grid().ToProto();
}

///
///进行概率滤波，滤除噪声较多的点
///滤除距离地图中心两侧太远以及太高的点
///将地图体素信息提取到一个ｖｅｃｔｏｒ容器中
///
std::vector<Eigen::Array4i> ProcessVoxelData(
    const HybridGrid& hybrid_grid,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index,
    Eigen::Array3i& current_vehicle_position_vframe_index_,
    Eigen::Vector3f& current_vehicle_position_vframe_, ActivemapConstant& activemap_constant) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();
  // constexpr float kXrayObstructedCellProbabilityLimit = /*0.501f*/0.750f  /*0.694f */ /*0.654f *//*0.551f*//*0.550000999999f*/;

  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {

    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);
    ///
    ///进行概率滤波，滤除噪声较多的点
    ///
    if (probability < activemap_constant.kXrayObstructedCellProbabilityLimit) {
      continue;
    }

    const Eigen::Vector3f cell_center_submap = hybrid_grid.GetCenterOfCell(it.GetCellIndex()); ///获取地图中心

    ///
    ///在线模式下滤除地图后２０米
    ///
    if (!activemap_constant.priormap_save &&
      cell_center_submap.y() <= current_vehicle_position_vframe_.y() - 20) {
      continue;
    }
    ///
    ///滤除距离地图中心两侧太远以及太高的点
    ///
    if (/*cell_center_submap.y()<-20||*/
      std::fabs(cell_center_submap.x()) > 60 || cell_center_submap.z() > 15) {
      continue;
    }
    ///
    ///将地图体素信息提取到一个ｖｅｃｔｏｒ容器中
    ///
    const Eigen::Array4i voxel_index_and_probability(::ivcommon::RoundToInt(
      cell_center_submap.x() * resolution_inverse),
      ::ivcommon::RoundToInt(cell_center_submap.y() * resolution_inverse),
      ::ivcommon::RoundToInt(cell_center_submap.z() * resolution_inverse),
      probability_value);
    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  ///
  ///获取车辆索引位置
  ///
  current_vehicle_position_vframe_index_ = (Eigen::Array3i){
    ::ivcommon::RoundToInt(current_vehicle_position_vframe_.x() * resolution_inverse),
    ::ivcommon::RoundToInt(current_vehicle_position_vframe_.y() * resolution_inverse),
    ::ivcommon::RoundToInt(current_vehicle_position_vframe_.z() * resolution_inverse) };
  return voxel_indices_and_probabilities;
}
///
///提取准可通行区域正障碍
/// 提取准可通行区域中的可通行区域
///提取准可通行区域悬空障碍
///
// void ProcessPixelValues(
//     const std::vector<PixelData> &accumulated_pixel_data, const float &resolution_, 
//     IplImage *temimage, const Eigen::Array3i &current_vehicle_position_vframe_index, 
//     ActivemapConstant &activemap_constant)
void ProcessPixelValues(
  const std::vector<PixelData>& accumulated_pixel_data, const float& resolution_, cv::Mat& temimage,
  const Eigen::Array3i& current_vehicle_position_vframe_index, ActivemapConstant& activemap_constant) {
  /*******************lzz10171036*******************/
  // cvZero(temimage);
  // temimage.setTo(cv::Scalar::all(0));
  // unsigned char *pdata = (unsigned char *)temimage->imageData;
  unsigned char* pdata;
  // const int image_height = temimage->height;
  // const int image_width = temimage->width;
  const int image_height = temimage.rows;
  const int image_width = temimage.cols;
  int image_index = 0;

  for (const PixelData& pixel : accumulated_pixel_data) {
    // pdata = (unsigned char *)temimage->imageData + 
    //   (temimage->height - 1 - image_index / temimage->width) * 
    //   temimage->widthStep + image_index % temimage->width;
    pdata = temimage.ptr<unsigned char>(image_height - 1 - image_index /
      image_width) + image_index % image_width;
    if (pixel.count == 0) ///未知区域
    {
      // *pdata =0; //0; //unknown area
      image_index++;
      continue;
    }
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    float free_index = z_difference > 0 ? (free_space / z_difference) : 0;

    if (((pixel.x_in_index - current_vehicle_position_vframe_index.x() >
      activemap_constant.ZDifference_change_thresh) ||
      (pixel.y_in_index - current_vehicle_position_vframe_index.y() >
        activemap_constant.ZDifference_change_thresh)) ? (z_difference >
          activemap_constant.kMinZDifference_beyond) :
      (z_difference > activemap_constant.kMinZDifference)) {
      if (free_index > activemap_constant.obstacle_emptythresh) {
        *pdata = 125; ///提取准可通行区域悬空障碍
      }
      else {
        *pdata = 254; ///提取准可通行区域正障碍
      }
      image_index++;
      continue;
    }
    else {
      *pdata = 100; /// 提取准可通行区域中的可通行区域
      image_index++;
      continue;
    }
  }
}
///
///提取三维地图图二维信息
///
std::vector<PixelData> Convert2PixelData(const int width, const int height,
  const Eigen::Array2i& min_index, const Eigen::Array2i& max_index,
  const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities,
  ActivemapConstant& activemap_constant) {
  std::vector<PixelData> accumulated_pixel_data(width * height);

  std::vector<PixelData> rough_accumulated_pixel_data(
    (width / activemap_constant.rough_intensity + 1) *
    (height / activemap_constant.rough_intensity + 1));
  for (const Eigen::Array4i& voxel_index_and_probability : voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();

    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      ///
      /// Out of bounds. This could happen because of floating point inaccuracy.
      ///
      continue;
    }
    const int x = pixel_index[0] - min_index.x();
    const int y = pixel_index[1] - min_index.y();
    PixelData& pixel = accumulated_pixel_data[(y * width + x)];
    pixel.x_in_index = pixel_index[0];
    pixel.y_in_index = pixel_index[1];
    PixelData& rough_pixel = rough_accumulated_pixel_data[
      (y / activemap_constant.rough_intensity *
        (width / activemap_constant.rough_intensity + 1) +
        x / activemap_constant.rough_intensity)]; ///用于斜坡检测

    ++pixel.count;
    rough_pixel.min_z = std::min(rough_pixel.min_z, voxel_index_and_probability[2]);            ///用于斜坡检测
    rough_pixel.max_z = std::max(rough_pixel.max_z, voxel_index_and_probability[2]);            ///用于斜坡检测
    pixel.min_z = rough_pixel.min_z; //std::min(pixel.min_z, voxel_index_and_probability[2]);///最小高度
    pixel.max_z = /*rough_pixel.max_z;*/ std::max(pixel.max_z, voxel_index_and_probability[2]); ///最大高度
    const float probability = mapping::ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;                                 ///概率和
    pixel.max_probability = std::max(pixel.max_probability, probability); ///最大概率
  }
  return accumulated_pixel_data;
}
///
///准可通行区域提取
///
void Submap::Traversablearea_Extraction(const ::ivcommon::Time& time_,
  ActivemapConstant activemap_constant_,
  ::ivcommon::transform::Rigid3d range_pose_observation_,
  std::map<int, TwidMapData>* twid_map_data_,
  std::string file_time_name_, bool twid_map_write_flag_) {
  iv_slam_ros_msgs::Traversablevehiclepose vehicle_global_pose_;
  traversable_area_extracted = false; ///参数初始化
  twid_display_thread_created = true;
  double a = 6378137;                                                                   ///参数初始化
  double e2 = 0.0818192 * 0.0818192;                                                    /////参数初始化　e的平方
  ::ivcommon::transform::GridZone zone = activemap_constant_.gps_zone;             ///参数初始化
  ::ivcommon::transform::Hemisphere hemi = ::ivcommon::transform::HEMI_NORTH; ///参数初始化

  if (activemap_constant_.priormapmode) {

    const float resolution = high_resolution_hybrid_grid_.resolution(); ///参数初始化
    //   Eigen::Array2i min_index(INT_MAX, INT_MAX);///参数初始化
    //   Eigen::Array2i max_index(INT_MIN, INT_MIN);///参数初始化
    //   Eigen::Array2i tem_submap_pose_index(INT_MIN, INT_MIN);///参数初始化

    ///
    ///进行概率滤波，滤除噪声较多的点
    ///滤除距离地图中心两侧太远以及太高的点
    ///将地图体素信息提取到一个ｖｅｃｔｏｒ容器中
    ///
    /*  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
        ProcessVoxelData(high_resolution_hybrid_grid_,&min_index,
        &max_index,current_vehicle_position_vframe_index,
        current_vehicle_position_vframe,activemap_constant_);

    const int height = max_index.y() - min_index.y() + 1;///准可通行区域尺寸
    const int width = max_index.x() - min_index.x() + 1;///准可通行区域尺寸
    ///
    ///提取三维地图图二维信息
    ///
    const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
        width, height, min_index, max_index, voxel_indices_and_probabilities,activemap_constant_);*/
        ///each pixelData include the min_z/max_z/probability_sum/max_probability and the global location
        //   IplImage * TwiDmapImage_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
        //   cvZero(TwiDmapImage_);
        ///
        ///提取准可通行区域正障碍
        /// 提取准可通行区域中的可通行区域
        ///提取准可通行区域悬空障碍
        ///
        //   ProcessPixelValues(accumulated_pixel_data,resolution,TwiDmapImage_,
        //     current_vehicle_position_vframe_index,activemap_constant_);

        //   int extension_index = activemap_constant_.extension_index;
        //   int extended_width = TwiDmapImage_->width + 2*extension_index;
        //   int extended_height = TwiDmapImage_->height+ 2*extension_index;
        //   int pose_index_x = -(min_index.x()-extension_index);
        //   int pose_index_y = -(min_index.y()-extension_index);

    activemap_constant_.traversable_area_msg.header.frame_id = "global_earth_frame";                            ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.header.stamp = ::ivcommon::ToRos(time_);                               ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.index = activemap_constant_.priormapindex;                         ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.prirormapmode = activemap_constant_.priormapmode;                  ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.resolution = resolution;                                           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.height = 0 /*extended_height*/;                                    ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.width = 0 /*extended_width*/;                                      ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose_image_index_x = 0 /*pose_index_x*/;               ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose_image_index_y = 0 /*pose_index_y*/;               ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.submap_finished_flag = activemap_constant_.finished;               ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.use_location_module = activemap_constant_.use_gps_location_module; ///发送准可通行区域结果
    double tem_submap_latitude(0), tem_submap_longitude(0), tem_location_submap_latitude(0), tem_location_submap_longitude(0);
    ::ivcommon::transform::Rigid3d tem_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();
    ::ivcommon::transform::Rigid3d tem_location_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();
    activemap_constant_.traversable_area_msg.use_location_module = false;
    //     tem_pendingpub_submappose = activemap_constant_.origin_position_pose * this->local_pose();
    if (activemap_constant_.use_gps_location_module) {
      activemap_constant_.traversable_area_msg.use_location_module = true;
      //     tem_location_pendingpub_submappose = this->gps_local_pose;
    }
    //   ::ivcommon::transform:: grid_to_geographic( a, e2,zone, hemi, (tem_pendingpub_submappose.translation().y()), (tem_pendingpub_submappose.translation().x()+500000),
    // 		       &tem_submap_latitude, &tem_submap_longitude);
    activemap_constant_.traversable_area_msg.triD_submap_pose.position.x = tem_submap_longitude * 180 / M_PI;                                    ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.position.y = tem_submap_latitude * 180 / M_PI;                                     ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.position.z = tem_pendingpub_submappose.translation().z();                          ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.w = tem_pendingpub_submappose.rotation().w();                          ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.x = tem_pendingpub_submappose.rotation().x();                          ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.y = tem_pendingpub_submappose.rotation().y();                          ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.z = tem_pendingpub_submappose.rotation().z();                          ///发送准可通行区域结果
    // ::ivcommon::transform:: grid_to_geographic( a, e2,zone, hemi, (tem_location_pendingpub_submappose.translation().y()), (tem_location_pendingpub_submappose.translation().x()+500000),
    // 		       &tem_location_submap_latitude, &tem_location_submap_longitude);
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.x = tem_location_submap_longitude * 180 / M_PI;           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.y = tem_location_submap_latitude * 180 / M_PI;            ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.z = tem_location_pendingpub_submappose.translation().z(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.w = tem_location_pendingpub_submappose.rotation().w(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.x = tem_location_pendingpub_submappose.rotation().x(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.y = tem_location_pendingpub_submappose.rotation().y(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.z = tem_location_pendingpub_submappose.rotation().z(); ///发送准可通行区域结果
    // unsigned char *pixeldata = (unsigned char *)TwiDmapImage_->imageData;

    activemap_constant_.traversable_area_msg.cells.clear();
    //   for(int i = 0; i<TwiDmapImage_->height;i++){
    //     for(int j = 0 ;j<activemap_constant_.traversable_area_msg.width;j++){
    //       if(j<extension_index||j>=TwiDmapImage_->width+extension_index){
    // 	activemap_constant_.traversable_area_msg.cells.push_back(0);
    // 	continue;
    //       }
    //       pixeldata =(unsigned char *)( TwiDmapImage_->imageData + (TwiDmapImage_->height-1-i)*TwiDmapImage_->widthStep +j-extension_index);
    //       if(*pixeldata >=250){
    //   activemap_constant_.traversable_area_msg.cells.push_back(2);
    //
    //     }else if(*pixeldata ==125){
    //       activemap_constant_.traversable_area_msg.cells.push_back(12);
    //
    //     }else if(*pixeldata !=0){
    //       activemap_constant_.traversable_area_msg.cells.push_back(1);}else{
    // 	activemap_constant_.traversable_area_msg.cells.push_back(0);}
    //   }
    //   }

    /*   extension 20m from the behind of the vehicle*/
    {

      /*    for(int i = 0; i<extension_index;i++){
      for(int j = 0 ;j<activemap_constant_.traversable_area_msg.width;j++){
        activemap_constant_.traversable_area_msg.cells.insert(
          activemap_constant_.traversable_area_msg.cells.begin()+
          i*activemap_constant_.traversable_area_msg.width+j,0);
      }
      }
      for(int i = 0; i<extension_index;i++){
      for(int j = 0 ;j<activemap_constant_.traversable_area_msg.width;j++){
        activemap_constant_.traversable_area_msg.cells.push_back(0);
      }
      }
      */
    }

    vehicle_global_pose_.header.stamp = activemap_constant_.traversable_area_msg.header.stamp;
    vehicle_global_pose_.header.frame_id = "global_earth_frame";
    double tem_vehicle_latitude, tem_vehicle_longitude,
      tem_location_module_vehicle_longitude, tem_location_module_vehicle_latitude;
    ::ivcommon::transform::Rigid3d tem_pendingpub_vehiclepose = ::ivcommon::transform::Rigid3d::Identity();
    ::ivcommon::transform::Rigid3d tem_pendingpub_location_modulevehiclepose = ::ivcommon::transform::Rigid3d::Identity();
    if (activemap_constant_.use_gps_location_module) {
      tem_pendingpub_location_modulevehiclepose = this->gps_local_pose *
        (activemap_constant_.origin_position_pose * this->local_pose()).inverse() *
        activemap_constant_.origin_position_pose * range_pose_observation_;
    }
    tem_pendingpub_vehiclepose = activemap_constant_.origin_position_pose * range_pose_observation_;
    ::ivcommon::transform::grid_to_geographic(a, e2, zone, hemi,
      (tem_pendingpub_vehiclepose.translation().y()),
      (tem_pendingpub_vehiclepose.translation().x() + 500000),
      &tem_vehicle_latitude, &tem_vehicle_longitude);

    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.x = tem_vehicle_longitude * 180 / M_PI;
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.y = tem_vehicle_latitude * 180 / M_PI;
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.z = tem_pendingpub_vehiclepose.translation().z();
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.w = tem_pendingpub_vehiclepose.rotation().w();
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.x = tem_pendingpub_vehiclepose.rotation().x();
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.y = tem_pendingpub_vehiclepose.rotation().y();
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.z = tem_pendingpub_vehiclepose.rotation().z();

    ::ivcommon::transform::grid_to_geographic(a, e2, zone, hemi, (tem_pendingpub_location_modulevehiclepose.translation().y()),
      (tem_pendingpub_location_modulevehiclepose.translation().x() + 500000),
      &tem_location_module_vehicle_latitude, &tem_location_module_vehicle_longitude);

    vehicle_global_pose_.location_module_pose.pose.pose.position.x = tem_location_module_vehicle_longitude * 180 / M_PI;
    vehicle_global_pose_.location_module_pose.pose.pose.position.y = tem_location_module_vehicle_latitude * 180 / M_PI;
    vehicle_global_pose_.location_module_pose.pose.pose.position.z = tem_pendingpub_location_modulevehiclepose.translation().z();
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.w = tem_pendingpub_location_modulevehiclepose.rotation().w();
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.x = tem_pendingpub_location_modulevehiclepose.rotation().x();
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.y = tem_pendingpub_location_modulevehiclepose.rotation().y();
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.z = tem_pendingpub_location_modulevehiclepose.rotation().z();

    activemap_constant_.vehicle_global_pose_pub.publish(vehicle_global_pose_);
    //    LOG(INFO)<<"priormap mode vehicle_global_pose_"<<vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose;
    activemap_constant_.submap_traversable_area_pub.publish(activemap_constant_.traversable_area_msg);
    LOG(INFO) << "mapping priormapmode activemap_constant_.traversable_area_msg.index:" <<
      activemap_constant_.traversable_area_msg.index << "online submap:" <<
      identifier_index << " time elapsed:" <<
      (ros::Time::now() - activemap_constant_.traversable_area_msg.header.stamp).toSec();
    //    if(activemap_constant_.traversable_area_display){
    //           IplImage* Display_Image = cvCreateImage(cvSize(extended_width, extended_height),IPL_DEPTH_8U,3);
    //    cvZero(Display_Image);
    //    unsigned char* pdata = (unsigned char*)TwiDmapImage_->imageData ;
    //    unsigned char* display_data = (unsigned char*)Display_Image->imageData;
    //
    //    for(int i = 0 ;i<Display_Image->height;i++){
    //      for(int j = 0 ;j<Display_Image->width;j++){
    //        if(i>=TwiDmapImage_->height +extension_index){
    // 	 break;
    //       }
    //        if(j<extension_index||j>=TwiDmapImage_->width+extension_index||i<extension_index){
    // 	 continue;
    //       }
    //         pdata  = (unsigned char*)TwiDmapImage_->imageData +(i-extension_index)*TwiDmapImage_->widthStep+j-extension_index;
    // 	if(*pdata == 125){
    // 	  display_data = (unsigned char*)Display_Image->imageData+(i)*Display_Image->widthStep+j*3;
    // 	 display_data[0]= 255;
    // 	 display_data[1]= 255;
    // 	 display_data[2]= 255;
    // 	 continue;
    // 	}
    //        if(*pdata >=250){
    // 	 display_data = (unsigned char*)Display_Image->imageData+(i)*Display_Image->widthStep+j*3;
    // 	 display_data[0]= 0;
    // 	 display_data[1]= 0;
    // 	 display_data[2]= 255;
    //       }else if(*pdata !=0){
    // 	display_data = (unsigned char*)Display_Image->imageData+(i)*Display_Image->widthStep+j*3;
    // 	 display_data[0]= 0;
    // 	 display_data[1]= 255;
    // 	 display_data[2]= 0;
    //       }
    //     }
    //   }
    //     int heightnum = Display_Image->height/(10/resolution);
    //     int widthnum = Display_Image->width/(10/resolution);
    //     for(int i = 0; i<heightnum ;i++)
    //       {
    //         cvLine(Display_Image,cvPoint(0,Display_Image->height*i/heightnum),
    //                 cvPoint(Display_Image->width-1,Display_Image->height*i/heightnum),cvScalar(255,0,0));
    //       }
    //
    //     for(int i=1;i<widthnum;i++)
    //     {
    //         cvLine(Display_Image,cvPoint(Display_Image->width*i/widthnum,0),
    //                 cvPoint(Display_Image->width*i/widthnum,Display_Image->height-1),cvScalar(255,0,0));
    //     }
    //
    //      float ogmresolution = resolution;
    //     cvCircle(Display_Image,cvPoint(current_vehicle_position_vframe_index.x()+pose_index_x,
    //  				 Display_Image->height-1-(current_vehicle_position_vframe_index.y()+pose_index_y)),5,
    // 	     cvScalar(0,255,255),-1);
    //       cvShowImage("twiD submap",Display_Image);
    //       cvWaitKey(1);
    //         cvReleaseImage(&Display_Image);
    //      Display_Image = nullptr;
    //     }
    //      cvReleaseImage(&TwiDmapImage_);
    //      TwiDmapImage_ = nullptr;

    twid_display_thread_created = false;
    // std::string tem_filename = "/home/zack-liu/test/time_test";
    // std::fstream tem_filestream;
    // tem_filestream.open(tem_filename, std::ios::out | std::ios::app);
    // ::ros::Time s_time = ::ivcommon::ToRos(time_);
    // double time_used = (ros::Time::now() - s_time).toSec() / 3;

    // tem_filestream << std::fixed << std::setprecision(10) << '\t' << time_used << '\n';
    // tem_filestream.close();
    return;
  }

  if (/*activemap_constant_.submap_traversable_area_pub.getNumSubscribers()>0*/
    !activemap_constant_.priormapmode) {
    const float resolution = high_resolution_hybrid_grid_.resolution();
    Eigen::Array2i min_index(INT_MAX, INT_MAX);
    Eigen::Array2i max_index(INT_MIN, INT_MIN);
    Eigen::Array2i tem_submap_pose_index(INT_MIN, INT_MIN);
    const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ProcessVoxelData(high_resolution_hybrid_grid_,
        &min_index,
        &max_index, current_vehicle_position_vframe_index, current_vehicle_position_vframe, activemap_constant_);
    const int height = max_index.y() - min_index.y() + 1;
    const int width = max_index.x() - min_index.x() + 1;

    //each pixelData include the min_z/max_z/probability_sum/max_probability and the global location
    const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities, activemap_constant_);
    // IplImage *TwiDmapImage_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    // cvZero(TwiDmapImage_);
    // printf("Assign zero mat to TwiDmapImage_(%d,%d)\n", width, height);
    cv::Mat TwiDmapImage_ = cv::Mat::zeros(cv::Size(width, height), CV_8U);
    // if (TwiDmapImage_.empty()) {
    //   printf("TwiDmapImage_ empty!\n");
    //   return;
    // }
    ProcessPixelValues(accumulated_pixel_data, resolution, TwiDmapImage_, current_vehicle_position_vframe_index, activemap_constant_);

    int extension_index = activemap_constant_.extension_index;
    // int extended_width = TwiDmapImage_->width + 2 * extension_index;
    // int extended_height = TwiDmapImage_->height + 2 * extension_index;
    int extended_width = width + 2 * extension_index;
    int extended_height = height + 2 * extension_index;
    int pose_index_x = -(min_index.x() - extension_index);
    int pose_index_y = -(min_index.y() - extension_index);
    activemap_constant_.traversable_area_msg.header.frame_id = "global_earth_frame";           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.header.stamp = ::ivcommon::ToRos(time_);              ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.height = extended_height;                         ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.width = extended_width;                           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.index = identifier_index;                         ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.prirormapmode = activemap_constant_.priormapmode; ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.resolution = resolution;                          ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose_image_index_x = pose_index_x;    ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose_image_index_y = pose_index_y;    ///发送准可通行区域结果
    double tem_submap_latitude, tem_submap_longitude, tem_location_submap_latitude, tem_location_submap_longitude;
    ::ivcommon::transform::Rigid3d tem_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();
    ::ivcommon::transform::Rigid3d tem_location_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();

    activemap_constant_.traversable_area_msg.use_location_module = false;
    tem_pendingpub_submappose = activemap_constant_.origin_position_pose * this->local_pose();
    if (activemap_constant_.use_gps_location_module) {

      activemap_constant_.traversable_area_msg.use_location_module = true;
      tem_location_pendingpub_submappose = this->gps_local_pose;
    }

    ::ivcommon::transform::grid_to_geographic(
      a, e2, zone, hemi, (tem_pendingpub_submappose.translation().y()),
      (tem_pendingpub_submappose.translation().x() + 500000),
      &tem_submap_latitude, &tem_submap_longitude);

    activemap_constant_.traversable_area_msg.triD_submap_pose.position.x = tem_submap_longitude * 180 / M_PI;           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.position.y = tem_submap_latitude * 180 / M_PI;            ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.position.z = tem_pendingpub_submappose.translation().z(); ///发送准可通行区域结果

    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.w = tem_pendingpub_submappose.rotation().w(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.x = tem_pendingpub_submappose.rotation().x(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.y = tem_pendingpub_submappose.rotation().y(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.triD_submap_pose.orientation.z = tem_pendingpub_submappose.rotation().z(); ///发送准可通行区域结果
    ::ivcommon::transform::grid_to_geographic(
      a, e2, zone, hemi, (tem_location_pendingpub_submappose.translation().y()),
      (tem_location_pendingpub_submappose.translation().x() + 500000),
      &tem_location_submap_latitude, &tem_location_submap_longitude);
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.x = tem_location_submap_longitude * 180 / M_PI;           ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.y = tem_location_submap_latitude * 180 / M_PI;            ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.position.z = tem_location_pendingpub_submappose.translation().z(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.w = tem_location_pendingpub_submappose.rotation().w(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.x = tem_location_pendingpub_submappose.rotation().x(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.y = tem_location_pendingpub_submappose.rotation().y(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.location_module_triD_submap_pose.orientation.z = tem_location_pendingpub_submappose.rotation().z(); ///发送准可通行区域结果
    activemap_constant_.traversable_area_msg.submap_finished_flag = activemap_constant_.finished;                                                ///发送准可通行区域结果

    // unsigned char *pixeldata = (unsigned char *)TwiDmapImage_->imageData;
    unsigned char* pixeldata;
    activemap_constant_.traversable_area_msg.cells.clear();
    for (int i = 0; i < TwiDmapImage_.rows; i++) {
      for (int j = 0; j < activemap_constant_.traversable_area_msg.width; j++) {
        if (j < extension_index || j >= TwiDmapImage_.cols + extension_index) {
          activemap_constant_.traversable_area_msg.cells.push_back(0); ///发送准可通行区域结果
          continue;
        }
        // pixeldata = (unsigned char *)(TwiDmapImage_->imageData + 
        //   (TwiDmapImage_->height - 1 - i) * TwiDmapImage_->widthStep + j - extension_index);
        pixeldata = TwiDmapImage_.ptr<unsigned char>(TwiDmapImage_.rows - 1 - i) + j - extension_index;
        if (*pixeldata >= 250) {
          activemap_constant_.traversable_area_msg.cells.push_back(2); ///发送准可通行区域结果
        }
        else if (*pixeldata == 125) {
          activemap_constant_.traversable_area_msg.cells.push_back(12); ///发送准可通行区域结果
        }
        else if (*pixeldata != 0) {
          activemap_constant_.traversable_area_msg.cells.push_back(1);
        }
        else {
          activemap_constant_.traversable_area_msg.cells.push_back(0);
        }
      }
    }

    /*   extension 20m from the behind of the vehicle*/
    {
      for (int i = 0; i < extension_index; i++) {
        for (int j = 0; j < activemap_constant_.traversable_area_msg.width; j++) {
          ///发送准可通行区域结果
          activemap_constant_.traversable_area_msg.cells.insert(
            activemap_constant_.traversable_area_msg.cells.begin() +
            i * activemap_constant_.traversable_area_msg.width + j, 0);
        }
      }
      for (int i = 0; i < extension_index; i++) {
        for (int j = 0; j < activemap_constant_.traversable_area_msg.width; j++) {
          activemap_constant_.traversable_area_msg.cells.push_back(0); ///发送准可通行区域结果
        }
      }
    }
    vehicle_global_pose_.header.stamp = activemap_constant_.traversable_area_msg.header.stamp;
    vehicle_global_pose_.header.frame_id = "global_earth_frame";
    double tem_vehicle_latitude, tem_vehicle_longitude, tem_location_module_vehicle_longitude, tem_location_module_vehicle_latitude;
    ::ivcommon::transform::Rigid3d tem_pendingpub_vehiclepose = ::ivcommon::transform::Rigid3d::Identity();
    ::ivcommon::transform::Rigid3d tem_pendingpub_location_modulevehiclepose = ::ivcommon::transform::Rigid3d::Identity();
    if (activemap_constant_.use_gps_location_module) {
      tem_pendingpub_location_modulevehiclepose = this->gps_local_pose *
        (activemap_constant_.origin_position_pose * this->local_pose()).inverse() *
        activemap_constant_.origin_position_pose * range_pose_observation_;
    }
    tem_pendingpub_vehiclepose = activemap_constant_.origin_position_pose * range_pose_observation_;
    ::ivcommon::transform::grid_to_geographic(a, e2, zone, hemi,
      (tem_pendingpub_vehiclepose.translation().y()),
      (tem_pendingpub_vehiclepose.translation().x() + 500000),
      &tem_vehicle_latitude, &tem_vehicle_longitude);
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.x = tem_vehicle_longitude * 180 / M_PI;           ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.y = tem_vehicle_latitude * 180 / M_PI;            ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.position.z = tem_pendingpub_vehiclepose.translation().z(); ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.w = tem_pendingpub_vehiclepose.rotation().w(); ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.x = tem_pendingpub_vehiclepose.rotation().x(); ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.y = tem_pendingpub_vehiclepose.rotation().y(); ///发送此时车辆位姿结果
    vehicle_global_pose_.primary_submap_vehicle_pose.pose.pose.orientation.z = tem_pendingpub_vehiclepose.rotation().z(); ///发送此时车辆位姿结果
    ::ivcommon::transform::grid_to_geographic(a, e2, zone, hemi, (tem_pendingpub_location_modulevehiclepose.translation().y()),
      (tem_pendingpub_location_modulevehiclepose.translation().x() + 500000),
      &tem_location_module_vehicle_latitude, &tem_location_module_vehicle_longitude);
    vehicle_global_pose_.location_module_pose.pose.pose.position.x = tem_location_module_vehicle_longitude * 180 / M_PI;          ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.position.y = tem_location_module_vehicle_latitude * 180 / M_PI;           ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.position.z = tem_pendingpub_location_modulevehiclepose.translation().z(); ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.w = tem_pendingpub_location_modulevehiclepose.rotation().w(); ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.x = tem_pendingpub_location_modulevehiclepose.rotation().x(); ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.y = tem_pendingpub_location_modulevehiclepose.rotation().y(); ///发送此时车辆位姿结果
    vehicle_global_pose_.location_module_pose.pose.pose.orientation.z = tem_pendingpub_location_modulevehiclepose.rotation().z(); ///发送此时车辆位姿结果
    activemap_constant_.vehicle_global_pose_pub.publish(vehicle_global_pose_);
    activemap_constant_.submap_traversable_area_pub.publish(activemap_constant_.traversable_area_msg);
    ///
    ///显示
    ///
    if (activemap_constant_.traversable_area_display) {
      // IplImage *Display_Image = cvCreateImage(cvSize(extended_width, extended_height), IPL_DEPTH_8U, 3);
      // cvZero(Display_Image);
      Display_Image = cv::Mat::zeros(cv::Size(extended_width, extended_height), CV_8UC3);
      // unsigned char *pdata = (unsigned char *)TwiDmapImage_->imageData;
      // unsigned char *display_data = (unsigned char *)Display_Image->imageData;
      unsigned char* pdata;
      unsigned char* display_data;

      for (int i = 0; i < Display_Image.rows; i++) {
        for (int j = 0; j < Display_Image.cols; j++) {
          if (i >= TwiDmapImage_.rows + extension_index) {
            break;
          }
          if (j < extension_index || j >= TwiDmapImage_.cols + extension_index || i < extension_index) {
            continue;
          }
          // pdata = (unsigned char *)TwiDmapImage_->imageData + 
          //   (i - extension_index) * TwiDmapImage_->widthStep + j - extension_index;
          pdata = TwiDmapImage_.ptr<unsigned char>(i - extension_index) + j - extension_index;
          display_data = Display_Image.ptr<unsigned char>(i);
          if (*pdata == 125) {
            // display_data = (unsigned char *)Display_Image->imageData + (i)*Display_Image->widthStep + j * 3;
            // display_data[0] = 255;
            // display_data[1] = 255;
            // display_data[2] = 255;
            display_data[3 * j] = 255;
            display_data[3 * j + 1] = 255;
            display_data[3 * j + 2] = 255;
            continue;
          }
          if (*pdata >= 250) {
            // display_data = (unsigned char *)Display_Image->imageData + (i)*Display_Image->widthStep + j * 3;
            // display_data[0] = 0;
            // display_data[1] = 0;
            // display_data[2] = 255;
            display_data[3 * j] = 0;
            display_data[3 * j + 1] = 0;
            display_data[3 * j + 2] = 255;
          }
          else if (*pdata != 0) {
            // display_data = (unsigned char *)Display_Image->imageData + (i)*Display_Image->widthStep + j * 3;
            // display_data[0] = 0;
            // display_data[1] = 255;
            // display_data[2] = 0;
            display_data[3 * j] = 0;
            display_data[3 * j + 1] = 255;
            display_data[3 * j + 2] = 0;
          }
        }
      }
      // int heightnum = Display_Image->height / (10 / resolution);
      // int widthnum = Display_Image->width / (10 / resolution);
      int heightnum = Display_Image.rows / (10 / resolution);
      int widthnum = Display_Image.cols / (10 / resolution);
      for (int i = 0; i < heightnum; i++) {
        // cvLine(Display_Image, cvPoint(0, Display_Image->height * i / heightnum),
        //        cvPoint(Display_Image->width - 1, Display_Image->height * i / heightnum), cvScalar(255, 0, 0));
        cv::line(Display_Image, cv::Point(0, Display_Image.rows * i / heightnum),
          cv::Point(Display_Image.cols - 1, Display_Image.rows * i / heightnum), cv::Scalar(255, 0, 0));
      }
      for (int i = 1; i < widthnum; i++) {
        // cvLine(Display_Image, cvPoint(Display_Image->width * i / widthnum, 0),
        //        cvPoint(Display_Image->width * i / widthnum, Display_Image->height - 1), cvScalar(255, 0, 0));
        cv::line(Display_Image, cv::Point(Display_Image.cols * i / widthnum, 0),
          cv::Point(Display_Image.cols * i / widthnum, Display_Image.rows - 1), cv::Scalar(255, 0, 0));
      }
      float ogmresolution = resolution;
      // cvCircle(Display_Image, cvPoint(current_vehicle_position_vframe_index.x() + 
      //   pose_index_x, Display_Image->height - 1 - 
      //   (current_vehicle_position_vframe_index.y() + pose_index_y)), 5,
      //   cvScalar(0, 255, 255), -1);
      cv::circle(Display_Image, cv::Point(current_vehicle_position_vframe_index.x() +
        pose_index_x, Display_Image.rows - 1 -
        (current_vehicle_position_vframe_index.y() + pose_index_y)), 5,
        cv::Scalar(0, 255, 255), -1);
      // cvShowImage("twiD submap", Display_Image);
      // cvWaitKey(1);
      // cv::imshow("twiD submap", Display_Image);
      // cv::waitKey(1);
      // cvReleaseImage(&Display_Image);
      // Display_Image = nullptr;
    }
    // cvReleaseImage(&TwiDmapImage_);
    // TwiDmapImage_ = nullptr;
    twid_display_thread_created = false;
  }
  // std::string tem_filename = "/home/zack-liu/test/time_test";
  // std::fstream tem_filestream;
  // tem_filestream.open(tem_filename, std::ios::out | std::ios::app);
  // ::ros::Time s_time = ::ivcommon::ToRos(time_);
  // double time_used = (ros::Time::now() - s_time).toSec() / 3;

  // tem_filestream << std::fixed << std::setprecision(10) << '\t' << time_used << '\n';
  // tem_filestream.close();
}
///
///保存三维地图
///
void Submap::Trid_Submap_Write(std::string file_time_name_) {
  clock_t trid_write_start = clock();
  trid_write_thread_created = true;
  std::string submap_file_name = "";
  std::string file_modle_name = "tridmap";
  submap_file_name = ::ivcommon::file_directory_generate(file_time_name_, file_modle_name);
  std::stringstream tem_stringstream;
  tem_stringstream.clear();
  tem_stringstream << identifier_index;
  submap_file_name += tem_stringstream.str();
  submap_file_name += ".proto";
  ::ivcommon::io::ProtoStreamWriter writer(submap_file_name);
  proto::Submap3D proto;
  proto.set_trajectory_index(0);
  ToProto(&proto);
  writer.WriteProto(proto);
  trid_write_thread_created = false;
  clock_t trid_write_stop = clock();
  double trid_write_elapsed = (double)(trid_write_stop - trid_write_start) / CLOCKS_PER_SEC;
}
///
///保存可视化的三维地图
///
void Submap::Visualization_Trid_Submap_Write(std::string file_time_name_) {
  visualization_trid_write_thread_created = true;
  constexpr float kXrayObstructedCellProbabilityLimit = 0.805f /*0.501f*/;
  static int Visualization_trid_submap_index = 0;
  octomap::OcTree temOctomap(high_resolution_hybrid_grid_.resolution());
  const iv_slam_mapping::mapping_3d::HybridGrid& tem_hybrid = high_resolution_hybrid_grid_;
  for (auto it = iv_slam_mapping::mapping_3d::HybridGrid::Iterator(tem_hybrid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = iv_slam_mapping::mapping::ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }
    const Eigen::Vector3f cell_center_submap = tem_hybrid.GetCenterOfCell(it.GetCellIndex());
    temOctomap.updateNode(cell_center_submap(0), cell_center_submap(1), cell_center_submap(2), true);
  }
  std::string submap_file_name = "";
  std::string file_modle_name = "visualization_tridmap";
  submap_file_name = ::ivcommon::file_directory_generate(file_time_name_, file_modle_name);
  std::stringstream tem_stringstream;
  tem_stringstream.clear();
  tem_stringstream.str("");
  tem_stringstream << Visualization_trid_submap_index;
  submap_file_name += tem_stringstream.str();
  submap_file_name += ".bt";
  temOctomap.writeBinary(submap_file_name);
  Visualization_trid_submap_index++;
  visualization_trid_write_thread_created = false;
}

///
/// Insert 'range_data' into this submap using 'range_data_inserter'. The
/// submap must not be finished yet.
///进行三维概率栅格地图创建
///
void Submap::InsertRangeData(const sensor::RangeData& range_data,
  const RangeDataInserter& range_data_inserter,
  const int high_resolution_max_range) {
  CHECK(!finished_);
  const sensor::RangeData transformed_range_data =
    sensor::TransformRangeData(range_data, local_pose().inverse().cast<float>());
  range_data_inserter.Insert(FilterRangeDataByMaxRange(transformed_range_data,
    high_resolution_max_range), &high_resolution_hybrid_grid_);
  SetNumRangeData(num_range_data() + 1);
}

void Submap::Finish() {
  CHECK(!finished_);
  finished_ = true;
}

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
ActiveSubmaps::ActiveSubmaps(const proto::SubmapsOptions& options, const ::ros::NodeHandle& activesubmap_nh)
  : options_(options), range_data_inserter_(options.range_data_inserter_options()), activesubmap_nh_(activesubmap_nh) {
  // We always want to have at least one submap which we can return and will
  // create it at the origin in absence of a better choice.
  //
  // TODO(whess): Start with no submaps, so that all of them can be approximately gravity aligned.
  time_t t = time(0);
  char time_char[32] = "";
  strftime(time_char, sizeof(time_char), "%Y%m%d_%H%M%S", localtime(&t));
  file_time_name = time_char;
  activemap_constant.vehicle_global_pose_pub = activesubmap_nh_.advertise<iv_slam_ros_msgs::Traversablevehiclepose>("vehicle_global_pose_topic", 1);
  activemap_constant.submap_traversable_area_pub = activesubmap_nh_.advertise<iv_slam_ros_msgs::PrimarytraversableArea>("traversible_area_topic", 1);
  activemap_constant.kXrayObstructedCellProbabilityLimit = options_.kxrayobstructedcellprobabilitylimit();
  activemap_constant.rough_intensity = options_.rough_intensity();
  activemap_constant.kMinZDifference = options_.kminzdifference();
  activemap_constant.kMinZDifference_beyond = options_.kminzdifference_beyond();
  activemap_constant.ZDifference_change_thresh = options_.zdifference_change_thresh();
  activemap_constant.obstacle_emptythresh = options_.obstacle_emptythresh();
  activemap_constant.extension_index = options_.extension_index();
  activemap_constant.traversable_area_display = options_.twid_submap_display();
  autonomousmapping_index = -1;
  last_mappingmode = -1;
  inserted_rangedata_index = 0;
  last_invoked_mapindex = -1;
  // Display_Image_ = nullptr;
  activemap_constant.origin_position_pose = ::ivcommon::transform::Rigid3d::Identity(); ///参数初始化
  // ROS_INFO("ActiveSubmaps: show traversable area: %d", activemap_constant.traversable_area_display);
  if (activemap_constant.traversable_area_display) {
    keep_visualize_thread_running = true;
    visualize_thread = std::make_shared<boost::thread>(boost::thread(boost::bind(&ActiveSubmaps::visualizeSubmap,this)));
  }
  else {
    keep_visualize_thread_running = false;
    visualize_thread = nullptr;
  }
}
///
///析构函数
///
ActiveSubmaps::~ActiveSubmaps() {
  keep_visualize_thread_running = false;
  if (visualize_thread != nullptr && visualize_thread->joinable())
    visualize_thread->join();
  LOG(INFO) << "ActiveSubmaps destructor ran!";
}

void ActiveSubmaps::visualizeSubmap () {
  while (keep_visualize_thread_running) {
    // printf("\rActiveSubmaps::visualizeSubmap running...");
    // fflush(stdout);
    if (!submaps_.empty() && !submaps_[0]->Display_Image.empty()) {
      cv::imshow("twiD submap", submaps_[0]->Display_Image);
      cv::waitKey(5);
    }
    usleep(100000);
  }
}

///
///获得当前正在维护的地图
///
std::vector<std::shared_ptr<Submap>> ActiveSubmaps::submaps() const {
  return submaps_;
}

int ActiveSubmaps::matching_index() const { return matching_submap_index_; }
///
/// Inserts 'range_data' into the Submap collection.
///
void ActiveSubmaps::InsertRangeData(const ::ivcommon::Time& time_, const sensor::RangeData& range_data,
  const ivcommon::transform::Rigid3d& activesubmap_pose_observation,
  const std::vector<int> current_mapping_indexs_, const int& mode) {

  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_, options_.high_resolution_max_range());
  } ///封信地图数据
  if (options_.enable_traversablearea_extraction()) {
    if (!(submaps_.front()->twid_display_thread_created)) {
      if (mode == 2) {
        if (last_mappingmode != mode && last_mappingmode != -1) {
          ///
          ///如果是从在线模式切换为先验地图模式
          ///先验地图编号直接从里程计信息获得
          ///同时将在线地图编号更改为自主建图编号
          ///
          activemap_constant.priormapmode = true;                             ///将服务于可通行区域提取模块
          activemap_constant.priormapindex = current_mapping_indexs_.front(); ///先验地图编号直接从里程计信息获得
          activemap_constant.finished = true;
          for (int i = 0; i < submaps_.size(); i++) {
            submaps_[i]->identifier_index = autonomousmapping_index; ///将在线地图编号更改为自主建图编号
            autonomousmapping_index--;
          }
        }
        else {
          activemap_constant.priormapmode = true;
          activemap_constant.priormapindex = current_mapping_indexs_.front(); ///将服务于可通行区域提取模块
          activemap_constant.finished = false;
        }
        submaps_.front()->current_vehicle_position_vframe =
          submaps_.front()->local_pose().inverse().cast<float>() * range_data.origin;
        bool tem_twid_map_write_flag = false /*options_.twid_submap_write()*/;
        ///
        ///准可通行区域提取
        ///
        // submaps_.front()->Traversablearea_Extraction(time_,
        //   activemap_constant, activesubmap_pose_observation,
        //   &twid_map_data, file_time_name, tem_twid_map_write_flag);
        boost::thread traversablearea_extraction_thread(boost::bind(
             &Submap::Traversablearea_Extraction,submaps_.front(),time_,
        	    activemap_constant,activesubmap_pose_observation,
        	    &twid_map_data,file_time_name,tem_twid_map_write_flag));
        // traversablearea_extraction_thread.join();///释放线程资源
        traversablearea_extraction_thread.detach();///释放线程资源
      }
      else {
        activemap_constant.priormapmode = false; ///将服务于可通行区域提取模块
        if (last_mappingmode == 2) {
          ///
          ///如果是从先验地图模式切换为在线建图模式
          ///将自主建图时的地图丢给在线建图
          ///同时将在线建图编号更改为里程计编号
          ///
          activemap_constant.finished = false;
          if (submaps_.size() > 1) {
            submaps_.erase(submaps_.end());
            autonomousmapping_index++;
            if (autonomousmapping_index >= -1) {
              autonomousmapping_index = -1;
            }
          }
          submaps_.front()->identifier_index = current_mapping_indexs_[0];
        }
        else {
          if (mode == 1) {
            activemap_constant.finished = true;
          }
          else {
            activemap_constant.finished = false;
          }
        }
        submaps_.front()->identifier_index = current_mapping_indexs_[0];
        submaps_.front()->current_vehicle_position_vframe = submaps_.front()->local_pose().inverse().cast<float>() * range_data.origin;
        bool tem_twid_map_write_flag = false;
        ///
        ///准可通行区域提取
        ///
        // submaps_.front()->Traversablearea_Extraction(time_,
        //   activemap_constant, activesubmap_pose_observation,
        //   &twid_map_data, file_time_name, tem_twid_map_write_flag);
        boost::thread traversablearea_extraction_thread(boost::bind(
          &Submap::Traversablearea_Extraction, submaps_.front(), time_,
          activemap_constant, activesubmap_pose_observation, &twid_map_data, 
          file_time_name, tem_twid_map_write_flag));
        // traversablearea_extraction_thread.join();
        traversablearea_extraction_thread.detach(); ///释放线程资源
      }
    }
  }
  if (mode != 2) {
    if ((submaps_.front()->identifier_index != current_mapping_indexs_[0] ||
      submaps_.back()->identifier_index != current_mapping_indexs_[1]) &&
      current_mapping_indexs_.size() > 1) {
      AddSubmap(activesubmap_pose_observation, current_mapping_indexs_[1]);
      if (activemap_constant.use_gps_location_module) {
        if (activemap_constant.location_module_data.size() > 0) {
          submaps_.back()->gps_local_pose = activemap_constant.location_module_data.back().pose;
          if (std::fabs<double>((::ivcommon::ToRos(time_) -
            ::ivcommon::ToRos(activemap_constant.location_module_data.back().time)).toSec()) > 0.3) {
            LOG(ERROR) << "Location_module_data time is too far from rangedata time!";
          }
        }
        else {
          LOG(WARNING) << "Location_module_data is null!";
        }
      }
    }
    if (mode == 1) {
      AddSubmap(activesubmap_pose_observation, current_mapping_indexs_[2]);
      if (activemap_constant.use_gps_location_module) {
        if (activemap_constant.location_module_data.size() > 0) {
          submaps_.back()->gps_local_pose = activemap_constant.location_module_data.back().pose;
          if (std::fabs<double>((::ivcommon::ToRos(time_) -
            ::ivcommon::ToRos(activemap_constant.location_module_data.back().time)).toSec()) > 0.3) {
            LOG(ERROR) << "Location_module_data time is too far from rangedata time!";
          }
        }
        else {
          LOG(WARNING) << "Location_module_data is null!";
        }
      }
    }
  }
  else if (submaps_.back()->num_range_data() >= options_.num_range_data()) {
    AddSubmap(activesubmap_pose_observation, autonomousmapping_index);
    if (activemap_constant.use_gps_location_module) {
      if (activemap_constant.location_module_data.size() > 0) {
        submaps_.back()->gps_local_pose = activemap_constant.location_module_data.back().pose;
        if (std::fabs<double>((::ivcommon::ToRos(time_) -
          ::ivcommon::ToRos(activemap_constant.location_module_data.back().time)).toSec()) > 0.3) {
          LOG(ERROR) << "Location_module_data time is too far from rangedata time!";
        }
      }
      else {
        LOG(WARNING) << "Location_module_data is null!";
      }
    }
    autonomousmapping_index--;
  }
  last_mappingmode = mode;
}
///
///添加地图，同时删除上一个地图
///
void ActiveSubmaps::AddSubmap(const ::ivcommon::transform::Rigid3d& local_pose, const int& submap_index) {
  if (submaps_.size() > 1) {
    submaps_.front()->Finish(); ///标志为完成并删除上一个地图
    ++matching_submap_index_;
    if (options_.trid_submap_write()) {
      if (!(submaps_.front()->trid_write_thread_created)) {
        boost::thread trid_thread(boost::bind(&Submap::Trid_Submap_Write, submaps_.front(), file_time_name)); ///保存三维地图
        trid_thread.detach();                                                                                 ///释放线程资源
      }
    }
    if (options_.visualization_trid_submap_write()) {
      if (!(submaps_.front()->visualization_trid_write_thread_created)) {
        boost::thread visualization_trid_thread(boost::bind(
          &Submap::Visualization_Trid_Submap_Write, submaps_.front(), file_time_name)); ///保存可视化的三维地图
        visualization_trid_thread.detach();  ///释放线程资源
      }
    }
    submaps_.erase(submaps_.begin());
  }
  submaps_.emplace_back(new Submap(options_.high_resolution(), options_.low_resolution(), local_pose)); ///添加新地图
  submaps_.back()->identifier_index = submap_index;                                                     ///赋值地图编号
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

} // namespace mapping_3d
} // namespace iv_slam_mapping

/*!
* \file submaps.cc
* \brief 子地图相关程序
*
* 子地图类，活动子地图类，主要负责点云的插入，活动子地图的管理
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/

#include "covgrid_slam/mapping3d/submaps.h"
#include "covgrid_slam/mapping/submaps.h"
#include <cmath>
#include <limits>

#include "ivcommon/common/math.h"
#include "glog/logging.h"
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "covgrid_slam/sensor/range_data.h"
#include <pcl/io/pcd_io.h>

namespace mapping3d {

namespace {

struct PixelData {
  int min_z = INT_MAX;
  int max_z = INT_MIN;
  int count = 0;
  float probability_sum = 0.f;
  float max_probability = 0.5f;
};

// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses and reflectivity
// information.
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
                                            const float max_range) {
  sensor::RangeData result{range_data.origin, {}, {}};
  for (const sensor::Point& hit : range_data.returns) {
    if ((hit - range_data.origin).norm() <= max_range) {
      result.returns.push_back(hit);
    }
  }
  return result;
}

std::vector<Eigen::Array4i> ProcessVoxelData(
    const HybridGrid& hybrid_grid,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();
  int total_submap_voxel_num=0;
  int hit_submap_voxel_num=0;
  int miss_submap_voxel_num = 0;

  constexpr float kXrayObstructedCellProbabilityLimit = 0.551f/*0.550000999999f*/;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    total_submap_voxel_num++;
    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);

    if (probability < kXrayObstructedCellProbabilityLimit) {
      if(probability <0.5){
	miss_submap_voxel_num++;
      }
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3d cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Array4i voxel_index_and_probability(
        ::ivcommon::RoundToInt(cell_center_submap.x() * resolution_inverse),
        ::ivcommon::RoundToInt(cell_center_submap.y() * resolution_inverse),
        ::ivcommon::RoundToInt(cell_center_submap.z() * resolution_inverse),
        probability_value);
//LOG(INFO)<<voxel_index_and_probability(0,0)<<'\t'<<voxel_index_and_probability(0,1);
    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
    hit_submap_voxel_num++;
  }
//  LOG(INFO)<<"total_submap_voxel_num:"<<total_submap_voxel_num<<'\t'<<"hit_submap_voxel_num:"<<hit_submap_voxel_num<<'\t'<<"miss_submap_voxel_num:"<<miss_submap_voxel_num;
  return voxel_indices_and_probabilities;
}

std::vector<Eigen::Array4i> ProcessVoxelDataNear(
    const HybridGrid& hybrid_grid,
    const Eigen::Array2i min_index,const Eigen::Array2i max_index,ivcommon::transform::Rigid3d pose) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();
  int total_submap_voxel_num=0;
  int hit_submap_voxel_num=0;
  int miss_submap_voxel_num = 0;

  constexpr float kXrayObstructedCellProbabilityHighLimit = 0.6f/*0.550000999999f*/;
  constexpr float kXrayObstructedCellProbabilityLowLimit = 0.551f/*0.550000999999f*/;
//  int gridsize = hybrid_grid.grid_size();
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
//	  if(gridsize > hybrid_grid.grid_size())
//		  break;
    total_submap_voxel_num++;

    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLowLimit) {
		if(probability <0.5){
			miss_submap_voxel_num++;
		}
    	continue;
    }

    const Eigen::Vector3d cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());

    Eigen::Vector3d  cell_now_pose = pose.inverse() * cell_center_submap;
    if(cell_now_pose.x()<min_index.x()||cell_now_pose.x()>max_index.x()
    		||cell_now_pose.y()<min_index.y()||cell_now_pose.y()>max_index.y())
    	continue;




    int ydis = cell_now_pose.y();
    if(ydis<0)
    	ydis = 0;
    else if(ydis>20)
    	ydis=20;
    float kLimit = kXrayObstructedCellProbabilityHighLimit -
    		(kXrayObstructedCellProbabilityHighLimit-kXrayObstructedCellProbabilityLowLimit)*ydis/20;
    if (probability < kLimit) {
      // We ignore non-obstructed cells.
      continue;
    }


    const Eigen::Array4i voxel_index_and_probability(
        ::ivcommon::RoundToInt(cell_now_pose.x() * resolution_inverse),
        ::ivcommon::RoundToInt(cell_now_pose.y() * resolution_inverse),
        ::ivcommon::RoundToInt(cell_now_pose.z() * resolution_inverse),
        probability_value);
//LOG(INFO)<<voxel_index_and_probability(0,0)<<'\t'<<voxel_index_and_probability(0,1);
    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();

    hit_submap_voxel_num++;
  }
//  LOG(INFO)<<"total_submap_voxel_num:"<<total_submap_voxel_num<<'\t'<<"hit_submap_voxel_num:"<<hit_submap_voxel_num<<'\t'<<"miss_submap_voxel_num:"<<miss_submap_voxel_num;
  return voxel_indices_and_probabilities;
}

void ProcessPixelValues(
    const std::vector<PixelData>& accumulated_pixel_data,const float &resolution_,cv::Mat& temimage) {


  const int image_height =  temimage.rows;
  const int image_width = temimage.cols;
  int image_index = 0;

  float kelevationlowerbound = 1.0f/resolution_;//unit(m/resolution)
  float kelevationupperbound = 2.5f/resolution_;
  float kMinZDifference = 0.5f/resolution_;
  float kDensethreshold = 0.5;
  //constexpr float kMinZDifference = 3.f;
 // constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    //image_index++;
	int j = image_height-1-image_index/image_width;
	int i = image_index%image_width;
	unsigned char* pdata = (unsigned char*)temimage.ptr<uchar>(j);
	if(pixel.count ==0)
	{
		pdata[i] = 0;
		image_index++;

	continue;
	}
	const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
	const float free_space = std::max(z_difference - pixel.count, 0.f);

	if(/*(pixel.min_z>kelevationlowerbound&&pixel.min_z<kelevationupperbound)||
	  (pixel.max_z>kelevationlowerbound&&pixel.max_z<kelevationupperbound)||*/z_difference>kMinZDifference&&(pixel.count/z_difference)>kDensethreshold){
		pdata[i] = 254;
		image_index++;

	continue;
	}else{

		pdata[i] = 0;
		image_index++;

		continue;
	}
  }

}

void ProcessPixelValuesProbalility(
    const std::vector<PixelData>& accumulated_pixel_data,const float &resolution_,cv::Mat& temimage) {
  /*******************lzz10171036*******************/


  const int image_height =  temimage.rows;
  const int image_width = temimage.cols;
  int image_index = 0;

  float kelevationlowerbound = 1.0f/resolution_;//unit(m/resolution)
  float kelevationupperbound = 2.5f/resolution_;
  float kMinZDifference = 0.5f/resolution_;
  float kDensethreshold = 0.5;
  //constexpr float kMinZDifference = 3.f;
 // constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    //image_index++;
	int j = image_height-1-image_index/image_width;
	int i = image_index%image_width;
	float* pdata = temimage.ptr<float>(j);
    if(pixel.count ==0)
    {
      pdata[i] = 0;
      image_index++;

    continue;
    }
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    const float free_space = std::max(z_difference - pixel.count, 0.f);

    if(z_difference>kMinZDifference&&(pixel.count/z_difference)>kDensethreshold){
		pdata[i] = (pixel.max_probability);
		image_index++;

    continue;
    }else{

      pdata[i] = 0;
      image_index++;

      continue;
    }
  }

}

std::vector<PixelData> Convert2PixelData(
  /*******************lzz10171036*******************/
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
//     const int x = max_index.x() - pixel_index[0];
//     const int y = max_index.y() - pixel_index[1];
    const int x = pixel_index[0]-min_index.x();
    const int y = pixel_index[1]-min_index.y();
    PixelData& pixel = accumulated_pixel_data[y * width + x];
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

std::vector<PixelData> Convert2PixelDataNear(
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities,ivcommon::transform::Rigid3d pose) {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
//     const int x = max_index.x() - pixel_index[0];
//     const int y = max_index.y() - pixel_index[1];
    const int x = pixel_index[0]-min_index.x();
    const int y = pixel_index[1]-min_index.y();
    PixelData& pixel = accumulated_pixel_data[y * width + x];
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
    const HybridGrid& hybrid_grid, const ivcommon::transform::Rigid3d& transform,
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

    const Eigen::Vector3d cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3d cell_center_global = transform * cell_center_submap;
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
      cell_data.push_back(0);  // value
      cell_data.push_back(0);  // alpha
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = mapping::ClampProbability(
        (pixel.probability_sum + free_space_probability * free_space_weight) /
        total_weight);
    const int delta =
        128 - mapping::ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cell_data.push_back(value);                         // value
    cell_data.push_back((value || alpha) ? alpha : 1);  // alpha
  }
  return cell_data;
}

}  // namespace

proto::SubmapsOptions CreateSubmapsOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_update_common_map(parameter_dictionary->GetBool("update_common_map"));
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_feature_resolution(parameter_dictionary->GetDouble("feature_resolution"));
  options.set_intensity_resolution(parameter_dictionary->GetDouble("intensity_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  options.set_savemap_flag(parameter_dictionary->GetBool("savemap_flag"));
  options.set_readmap_flag(parameter_dictionary->GetBool("readmap_flag"));
  options.set_savemap_dir(parameter_dictionary->GetString("savemap_dir"));
  options.set_readmap_dir(parameter_dictionary->GetString("readmap_dir"));
  options.set_update_flag(parameter_dictionary->GetBool("update_flag"));
  options.set_automode_flag(parameter_dictionary->GetBool("automode_flag"));
  options.set_rotational_histogram_size(parameter_dictionary->GetNonNegativeInt("rotational_histogram_size"));
//  if(options.automode_flag())
//    {
//      options.set_readmap_flag(false) ;
//      options.set_update_flag(true) ;
//    }
  if(options.readmap_flag()&&options.savemap_flag())
  {
	  options.set_savemap_dir(options.readmap_dir());
  }
  else if(options.savemap_flag())
    {
     std::string savemapdir = options.savemap_dir();
     char readable_start_time[100];
     time_t timel;
     time(&timel);
     tm* pTmp=localtime(&timel);

     memset(readable_start_time,0,21);
     sprintf(readable_start_time, "%04d-%02d-%02d_%02d-%02d-%02d",
      pTmp->tm_year + 1900,
      pTmp->tm_mon + 1,
      pTmp->tm_mday,
      pTmp->tm_hour,
      pTmp->tm_min,
      pTmp->tm_sec);

     savemapdir += '/';
     savemapdir += readable_start_time;
     int flag=mkdir(savemapdir.c_str(), 0777);
     if (flag == 0)
     {
       LOG(INFO)<<"make dir successfully"<<std::endl;

     } else {
       LOG(ERROR)<<"make dir errorly"<<std::endl;
     }
     options.set_savemap_dir(savemapdir);
     options.set_readmap_dir(savemapdir);
    }
  return options;
}

Submap::Submap(const float high_resolution, const float low_resolution,const float feature_resolution,const float intensity_resolution,const int histogram_size,
               const ivcommon::transform::Rigid3d& local_pose)
    : mapping::Submap(local_pose),
      high_resolution_hybrid_grid_(high_resolution),
      low_resolution_hybrid_grid_(low_resolution) ,
      feature_hybrid_grid_(feature_resolution),
	  high_intensity_feature_hybrid_grid_(feature_resolution),
	  intensity_hybrid_grid_(intensity_resolution),
	  histogram_(Eigen::VectorXf::Zero(histogram_size))
      {
		point_cloud_.reserve(130000*80);
      }

Submap::Submap(const proto::Submap3D& proto)
    : mapping::Submap(ivcommon::transform::ToRigid3(proto.local_pose())),
      high_resolution_hybrid_grid_(proto.high_resolution_hybrid_grid()),
      low_resolution_hybrid_grid_(proto.low_resolution_hybrid_grid()),
      feature_hybrid_grid_(proto.feature_hybrid_grid()),
	  high_intensity_feature_hybrid_grid_(proto.high_intensity_feature_hybrid_grid()),
	  intensity_hybrid_grid_(proto.intensity_hybrid_grid()),
	  histogram_(Eigen::VectorXf::Zero(proto.histogram_size()))
	  {
		set_global_pose_adjusted(ivcommon::transform::ToRigid3(proto.global_pose_adjusted()));
		for(int i=0;i<proto.histogram_size();i++)
		{
		histogram_[i] = proto.histogram(i);
//		  LOG(WARNING)<<histogram_[i];
		}
		SetIndex(proto.submap_index());
//		SetNumRangeData(proto.num_range_data());
}

void Submap::ToProto(proto::Submap3D* const proto) const {
//   auto* const submap_3d = proto->mutable_submap_3d();
  *proto->mutable_local_pose() = ivcommon::transform::ToProto(local_pose());
  *proto->mutable_global_pose_adjusted() = ivcommon::transform::ToProto(global_pose_adjusted());
  proto->set_num_range_data(num_range_data());
  proto->set_finished(finished_);
  proto->set_submap_index(index());
  *proto->mutable_high_resolution_hybrid_grid() =
      high_resolution_hybrid_grid().ToProto();
  *proto->mutable_low_resolution_hybrid_grid() =
      low_resolution_hybrid_grid().ToProto();
  *proto->mutable_feature_hybrid_grid() =
      feature_hybrid_grid().ToProto();
  for(int i=0;i<histogram_.size();i++)
  {
	  proto->mutable_histogram()->Add(histogram_[i]);
  }
}

/// \brief 加载子地图列表
///
/// 函数详细说明，这里写函数的详细说明信息，说明可以换行
/// ，如这里所示，同时需要注意的是详细说明和简要说明之间必须空一行
/// ，详细说明之前不需要任何标识符
/// \param n1 参数1说明
/// \param c2 参数2说明
/// \return 返回说明
void Submap::InsertRangeData(const sensor::RangeData& range_data,
                             const RangeDataInserter& range_data_inserter,
                             const int high_resolution_max_range,bool update_common_map) {
  CHECK(!finished_);
  const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
      range_data, local_pose().inverse());
  if(update_common_map)
  {
  range_data_inserter.Insert(
      FilterRangeDataByMaxRange(transformed_range_data,
                                high_resolution_max_range),
      &high_resolution_hybrid_grid_);
  updated_ = true;


	  range_data_inserter.Insert(transformed_range_data,
								 &low_resolution_hybrid_grid_);
	//  LOG(INFO)<<"inserting featuremap";

  }
  range_data_inserter.Insert(transformed_range_data,
								 &feature_hybrid_grid_);
  for(auto point:range_data.returns)
  {
	  pcl::PointXYZI temppoint;
	  temppoint.x = point[0];
	  temppoint.y = point[1];
	  temppoint.z = point[2];
	  temppoint.intensity = point.intensity;
	  point_cloud_.push_back(temppoint);
  }
//  range_data_inserter.Insert(transformed_range_data,
//								 &high_intensity_feature_hybrid_grid_,0.9);
//  range_data_inserter.Insert(transformed_range_data,
//								 &intensity_hybrid_grid_);
//  LOG(INFO)<<"inserting Histogram";
  scan_matching::AddValuesToHistogram(
		  scan_matching::GetValuesForHistogram(transformed_range_data.returns),
      0.f, &histogram_);
//  LOG(INFO)<<"insert end";
  SetNumRangeData(num_range_data() + 1);

}



void Submap::save_To_2d_Grid(const HybridGrid& hybrid_grid,const std::string map_name_index){
  const float resolution = hybrid_grid.resolution();
  Eigen::Array2i origin(INT_MAX, INT_MAX);
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ProcessVoxelData(hybrid_grid,
                     &min_index,
                       &max_index);
  const int height = max_index.y() - min_index.y() + 1;
  const int width = max_index.x() - min_index.x() + 1;

  const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);//each pixelData include the min_z/max_z/probability_sum/max_probability and the global location
  cv::Mat TwiDmapImage = cv::Mat::zeros(cv::Size(width,height),CV_8UC1);
  ProcessPixelValues(accumulated_pixel_data,resolution,TwiDmapImage);
//   std::string output;
//   ::ivcommon::FastGzipString(cell_data, &output);
  origin = -min_index;

  std::string map_header_name = map_name_index+".2dmap";
  std::ofstream twod_submap_writer;
  twod_submap_writer.open(map_header_name.c_str(),std::ios::out);
  twod_submap_writer<<resolution<<'\t'<<width<<'\t'<<height<<'\t'<<origin.x()<<'\t'<<origin.y()<<'\t'
  <<this->local_pose().translation().x()<<'\t'<<this->local_pose().translation().y()<<
  '\t'<<this->local_pose().translation().z();
  std::string map_img_name = map_name_index+".jpg";


  cv::imwrite(map_img_name.c_str(),TwiDmapImage);

  twod_submap_writer.close();
}

void Submap::covert_To_2d_Grid(const HybridGrid& hybrid_grid,const ivcommon::transform::Rigid3d& pose_estimate){

	if(!updated_)
	{
		usleep(1000);
		return;
	}
	updated_=false;

  ::ivcommon::Time time1 = ::ivcommon::now();

  const float resolution = hybrid_grid.resolution();
  Eigen::Array2i origin(0, 0);
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ProcessVoxelData(hybrid_grid,
                     &min_index,
                       &max_index);
  const int height = max_index.y() - min_index.y() + 1;
  const int width = max_index.x() - min_index.x() + 1;
//  LOG(INFO)<<"width"<<width<<"height"<<height;
  const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);//each pixelData include the min_z/max_z/probability_sum/max_probability and the global location

//  LOG(INFO)<<"convert_time_elapsed2:"<<::ivcommon::ToSeconds(::ivcommon::now()- time1)<<"!!!!!!!!!!!!!!!!!!!!!2";
  cv::Mat TwiDmapImage = cv::Mat::zeros(cv::Size(width,height),CV_8UC1);

  ProcessPixelValues(accumulated_pixel_data,resolution,TwiDmapImage);
  origin = -min_index;

  float showresolution = 0.2;
  float scale = showresolution/resolution;
  int showheight = height/scale;
  int showwidth = width/scale;
//  origin = origin/scale;
  cv::Mat Display_Image = cv::Mat::zeros(cv::Size(showwidth,showheight),CV_8UC3);
  for(int j = 0 ;j<Display_Image.rows;j++){
 	  int indexj = j*scale;
	  unsigned char* pdata  = (unsigned char*)TwiDmapImage.ptr<uchar>(indexj);
	  unsigned char* display_data = (unsigned char*)Display_Image.ptr<uchar>(j);
     for(int i = 0 ;i<Display_Image.cols;i++){
    	 int indexi = i*scale;


       //if(*pdata == 254)
       {

	 display_data[3*i]= pdata[indexi];
	 display_data[3*i+1]= pdata[indexi];
	 display_data[3*i+2]= pdata[indexi];
      }

    }
  }


	int heightnum = showheight/(10/showresolution);
	int widthnum = showwidth/(10/showresolution);
	for(int i = 0; i<heightnum ;i++)
	  {
		cv::line(Display_Image,cv::Point(0,Display_Image.rows*i/heightnum),
				cv::Point(Display_Image.cols-1,Display_Image.rows*i/heightnum),cv::Scalar(255,0,0));
	  }

	for(int i=1;i<widthnum;i++)
	{
		cv::line(Display_Image,cv::Point(Display_Image.cols*i/widthnum,0),
				cv::Point(Display_Image.cols*i/widthnum,Display_Image.rows-1),cv::Scalar(255,0,0));
	}

	float ogmresolution = showresolution;

    cv::rectangle(Display_Image,cv::Point(origin.x()/scale-1/ogmresolution,
    		showheight-1-origin.y()/scale-2/ogmresolution),
            cv::Point(origin.x()/scale+1/ogmresolution,
            		showheight-1-origin.y()/scale+2/ogmresolution),
            cv::Scalar(0,0,255));


    char  windowname[60];
    std::sprintf(windowname,"twiD submap");
    ::ivcommon::transform::Rigid3d poseinlocal = local_pose().inverse()*pose_estimate;

    Eigen::Array2i vehiclepos(::ivcommon::RoundToInt(poseinlocal.translation().x()/resolution)
    							,::ivcommon::RoundToInt(poseinlocal.translation().y()/resolution));

    vehiclepos -= min_index;

    cv::circle(Display_Image,cv::Point(vehiclepos.x()/scale,showheight-1-vehiclepos.y()/scale),5,
            cv::Scalar(0,255,0),-1);

//    cvNamedWindow(windowname,0);
    cv::imshow(windowname,Display_Image);
    cv::waitKey(5);


}

void Submap::covert_To_2d_NowGrid(const HybridGrid& hybrid_grid,const ivcommon::transform::Rigid3d& pose_estimate){

	if(!updated_)
	{
		usleep(1000);
		return;
	}
	updated_=false;
  ::ivcommon::Time time1 = ::ivcommon::now();
  ivcommon::transform::Rigid3d poseinlocal = local_pose().inverse()*pose_estimate;
  const float resolution = hybrid_grid.resolution();
  Eigen::Array2i origin(0, 0);
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  min_index = Eigen::Array2i(-20/resolution-1,-20/resolution-1);//min_index->cwiseMin(pixel_index);
  max_index = Eigen::Array2i(20/resolution+1,60/resolution+1);//max_index->cwiseMax(pixel_index);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
		  ProcessVoxelDataNear(hybrid_grid,
                     min_index,
                       max_index,poseinlocal);

  const int height = max_index.y() - min_index.y() + 1;
  const int width = max_index.x() - min_index.x() + 1;
//  LOG(INFO)<<"width"<<width<<"height"<<height;
  const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);//each pixelData include the min_z/max_z/probability_sum/max_probability and the global location

//  LOG(INFO)<<"convert_time_elapsed1:"<<::ivcommon::ToSeconds(::ivcommon::now()- time1)<<"!!!!!!!!!!!!!!!!!!!!!1";

  cv::Mat TwiDmapImage = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);


  //IplImage* TwiDmapImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
//  cvZero(TwiDmapImage);
  ProcessPixelValuesProbalility(accumulated_pixel_data,resolution,TwiDmapImage);

//  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
//  cv::morphologyEx(TwiDmapImage, TwiDmapImage, cv::MORPH_CLOSE, element);
//
////  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
////  cv::morphologyEx(TwiDmapImage, TwiDmapImage, cv::MORPH_GRADIENT, element);
//
//  cv::Mat kern = (cv::Mat_<double>(5,3) <<  0.75, 1.5,  0.75,                          // 生成一个掩模核 大小为 3x3 ， 通过<<  输入到矩阵Mat_<char> 中，然后隐式转换成Mat类型
//                                 -0.25,  -0.5, -0.25,
//								 -0.25,  -0.5, -0.25,
//								 -0.25,  -0.5, -0.25,
//                                  0.25, 0.5,  0.25);
//  kern = kern/2;
//
////  cv::Mat out;
//  cv::filter2D(TwiDmapImage, TwiDmapImage, TwiDmapImage.depth(), kern );
  origin = -min_index;

  float showresolution = 0.2;
  float scale = showresolution/resolution;
  int showheight = height/scale;
  int showwidth = width/scale;
  cv::Mat Display_Image = cv::Mat::zeros(cv::Size(showwidth,showheight),CV_8UC3);

//  cv::imshow("out",out);

  for(int j = 0 ;j<Display_Image.rows;j++){
 	  int indexj = j*scale;
	  float* pdata  = TwiDmapImage.ptr<float>(indexj);
	  unsigned char* display_data = (unsigned char*)Display_Image.ptr<uchar>(j);
     for(int i = 0 ;i<Display_Image.cols;i++){
    	 int indexi = i*scale;
    	 uchar pixel;
    	 if(pdata[indexi]<0)
    		 pixel=0;
    	 else if(pdata[indexi]>1)
    		 pixel=255;
    	 else
    	     pixel=pdata[indexi]*255;
       //if(*pdata == 254)
       {

	 display_data[3*i]= pixel;
	 display_data[3*i+1]= pixel;
	 display_data[3*i+2]= pixel;
      }

    }
  }


  	int linestep = 10/showresolution;
	int heightnum = showheight/(linestep);
	int widthnum = showwidth/(linestep);
	for(int i = 0; i<heightnum ;i++)
	  {
		cv::line(Display_Image,cv::Point(0,linestep*i),
				cv::Point(Display_Image.cols-1,linestep*i),cv::Scalar(255,0,0));
	  }

	for(int i=1;i<widthnum;i++)
	{
		cv::line(Display_Image,cv::Point(linestep*i,0),
				cv::Point(linestep*i,Display_Image.rows-1),cv::Scalar(255,0,0));
	}



    char  windowname[60];
    std::sprintf(windowname,"now submap");


    cv::circle(Display_Image,cv::Point(origin.x()/scale,showheight-1-origin.y()/scale),5,
            cv::Scalar(0,255,0),-1);

//    cvNamedWindow(windowname,0);
    cv::imshow(windowname,Display_Image);
    cv::waitKey(5);

//    LOG(INFO)<<"convert_time_elapsed2:"<<::ivcommon::ToSeconds(::ivcommon::now()- time1)<<"!!!!!!!!!!!!!!!!!!!!!1";
}


//void Submap::covert_To_2d_Grid(const HybridGrid& hybrid_grid){
//	cvNamedWindow("image",0);
//	::ivcommon::Time timestart = ::ivcommon::now();
//  const float resolution = hybrid_grid.resolution();
//  Eigen::Array2i origin(INT_MAX, INT_MAX);
//  Eigen::Array2i min_index(INT_MAX, INT_MAX);
//  Eigen::Array2i max_index(INT_MIN, INT_MIN);
//  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
//      ProcessVoxelData(hybrid_grid,
//                     &min_index,
//                       &max_index);
//  const int height = max_index.y() - min_index.y() + 1;
//  const int width = max_index.x() - min_index.x() + 1;
//  LOG(INFO)<<"height:"<<height<<" width:"<<width<<" covert_To_2d_Grid cost1:"<<::ivcommon::ToSeconds(::ivcommon::now()-timestart);
//  const std::vector<PixelData> accumulated_pixel_data = Convert2PixelData(
//      width, height, min_index, max_index, voxel_indices_and_probabilities);//each pixelData include the min_z/max_z/probability_sum/max_probability and the global location
//  IplImage* TwiDmapImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
//  const std::vector<int> cell_data = ProcessPixelValues(accumulated_pixel_data,resolution,TwiDmapImage);
////   std::string output;
////   ::ivcommon::FastGzipString(cell_data, &output);
//  origin = -min_index;
//
//  LOG(INFO)<<"height:"<<height<<" width:"<<width<<" covert_To_2d_Grid cost2:"<<::ivcommon::ToSeconds(::ivcommon::now()-timestart);
//
//  cvShowImage("image",TwiDmapImage);
//  cvWaitKey(1);
//  cvReleaseImage(&TwiDmapImage);
//
//}

void Submap::save_histogram(const Eigen::VectorXf& histogram,const std::string filename)
{
	std::fstream file(filename,std::ios::out);
	int size = histogram.size();
	file<<size<<std::endl;
	for(int i=0;i<size;i++)
	{
		file<<histogram(i)<<std::endl;
	}

}

void Submap::load_histogram(Eigen::VectorXf& histogram,const std::string filename)
{

	std::fstream file(filename,std::ios::in);
	int size;
	file>>size;
	histogram = Eigen::VectorXf::Zero(size);
	for(int i=0;i<size;i++)
	{
		file>>histogram(i);
	}
}
/// \brief 保存地图
///
/// 保存地图数据，按照自己定义的格式（速度较慢已弃用）
/// \param mapdir 地图保存文件夹
void Submap::save(const std::string& mapdir)
{
  std::stringstream sstr;
  sstr<<"/"<<index();

  std::string filename = mapdir + sstr.str() + ".fmap";
  feature_hybrid_grid_.save(filename.c_str());
  filename = mapdir + sstr.str();
  save_To_2d_Grid(high_resolution_hybrid_grid_,filename);
  filename = mapdir + sstr.str() + ".hist";
  save_histogram(histogram_,filename);
  filename = mapdir + sstr.str() + ".h3dmap";
  high_resolution_hybrid_grid_.save(filename.c_str());
  filename = mapdir + sstr.str() + ".l3dmap";
  low_resolution_hybrid_grid_.save(filename.c_str());

}

/// \brief 保存proto格式地图
///
/// 保存地图数据，按照proto格式，速度较快
/// \param mapdir 地图保存文件夹
void Submap::saveToProto(const std::string& mapdir)
{
  std::stringstream sstr;
  sstr<<"/"<<index();

  string filename = mapdir + sstr.str();
  LOG(WARNING)<<"save dir is "<<filename;
  save_To_2d_Grid(high_resolution_hybrid_grid_,filename);
  filename = mapdir + sstr.str() + ".pcd";
  pcl::io::savePCDFile(filename,point_cloud_,true);
  proto::Submap3D proto;
  this->ToProto(&proto);
  filename = mapdir + sstr.str() + ".proto";
  ::ivcommon::io::ProtoStreamWriter protowriter(filename);
  protowriter.WriteProto(proto);
  filename = mapdir + sstr.str() + ".traj";
  std::ofstream trajectory_writer;
  trajectory_writer.open(filename.c_str(),std::ios::out);
  auto& trajectory = this->trajectory();

  std::vector<mapping::trajectoryPose> trimed_trajectory;

  for(const auto& node:trajectory)
  {
	  if(node.withfusepose)
		  trimed_trajectory.push_back(node);
  }
  if(trimed_trajectory.size() <= trajectory.size()/2)
	  trimed_trajectory = trajectory;
  if(trimed_trajectory.size()==0)
  {
	  LOG(ERROR)<<index()<<" the num of trajectory node is zero!";
  }
  trajectory_writer<<trimed_trajectory.size()<<std::endl;
  for(const auto& node:trimed_trajectory)
  {
	  trajectory_writer<<std::fixed
			  <<std::setprecision(7)<<node.estimatepose.rotation().x()<<" "<<node.estimatepose.rotation().y()<<" "<<node.estimatepose.rotation().z()<<" "<<node.estimatepose.rotation().w()<<" "
			  <<std::setprecision(3)<<node.estimatepose.translation()[0]<<" "<<node.estimatepose.translation()[1]<<" "<<node.estimatepose.translation()[2]<<" "
			  <<std::setprecision(7)<<node.gpspose.rotation().x()<<" "<<node.gpspose.rotation().y()<<" "<<node.gpspose.rotation().z()<<" "<<node.gpspose.rotation().w()<<" "
			  <<std::setprecision(3)<<node.gpspose.translation()[0]<<" "<<node.gpspose.translation()[1]<<" "<<node.gpspose.translation()[2]<<" "
			  <<node.withfusepose<<" "
			  <<std::setprecision(7)<<node.fusepose.rotation().x()<<" "<<node.fusepose.rotation().y()<<" "<<node.fusepose.rotation().z()<<" "<<node.fusepose.rotation().w()<<" "
			  <<std::setprecision(3)<<node.fusepose.translation()[0]<<" "<<node.fusepose.translation()[1]<<" "<<node.fusepose.translation()[2]<<std::endl;
  }

}

void Submap::load(int index,const std::string& mapdir)
{
  this->SetIndex(index);
  std::stringstream sstr;
  sstr<<"/"<<index;
  std::string filename = mapdir + sstr.str() + ".fmap";
  feature_hybrid_grid_.load(filename.c_str());
  filename = mapdir + sstr.str() + ".hist";
  load_histogram(histogram_,filename);
  filename = mapdir + sstr.str() + ".h3dmap";
  high_resolution_hybrid_grid_.load(filename.c_str());
  filename = mapdir + sstr.str() + ".l3dmap";
  low_resolution_hybrid_grid_.load(filename.c_str());
}

void Submap::Finish() {
  CHECK(!finished_);
  finished_ = true;
}

ActiveSubmaps::ActiveSubmaps(const proto::SubmapsOptions& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()),
	  updated_(false){
  // We always want to have at least one submap which we can return and will
  // create it at the origin in absence of a better choice.
  //
  // TODO(whess): Start with no submaps, so that all of them can be
  // approximately gravity aligned.
//  if(!options_.readmap_flag())
//    AddSubmap(ivcommon::transform::Rigid3d::Identity());
}

std::deque<std::shared_ptr<Submap>> ActiveSubmaps::submaps() const {
	mutex_.lock();
	auto res = submaps_;
	mutex_.unlock();
  return res;
}

int ActiveSubmaps::matching_index() const { return matching_submap_index_; }

//void ActiveSubmaps::set_matching_index(int index) {
//  matching_submap_index_ = index;
//  int plusnum = 0;
//  for (auto& submap : submaps_) {
//	  submap->SetIndex(matching_submap_index_+plusnum);
//	  plusnum++;
//  }
//}

void ActiveSubmaps::InsertRangeData(
    const sensor::RangeData& range_data,
    const Eigen::Quaterniond& gravity_alignment) {

    mutex_.lock();
  if(options_.update_flag())
    {

      for (auto& submap : submaps_) {
	  if(submap->finished())
	    continue;
        submap->InsertRangeData(range_data, range_data_inserter_,
                                options_.high_resolution_max_range(),options_.update_common_map());
//        if(submap->index()==matching_submap_index_)
//        	submap->covert_To_2d_Grid(submap->high_resolution_hybrid_grid());

      }
      updated_=true;

    }
  mutex_.unlock();
  if (!options_.readmap_flag() && submaps_.back()->num_range_data() >= options_.num_range_data() && submaps_.back()->feature_hybrid_grid().get_confirmed_voxel_number() > 2e3) {
      AddSubmap(ivcommon::transform::Rigid3d(range_data.origin.cast<double>(),
                                   gravity_alignment));
  }

}
/// \brief 加载子地图列表
///
/// 函数详细说明，这里写函数的详细说明信息，说明可以换行
/// ，如这里所示，同时需要注意的是详细说明和简要说明之间必须空一行
/// ，详细说明之前不需要任何标识符
/// \param n1 参数1说明
/// \param c2 参数2说明
/// \return 返回说明
void ActiveSubmaps::ProcessMatchingSubmap(const ivcommon::transform::Rigid3d pose_estimate)
{
	if(submaps_.size()>0)
	{
		mutex_.lock();
		auto submap = submaps_.front();
		mutex_.unlock();
		submap->covert_To_2d_NowGrid(submaps_.front()->high_resolution_hybrid_grid(),pose_estimate);
	}
	else
		usleep(1000);
}

void ActiveSubmaps::AddSubmap(const ivcommon::transform::Rigid3d& local_pose) {
  mutex_.lock();
  if (submaps_.size() > 1) {
      submaps_.front()->Finish();
      ++matching_submap_index_;
  }
  int index = -1;
  if(submaps_.size()>0)
	  index = submaps_.back()->index()+1;
  else
	  index = matching_submap_index_;
  submaps_.emplace_back(new Submap(options_.high_resolution(),options_.low_resolution(), options_.feature_resolution(),
		  	  	  	  	  	  	  	  options_.intensity_resolution(),
                                   options_.rotational_histogram_size(),local_pose));
  submaps_.back()->set_global_pose_adjusted(global_init_pose().pose*local_pose);
  submaps_.back()->SetIndex(index);
  if(submaps_.size()>1)
  {
//	  submaps_[submaps_.size()-2]->AddLinkIndex(submaps_.back()->index());
	  submaps_.back()->AddLinkIndex(submaps_[submaps_.size()-2]->index());
  }
//  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
  mutex_.unlock();
}

void ActiveSubmaps::AddPose(const mapping::trajectoryPose& pose)
{
	  if(options_.update_flag())
	    {
	      mutex_.lock();
	      for (auto& submap : submaps_) {
		  if(submap->finished())
		    continue;
	        submap->AddPose(pose);
	      }
	      mutex_.unlock();
	    }
}

void ActiveSubmaps::AddFusePose(const ivcommon::transform::posestamped& fusepose)
{
	  if(options_.update_flag())
	    {
	      mutex_.lock();
	      for (auto& submap : submaps_) {
		  if(submap->finished())
		    continue;
	        submap->AddFusePose(fusepose);
	      }
	      mutex_.unlock();
	    }
}


}  // namespace mapping_3d

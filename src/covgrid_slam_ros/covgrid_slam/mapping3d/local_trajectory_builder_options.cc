/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "covgrid_slam/mapping3d/local_trajectory_builder_options.h"

#include "ivcommon/common/configuration_file_resolver.h"
#include "glog/logging.h"
#include "covgrid_slam/mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "covgrid_slam/mapping3d/motion_filter.h"
#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher.h"
#include "covgrid_slam/mapping3d/scan_matching/fast_correlative_scan_matcher.h"
#include "covgrid_slam/mapping3d/submaps.h"
#include "covgrid_slam/sensor/voxel_filter.h"

namespace mapping3d {
/// \brief 读取配置文件
///
/// \param parameter_dictionary 配置所在文件
/// \return 里程计配置
proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_scans_per_accumulation(
      parameter_dictionary->GetInt("scans_per_accumulation"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  *options.mutable_high_resolution_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("high_resolution_adaptive_voxel_filter")
              .get());
  *options.mutable_low_resolution_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("low_resolution_adaptive_voxel_filter")
              .get());

  *options.mutable_feature_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("feature_adaptive_voxel_filter")
              .get());
  *options.mutable_intensity_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("intensity_adaptive_voxel_filter")
              .get());
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_scan_matcher_options() =
      scan_matching::CreateScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() = CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));//10
  options.set_without_imu(
      parameter_dictionary->GetBool("without_imu"));//10
  *options.mutable_submaps_options() = mapping3d::CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  *options.mutable_fastcorrelativescanmatcher_options() = mapping3d::scan_matching::CreateFastCorrelativeScanMatcherOptions(
		  parameter_dictionary->GetDictionary("fast_correlative_scan_matcher_3d").get());
  *options.mutable_initmatch_options() = CreateInitMatchOptions(
		  parameter_dictionary->GetDictionary("initmatch").get());
  return options;
}
/// \brief 加载初始化匹配配置
///
/// \param parameter_dictionary 配置文件
/// \return 初始化匹配的配置
proto::InitMatchOptions CreateInitMatchOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary) {
  proto::InitMatchOptions options;
  options.set_min_score(
      parameter_dictionary->GetDouble("min_score"));
  options.set_min_low_resolution_score(
      parameter_dictionary->GetDouble("min_low_resolution_score"));
  return options;
}
/// \brief 加载配置文件
///
/// \param configuration_directory 配置文件夹
/// \param configuration_basename 配置文件名
/// \return 雷达里程计参数
proto::LocalTrajectoryBuilderOptions LoadOptions(
    const string& configuration_directory,
    const string& configuration_basename) {
  auto file_resolver = ::ivcommon::make_unique<
      ::ivcommon::ConfigurationFileResolver>(
      std::vector<string>{configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  ::ivcommon::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));

//  auto lua_parameter_dictionary =
//      ::ivcommon::LuaParameterDictionary::NonReferenceCounted(
//          code, std::move(file_resolver));
  return  CreateLocalTrajectoryBuilderOptions(&lua_parameter_dictionary);
}

}  // namespace mapping_3d

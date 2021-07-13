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

#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher.h"

#include <string>
#include <utility>
#include <vector>

#include "ivcommon/common/make_unique.h"

#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

#include "covgrid_slam/common/optimize_solver_options.h"
#include "covgrid_slam/mapping3d/scan_matching/feature_space_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/intensity_space_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/occupied_space_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/rotation_delta_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/translation_delta_cost_functor.h"

namespace mapping3d {
namespace scan_matching {
namespace {

struct YawOnlyQuaternionPlus {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    const T clamped_delta = ::ivcommon::Clamp(delta[0], T(-0.5), T(0.5));
    T q_delta[4];
    q_delta[0] = ceres::sqrt(1. - clamped_delta * clamped_delta);
    q_delta[1] = T(0.);
    q_delta[2] = T(0.);
    q_delta[3] = clamped_delta;
    ceres::QuaternionProduct(q_delta, x, x_plus_delta);
    return true;
  }
};

}  // namespace

proto::ScanMatcherOptions CreateScanMatcherOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary) {
  proto::ScanMatcherOptions options;
  for (int i = 0;; ++i) {
    const string lua_identifier = "occupied_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_occupied_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }
  for (int i = 0;; ++i) {
    const string lua_identifier = "feature_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_feature_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }
  for (int i = 0;; ++i) {
    const string lua_identifier = "intensity_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_intensity_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }

  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_only_optimize_yaw(
      parameter_dictionary->GetBool("only_optimize_yaw"));
  options.set_g2o_optimize(parameter_dictionary->GetBool("g2o_optimize"));
  options.set_fuse_ins(parameter_dictionary->GetBool("fuse_ins"));
  *options.mutable_ceres_solver_options() =
      ::common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  *options.mutable_g2o_solver_options() =
      ::common::CreateG2oSolverOptionsProto(
          parameter_dictionary->GetDictionary("g2o_solver_options").get());
  return options;
}

CeresScanMatcher::CeresScanMatcher(
    const proto::ScanMatcherOptions& options)
    : options_(options),
      ceres_solver_options_(
          ::common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
//  ceres_solver_options_.minimizer_progress_to_stdout=true;
}

void CeresScanMatcher::Match(const ivcommon::transform::posestamped& previous_pose,
			     const ivcommon::transform::posestamped& initial_pose_estimate,
			     const ivcommon::transform::posestamped& last_pose,
                             const std::vector<PointCloudAndHybridGridPointers>&
                                 point_clouds_and_hybrid_grids,
			     const std::vector<PointCloudAndFeatureHybridGridPointers>&
				 point_clouds_and_featurehybrid_grids,
				 const std::vector<PointCloudAndIntensityHybridGridPointers>&
				 				 point_clouds_and_intensityhybrid_grids,
                             ivcommon::transform::Rigid3d* const pose_estimate,
                             double* const cost) {
  static int num = 0;
//  LOG(INFO)<<"ceres optimize";
  ceres::Problem problem;

  CeresPose ceres_pose(
      initial_pose_estimate, nullptr /* translation_parameterization */,
      options_.only_optimize_yaw()
          ? std::unique_ptr<ceres::LocalParameterization>(
                ::ivcommon::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>())
          : std::unique_ptr<ceres::LocalParameterization>(
                ::ivcommon::make_unique<ceres::QuaternionParameterization>()),
      &problem);

  CHECK_EQ(options_.occupied_space_weight_size(),
	   point_clouds_and_hybrid_grids.size());
  for (size_t i = 0; i != point_clouds_and_hybrid_grids.size(); ++i) {
    if(options_.occupied_space_weight(i)<=0)
      continue;
    CHECK_GT(options_.occupied_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
	*point_clouds_and_hybrid_grids[i].first;
    const HybridGrid& hybrid_grid = *point_clouds_and_hybrid_grids[i].second;
    problem.AddResidualBlock(
	new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor,
					ceres::DYNAMIC, 3, 4>(
	    new OccupiedSpaceCostFunctor(
		options_.occupied_space_weight(i) /
		    std::sqrt(static_cast<double>(point_cloud.size())),
		point_cloud, hybrid_grid,{last_pose,initial_pose_estimate}),
	    point_cloud.size()),
	nullptr, ceres_pose.translation(), ceres_pose.rotation());
  }

  CHECK_EQ(options_.feature_space_weight_size(),
	   point_clouds_and_featurehybrid_grids.size());
  for (size_t i = 0; i != point_clouds_and_featurehybrid_grids.size(); ++i) {
    if(options_.feature_space_weight(i)<=0)
      continue;
    CHECK_GT(options_.feature_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
	*point_clouds_and_featurehybrid_grids[i].first;

    LOG(INFO)<<"featurenum:"<<point_cloud.size();
    LOG(INFO)<<"featurenum:"<<point_cloud.size();
    if(point_cloud.size()<10)
      continue;
    const FeatureHybridGrid& hybrid_grid = *point_clouds_and_featurehybrid_grids[i].second;
    problem.AddResidualBlock(
	new ceres::AutoDiffCostFunction<FeatureSpaceCostFunctor,
					ceres::DYNAMIC, 3, 4>(
	    new FeatureSpaceCostFunctor(
		options_.feature_space_weight(i) /
		    std::sqrt(static_cast<double>(point_cloud.size())),
		    point_cloud, hybrid_grid,{last_pose,initial_pose_estimate}),
		    point_cloud.size()),
    nullptr, ceres_pose.translation(), ceres_pose.rotation());
  }

  for (size_t i = 0; i != point_clouds_and_intensityhybrid_grids.size(); ++i) {
    if(options_.intensity_space_weight(i)<=0)
      continue;
    CHECK_GT(options_.intensity_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
	*point_clouds_and_intensityhybrid_grids[i].first;

    LOG(INFO)<<"intensitynum:"<<point_cloud.size()<<std::endl;
    if(point_cloud.size()<10)
      continue;
    const IntensityHybridGrid& hybrid_grid = *point_clouds_and_intensityhybrid_grids[i].second;
    problem.AddResidualBlock(
	new ceres::AutoDiffCostFunction<IntensitySpaceCostFunctor,
					ceres::DYNAMIC, 3, 4>(
	    new IntensitySpaceCostFunctor(
		options_.intensity_space_weight(i) /
		    std::sqrt(static_cast<double>(point_cloud.size())),
		    point_cloud, hybrid_grid,{last_pose,initial_pose_estimate}),
		    point_cloud.size()),
	nullptr, ceres_pose.translation(), ceres_pose.rotation());
  }

  if(options_.translation_weight()>0) {
    CHECK_GT(options_.translation_weight(), 0.);
    problem.AddResidualBlock(
	new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 3, 3>(
	    new TranslationDeltaCostFunctor(options_.translation_weight(),
					    previous_pose.pose)),
	nullptr, ceres_pose.translation());
  }
  if(options_.rotation_weight()>0) {
    CHECK_GT(options_.rotation_weight(), 0.);
    problem.AddResidualBlock(
	new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
	    new RotationDeltaCostFunctor(options_.rotation_weight(),
					 initial_pose_estimate.pose.rotation())),
	nullptr, ceres_pose.rotation());

  }
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem,&summary);
//  if(summary->termination_type == ceres::TerminationType::NO_CONVERGENCE)
//    LOG(WARNING)<<"NO_CONVERGENCE";
//  LOG(INFO)<<summary->FullReport().c_str();
  *cost = summary.final_cost;
  *pose_estimate = ceres_pose.ToRigid();

  num++;
}

}  // namespace scan_matching
}  // namespace mapping_3d

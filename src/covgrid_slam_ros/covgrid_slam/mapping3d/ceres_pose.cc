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

#include "covgrid_slam/mapping3d/ceres_pose.h"

namespace mapping3d {

CeresPose::CeresPose(
    const ivcommon::transform::posestamped& rigid,
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : translation_({{rigid.pose.translation().x(), rigid.pose.translation().y(),
      rigid.pose.translation().z()}}),
      rotation_({{rigid.pose.rotation().w(), rigid.pose.rotation().x(),
	rigid.pose.rotation().y(), rigid.pose.rotation().z()}}) {
  problem->AddParameterBlock(translation_.data(), 3,
                             translation_parametrization.release());
  problem->AddParameterBlock(rotation_.data(), 4,
                             rotation_parametrization.release());
}

const ivcommon::transform::Rigid3d CeresPose::ToRigid() const {
  return ivcommon::transform::Rigid3d(
      Eigen::Map<const Eigen::Vector3d>(translation_.data()),
      Eigen::Quaterniond(rotation_[0], rotation_[1], rotation_[2],
                         rotation_[3]));
}

}  // namespace mapping_3d

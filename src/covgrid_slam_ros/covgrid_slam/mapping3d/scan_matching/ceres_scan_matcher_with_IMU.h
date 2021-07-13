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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_IMUSCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_IMUSCAN_MATCHER_H_

#include <utility>
#include <vector>
#include <glog/logging.h>

#include "Eigen/Core"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "covgrid_slam/mapping3d/scan_matching/proto/scan_matcher_options.pb.h"
#include "ivcommon/transform/rigid_transform.h"

#include "covgrid_slam/mapping3d/ceres_pose.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/sensor/data_type.h"
#include "covgrid_slam/mapping3d/submaps.h"

#include "ceres_scan_matcher_with_IMU/imu_cost_functor.h"

namespace mapping3d {

namespace scan_matching {


using PointCloudAndSubmap =
    std::pair<sensor::PointCloud, std::shared_ptr<const Submap>>;
using PointCloudPtrAndSubmap =
        std::pair<sensor::PointCloud* , std::shared_ptr<const Submap>>;
// This scan matcher uses Ceres to align scans with an existing map.
class Ceres_IMUScan_Matcher {
public:
    explicit Ceres_IMUScan_Matcher(const proto::ScanMatcherOptions& options, ::ivcommon::Time time, const mapping3d::PosewithGps& global_pose_init);

    Ceres_IMUScan_Matcher(const Ceres_IMUScan_Matcher&) = delete;
    Ceres_IMUScan_Matcher& operator=(const Ceres_IMUScan_Matcher&) = delete;

    // Aligns 'point_clouds' within the 'hybrid_grids' given an
    // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
    // 'summary'.
    void Match(const ivcommon::transform::posestamped& initial_pose_estimate,
         const Eigen::Vector3d& linear_vel,
         const ivcommon::transform::posestamped& last_pose,
               const std::deque<sensor::ImuData>& imudata,
               const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap,
             ivcommon::transform::Rigid3d* pose_estimate,
             double* cost);

    enum SolverFlag
    {
    INITIAL,
    NON_LINEAR
    };

    enum MarginalizationFlag
    {
    MARGIN_OLD = 0,
    MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Eigen::Vector3d g;

    //states in b[i] in reference world.
    // the pose of the world reference is the real world pose, and position(pos) is c0 in initialStructure().
    Eigen::Vector3d Ps_l, Ps_c;
    Eigen::Quaterniond Qs_l, Qs_c;///double* ptr = Qs[j].coeffs().data(); ptr[i]=num; the way to access data inside Quat
    Eigen::Vector3d Vs_l, Vs_c;
    Eigen::Vector3d Bas_l, Bas_c;
    Eigen::Vector3d Bgs_l, Bgs_c;
    Eigen::Vector3d P_last_; /// for motion degradation check
    Eigen::Quaterniond Q_last_;

    IntegrationBase *pre_integrations;
    Eigen::Vector3d acc_0, gyr_0;// newest acc and gyr

    std::vector<double> dt_buf;
    std::vector<Vector3d> linear_acceleration_buf;
    std::vector<Vector3d> angular_velocity_buf;
    PointCloudAndSubmap feature_pointcloud_submap_buf;

    int frame_count;
    bool first_imu;
    Eigen::Vector3d linear_vel_extern_ = Eigen::Vector3d::Zero();//body frame


private:
    void ClearStates();
    void prepareIMU(const std::deque<sensor::ImuData>& imudata, ::ivcommon::Time pointcloudTime);
    void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processPointCloud(const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap);
    bool check_degraded_movement();
    void solveodometry( const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap);
    void slidewindow( const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap);
    void logIMUdata();
    const proto::ScanMatcherOptions options_;
    ceres::Solver::Options ceres_solver_options_;
    double solve_cost_;

    ::ivcommon::Time current_time_;

};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_IMUSCAN_MATCHER_H_

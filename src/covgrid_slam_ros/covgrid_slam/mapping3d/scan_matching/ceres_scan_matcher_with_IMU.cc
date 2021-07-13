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

#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher_with_IMU.h"

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

    Ceres_IMUScan_Matcher::Ceres_IMUScan_Matcher(
    const proto::ScanMatcherOptions& options, ::ivcommon::Time time, const mapping3d::PosewithGps& global_pose_init)
    : options_(options), current_time_(time),
      ceres_solver_options_(
          ::common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
//  ceres_solver_options_.minimizer_progress_to_stdout=true;
  ClearStates();
  Qs_l = global_pose_init.pose.rotation();
  auto euler = ivcommon::transform::toRollPitchYaw(Qs_l);
  Qs_l = ivcommon::transform::RollPitchYaw(euler(0), euler(1), 0 * M_PI / 180);//2.709
  Qs_c = Qs_l;
  double latitude_rad = global_pose_init.gps.latitude * M_PI / 180;
  double altitude = global_pose_init.gps.altitude;
  double g_norm = 9.78046 * (1 + 0.0052884 * pow(sin(latitude_rad), 2) - 0.0000059 * pow(sin(2 * latitude_rad), 2)) - 0.000003086 * altitude;
  g = Eigen::Vector3d(0, 0, g_norm);

}

void Ceres_IMUScan_Matcher::ClearStates()
{

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Qs_l.setIdentity();
        Ps_l.setZero();
        Vs_l.setZero();
        Bas_l.setZero();
        Bgs_l.setZero();
        Qs_c.setIdentity();
        Ps_c.setZero();
        Vs_c.setZero();
        Bas_c.setZero();
        Bgs_c.setZero();
        dt_buf.clear();
        linear_acceleration_buf.clear();
        angular_velocity_buf.clear();

        if (pre_integrations != nullptr)
        {
            delete pre_integrations;
            pre_integrations = nullptr;
        }

    }

    //solver_flag = INITIAL;
    first_imu = false;
    frame_count = 0;
    //TODO: make ldiar-IMU alignment

//    if (tmp_pre_integration != nullptr)
//        delete tmp_pre_integration;
//
//    tmp_pre_integration = nullptr;
}

void Ceres_IMUScan_Matcher::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if(!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (pre_integrations == nullptr)
    {
        pre_integrations = new IntegrationBase{acc_0, gyr_0, Bas_c, Bgs_c};
    }

    // init: frame_count = 0. 1st step: processImage frame_count++ ( = 1).
    // 2nd step: pre_integration[1] store transform data from last_frame_count(0) to current (1).
    if (frame_count != 0)
    {

        pre_integrations->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        //tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        dt_buf.push_back(dt);
        linear_acceleration_buf.push_back(linear_acceleration);
        angular_velocity_buf.push_back(angular_velocity);
        //int j = frame_count;
        // use current data to get states representing last frame to current frame.
        //before initialStructure, these states are meaningless.
        // and initialStructure() -> visualInitialAlign() will give states their init value.
        //TODO: consider use velocity_external to obtain initial pose and vel each loop.
        Vector3d un_acc_0 = Qs_c * (acc_0 - Bas_c) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs_c;
        Qs_c *= Utility::deltaQ(un_gyr * dt);
        Qs_c.normalize();
        Vector3d un_acc_1 = Qs_c * (linear_acceleration - Bas_c) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        LOG(WARNING)<< "acc to EN "<<un_acc;
        Ps_c += dt * Vs_c + 0.5 * dt * dt * un_acc;
        Vs_c += dt * un_acc;
        //logIMUdata();


    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Ceres_IMUScan_Matcher::prepareIMU(const std::deque<sensor::ImuData>& imudata, ::ivcommon::Time pointcloudTime)
{
    CHECK_GT(pointcloudTime, current_time_);
    Eigen::Vector3d linear_acceleration, angular_velocity;
    P_last_ = Ps_c;
    Q_last_ = Qs_c;
    for(const auto& it : imudata)
    {
        auto t_imu = it.time;
        if(t_imu <= pointcloudTime)
        {
            double dt = ::ivcommon::ToSeconds(it.time - current_time_);
            if(dt <= 0)
            {
                continue;
            }

            current_time_ = t_imu;
            linear_acceleration = it.linear_acceleration;
            angular_velocity = it.angular_velocity;
            processIMU(dt, linear_acceleration, angular_velocity);
        }
        else
        {
            double dt_1 = ::ivcommon::ToSeconds(pointcloudTime - current_time_);
            double dt_2 = ::ivcommon::ToSeconds(t_imu - pointcloudTime);
            CHECK_GT(dt_1, 0);
            CHECK_GT(dt_2, 0);
            current_time_ = pointcloudTime;
            linear_acceleration = dt_2 / (dt_1 + dt_2) * linear_acceleration + dt_1 / (dt_1 + dt_2) * it.linear_acceleration;
            angular_velocity = dt_2 / (dt_1 + dt_2) * angular_velocity + dt_1 / (dt_1 + dt_2) * it.angular_velocity;
            processIMU(dt_1, linear_acceleration, angular_velocity);
            break;
        }
    }
}

bool Ceres_IMUScan_Matcher::check_degraded_movement()
{

    CHECK_GT(frame_count, 0);
    return (Ps_c - Ps_l).norm() < 0.1;

}

void Ceres_IMUScan_Matcher::processPointCloud(const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap)
{
    if(frame_count == 0)
        feature_pointcloud_submap_buf = {*point_clouds_ptr_and_submap.first, point_clouds_ptr_and_submap.second};
    //TODO:: use low resolution feature hybrid grid to check.
    //marginalization_flag = check_degraded_movement() ? MARGIN_SECOND_NEW : MARGIN_OLD;
    frame_count ++;

    solveodometry(point_clouds_ptr_and_submap);

    if(frame_count == WINDOW_SIZE)
    {
        slidewindow(point_clouds_ptr_and_submap);
        frame_count = 1;
    }

}

void Ceres_IMUScan_Matcher::solveodometry( const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap)
{
    ceres::Problem problem;
    //Add Parameter blocks
    LOG(WARNING)<<"before optimize state is : "<<Ps_c<<"\n"<<Vs_c<<"\n"<<Bas_c<<"\n"<<Bgs_c;
    LOG(WARNING)<<"before euler angle : "<<::ivcommon::transform::toRollPitchYaw(Qs_c) * 180 /M_PI;

    ceres::LocalParameterization *local_parameterization_current = new ceres::EigenQuaternionParameterization();
    problem.AddParameterBlock(Qs_c.coeffs().data(), 4, local_parameterization_current);
    problem.AddParameterBlock(Ps_c.data(), 3);
    problem.AddParameterBlock(Vs_c.data(), 3);
    problem.AddParameterBlock(Bas_c.data(), 3);
    problem.AddParameterBlock(Bgs_c.data(), 3);
//    ceres::LocalParameterization *local_parameterization_last = new ceres::EigenQuaternionParameterization();
//    problem.AddParameterBlock(Qs_l.coeffs().data(), 4, local_parameterization_last);
//    problem.AddParameterBlock(Ps_l.data(), 3);
//    problem.AddParameterBlock(Vs_l.data(), 3);
//    problem.AddParameterBlock(Bas_l.data(), 3);
//    problem.AddParameterBlock(Bgs_l.data(), 3);

    //Add feature distance error with reference to Feature Map.
    const sensor::PointCloud& point_cloud = *point_clouds_ptr_and_submap.first;
    if(point_cloud.size() < 10 )
    {
        LOG(WARNING)<<"few feature points detected in frame : " << frame_count;
        return;
    }
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<FeatureSpaceCostFunctor,
                    ceres::DYNAMIC, 3, 4>(
                    new FeatureSpaceCostFunctor(
                            10 * options_.feature_space_weight(0) /
                            std::sqrt(static_cast<double>(point_cloud.size())),
                            point_cloud, point_clouds_ptr_and_submap.second ->feature_hybrid_grid(),
                            {::ivcommon::transform::posestamped(),ivcommon::transform::posestamped()},
                            true,
                            point_clouds_ptr_and_submap.second),
                    point_cloud.size()),
            nullptr, Ps_c.data(), Qs_c.coeffs().data());


    //if( frame_count == WINDOW_SIZE )
    {
        //Add IMU measurements error
        int confirmed_voxel_num = point_clouds_ptr_and_submap.second->feature_hybrid_grid().get_confirmed_voxel_number();
        if(confirmed_voxel_num < 1e3)
        {
            LOG(WARNING)<<"few confirmed voxels in submap, the number of voxel is: " << confirmed_voxel_num;
            Vs_c = Qs_c * linear_vel_extern_;

        }
        else if(pre_integrations->sum_dt <10)
        {
            problem.AddResidualBlock(new ceres::NumericDiffCostFunction<IMUCostFunctor, ceres::CENTRAL, 15, 3, 4, 3, 3, 3>(
                    new IMUCostFunctor(pre_integrations, Ps_l, Qs_l, Vs_l, Bas_l, Bgs_l))
                    , nullptr, Ps_c.data(), Qs_c.coeffs().data(), Vs_c.data(), Bas_c.data(), Bgs_c.data());
        }
    }



    //TODO: Add previous pose limitation
//    for(int i = 0; i < frame_count + 1; i++)
//    {
//        problem.AddResidualBlock(
//                new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 3, 3>(
//                        new TranslationDeltaCostFunctor(6,
//                                                        ivcommon::transform::Rigid3d(Ps[i], Eigen::Quaterniond::Identity()))),
//                nullptr, Ps[i].data());
//        problem.AddResidualBlock(
//                new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
//                        new RotationDeltaCostFunctor(2e2,
//                                                     Qs[i])),
//                nullptr, Qs[i].coeffs().data());
//    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_solver_options_, &problem,&summary);
    solve_cost_ = summary.final_cost;
//    for(int i = 0; i < frame_count + 1; i++)
    {
//        LOG(WARNING)<<" Ps["<<i<<"] is \n"<<Ps[i];
//        LOG(WARNING)<<" Vs["<<i<<"] is \n"<<Vs[i];
//        LOG(WARNING)<<" Qs["<<i<<"] is \n"<<Qs[i].coeffs();
        LOG(WARNING)<<"after optimize state is : "<<Ps_c<<"\n"<<Vs_c<<"\n"<<Bas_c<<"\n"<<Bgs_c;
        LOG(WARNING)<<"after euler angle : "<<::ivcommon::transform::toRollPitchYaw(Qs_c) * 180 /M_PI;
    }


}

void Ceres_IMUScan_Matcher::slidewindow(const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap)
{
    CHECK_EQ(frame_count, WINDOW_SIZE);

    Qs_l.coeffs() = Qs_c.coeffs();


    dt_buf.clear();
    linear_acceleration_buf.clear();
    angular_velocity_buf.clear();
    feature_pointcloud_submap_buf = {*point_clouds_ptr_and_submap.first, point_clouds_ptr_and_submap.second};

    Ps_l = Ps_c;
    Vs_l = Vs_c;
    Bas_l = Bas_c;
    Bgs_l = Bgs_c;

    // origin value of states[WINDOW_SIZE] start at states[WINDOW_SIZE - 1] ,which is the latest state.
    delete pre_integrations;
    pre_integrations = new IntegrationBase{acc_0, gyr_0, Bas_c, Bgs_c};

//        for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
//        {
//            double tmp_dt = dt_buf[frame_count][i];
//            Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
//            Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
//
//            pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
//
//            dt_buf[frame_count - 1].push_back(tmp_dt);
//            linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
//            angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
//        }
//
//        //states of Frame[WINDOW_SIZE], as the states of new frame in the next loop, has located in the latest state.
//        Ps[frame_count - 1] = Ps[frame_count];
//        Vs[frame_count - 1] = Vs[frame_count];
//        Qs[frame_count - 1] = Qs[frame_count];
//        Bas[frame_count - 1] = Bas[frame_count];
//        Bgs[frame_count - 1] = Bgs[frame_count];
//        feature_pointcloud_submap_buf[frame_count - 1] = feature_pointcloud_submap_buf[frame_count];
//
//        delete pre_integrations[WINDOW_SIZE];
//        pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
//
//        dt_buf[WINDOW_SIZE].clear();
//        linear_acceleration_buf[WINDOW_SIZE].clear();
//        angular_velocity_buf[WINDOW_SIZE].clear();
//        LOG(WARNING)<<"slide window SECOND NEW   ";

}

void Ceres_IMUScan_Matcher::Match(const ivcommon::transform::posestamped& initial_pose_estimate,
			     const Eigen::Vector3d& linear_vel,
			     const ivcommon::transform::posestamped& last_pose,
                             const std::deque<sensor::ImuData>& imudata,
			     const PointCloudPtrAndSubmap& point_clouds_ptr_and_submap,
                             ivcommon::transform::Rigid3d* const pose_estimate,
                             double* const cost) {
    static int match_counter = 0;

    linear_vel_extern_ = linear_vel;
    prepareIMU(imudata, initial_pose_estimate.time);
    processPointCloud(point_clouds_ptr_and_submap);
    *cost = solve_cost_;
    auto pose_estimate_global = ivcommon::transform::Rigid3d(Ps_c, Qs_c);
    *pose_estimate = point_clouds_ptr_and_submap.second->local_pose().inverse() * pose_estimate_global;
    //*pose_estimate =  pose_estimate_global;
    match_counter++;
}

void Ceres_IMUScan_Matcher::logIMUdata()
{
    string dir("logdir");
    static std::ofstream posefile(dir.c_str());

    posefile<<std::fixed<<std::setprecision(3)
            <<" "<<Ps_c[0]
            <<" "<<Ps_c[1]
            <<" "<<Ps_c[2]
            <<std::endl;
}

}  // namespace scan_matching
}  // namespace mapping_3d

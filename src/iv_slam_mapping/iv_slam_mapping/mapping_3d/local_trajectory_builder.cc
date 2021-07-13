#include "iv_slam_mapping/mapping_3d/local_trajectory_builder.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "iv_slam_mapping/mapping_3d/proto/submaps_options.pb.h"
#include "glog/logging.h"
namespace iv_slam_mapping
{
  namespace mapping_3d
  {

    LocalTrajectoryBuilder::LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options, const ::ros::NodeHandle& local_trajectory_builder_nh)
      : options_(options), active_submaps_(options.submaps_options(), local_trajectory_builder_nh),
      accumulated_range_data_{ Eigen::Vector3f::Zero(), {}, {} } {
      last_mapping_mode = -1;                                                                     ///参数初始化
      active_submap_initial_index = 0;                                                            ///参数初始化
      active_submaps_.activemap_constant.use_gps_location_module = options.use_location_module(); ///参数初始化
      processdynamicobjectdata_running = false;                                                   ///参数初始化
    }

    LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}
    ///
    ///滤除动态障碍物
    ///
    void LocalTrajectoryBuilder::ProcessDynamicObjectData(::ivcommon::Time time, const sensor::Data::DynamicObject dynamic_objects) {
      ::ivcommon::MutexLocker lock(&high_resolution_grid_mutex);
      if (processdynamicobjectdata_running == true)       {
        return;
      }
      processdynamicobjectdata_running = true;
      int expand_intesity = 0;
      for (auto& submap : active_submaps_.submaps_)       {
        auto& high_resolution_grid = submap->high_resolution_hybrid_grid_;

        if (dynamic_objects.target_num <= 0)         {
          LOG(INFO) << "dynamic_objects.target_num:" << dynamic_objects.target_num;
          processdynamicobjectdata_running = false;
          return;
        }

        for (int i = 0; i < dynamic_objects.target_num; i++)         {
          auto& tem_movingtarget = dynamic_objects.moving_target.at(i); //todo

          if (tem_movingtarget.history_num > 2)           {
            for (int j = 0; j < tem_movingtarget.history_num; j++)             {
              auto& one_history_traj = tem_movingtarget.history_traj.at(j);
              std::vector<Eigen::Array3i> object_rectangle;
              std::vector<Eigen::Array3i> object_rectangle_in_low_grid;

              for (int k = 0; k < 4; k++)               {

                Eigen::Vector3d tem_local_point = submap->local_pose().inverse() * (active_submaps_.activemap_constant.origin_position_pose.inverse() *
                  Eigen::Vector3d(one_history_traj.points[k].x(), one_history_traj.points[k].y(), (active_submaps_.activemap_constant.origin_position_pose * submap->local_pose()).translation().z()));
                object_rectangle.push_back(high_resolution_grid.GetCellIndex(tem_local_point.cast<float>()));
              }

              int min_x = std::min<int>(object_rectangle[0].x(),
                std::min<int>(object_rectangle[1].x(), std::min<int>(object_rectangle[2].x(), object_rectangle[3].x())));
              int min_y = std::min<int>(object_rectangle[0].y(),
                std::min<int>(object_rectangle[1].y(), std::min<int>(object_rectangle[2].y(), object_rectangle[3].y())));
              int max_x = std::max<int>(object_rectangle[0].x(),
                std::max<int>(object_rectangle[1].x(), std::max<int>(object_rectangle[2].x(), object_rectangle[3].x())));
              int max_y = std::max<int>(object_rectangle[0].y(),
                std::max<int>(object_rectangle[1].y(), std::max<int>(object_rectangle[2].y(), object_rectangle[3].y())));

              int min_x_in_low_grid = std::min<int>(object_rectangle_in_low_grid[0].x(),
                std::min<int>(object_rectangle_in_low_grid[1].x(), std::min<int>(object_rectangle_in_low_grid[2].x(), object_rectangle_in_low_grid[3].x())));
              int min_y_in_low_grid = std::min<int>(object_rectangle_in_low_grid[0].y(),
                std::min<int>(object_rectangle_in_low_grid[1].y(), std::min<int>(object_rectangle_in_low_grid[2].y(), object_rectangle_in_low_grid[3].y())));
              int max_x_in_low_grid = std::max<int>(object_rectangle_in_low_grid[0].x(),
                std::max<int>(object_rectangle_in_low_grid[1].x(), std::max<int>(object_rectangle_in_low_grid[2].x(), object_rectangle_in_low_grid[3].x())));
              int max_y_in_low_grid = std::max<int>(object_rectangle_in_low_grid[0].y(),
                std::max<int>(object_rectangle_in_low_grid[1].y(), std::max<int>(object_rectangle_in_low_grid[2].y(), object_rectangle_in_low_grid[3].y())));

              for (int i = -3 / high_resolution_grid.resolution(); i <= 5 / high_resolution_grid.resolution(); i++)               {
                for (int j = min_x - expand_intesity; j <= max_x + expand_intesity; j++)                 {
                  for (int k = min_y - expand_intesity; k <= max_y + expand_intesity; k++)                   {
                    float tem_probability = high_resolution_grid.GetProbability(Eigen::Array3i(j, k, i));
                    if (tem_probability > 0.5)                     {
                      if (0 /*i>-1&&i<1*/)                       {
                        high_resolution_grid.SetProbability(Eigen::Array3i(j, k, i), 0.8);
                      }
                      else                       {
                        high_resolution_grid.SetProbability(Eigen::Array3i(j, k, i), 0.4);
                      }
                    }
                  }
                }
              }
            }
          }
          else if (tem_movingtarget.history_num > 0)           {
            auto& one_history_traj = tem_movingtarget.history_traj.at(0);
            // 	sensor::Data::DynamicObject::MovingTarget::HistoryTraj tem_one_history_traj;//todo replace  tem_one_history_traj with one_history_traj
            std::vector<Eigen::Array3i> object_rectangle;
            std::vector<Eigen::Array3i> object_rectangle_in_low_grid;

            for (int k = 0; k < 4; k++)             {

              Eigen::Vector3d tem_local_point = submap->local_pose().inverse() * (active_submaps_.activemap_constant.origin_position_pose.inverse() *
                Eigen::Vector3d(one_history_traj.points[k].x(), one_history_traj.points[k].y(), (active_submaps_.activemap_constant.origin_position_pose * submap->local_pose()).translation().z()));

              object_rectangle.push_back(high_resolution_grid.GetCellIndex(tem_local_point.cast<float>()));
            }

            int min_x = std::min<int>(object_rectangle[0].x(),
              std::min<int>(object_rectangle[1].x(), std::min<int>(object_rectangle[2].x(), object_rectangle[3].x())));
            int min_y = std::min<int>(object_rectangle[0].y(),
              std::min<int>(object_rectangle[1].y(), std::min<int>(object_rectangle[2].y(), object_rectangle[3].y())));
            int max_x = std::max<int>(object_rectangle[0].x(),
              std::max<int>(object_rectangle[1].x(), std::max<int>(object_rectangle[2].x(), object_rectangle[3].x())));
            int max_y = std::max<int>(object_rectangle[0].y(),
              std::max<int>(object_rectangle[1].y(), std::max<int>(object_rectangle[2].y(), object_rectangle[3].y())));

            int min_x_in_low_grid = std::min<int>(object_rectangle_in_low_grid[0].x(),
              std::min<int>(object_rectangle_in_low_grid[1].x(), std::min<int>(object_rectangle_in_low_grid[2].x(), object_rectangle_in_low_grid[3].x())));
            int min_y_in_low_grid = std::min<int>(object_rectangle_in_low_grid[0].y(),
              std::min<int>(object_rectangle_in_low_grid[1].y(), std::min<int>(object_rectangle_in_low_grid[2].y(), object_rectangle_in_low_grid[3].y())));
            int max_x_in_low_grid = std::max<int>(object_rectangle_in_low_grid[0].x(),
              std::max<int>(object_rectangle_in_low_grid[1].x(), std::max<int>(object_rectangle_in_low_grid[2].x(), object_rectangle_in_low_grid[3].x())));
            int max_y_in_low_grid = std::max<int>(object_rectangle_in_low_grid[0].y(),
              std::max<int>(object_rectangle_in_low_grid[1].y(), std::max<int>(object_rectangle_in_low_grid[2].y(), object_rectangle_in_low_grid[3].y())));

            for (int i = -3 / high_resolution_grid.resolution(); i <= 5 / high_resolution_grid.resolution(); i++)             {
              for (int j = min_x - expand_intesity; j <= max_x + expand_intesity; j++)               {
                for (int k = min_y - expand_intesity; k <= max_y + expand_intesity; k++)                 {
                  float tem_probability = high_resolution_grid.GetProbability(Eigen::Array3i(j, k, i));
                  if (tem_probability > 0.5)                   {
                    if (0 /*i>-1&&i<1*/)                     {
                      high_resolution_grid.SetProbability(Eigen::Array3i(j, k, i), 0.8);
                    }
                    else                     {
                      high_resolution_grid.SetProbability(Eigen::Array3i(j, k, i), 0.4);
                    }
                  }
                }
              }
            }
          }
        }
      }
      processdynamicobjectdata_running = false;
      //     LOG(INFO)<<"!!!!!!!!!!!!!dynamic_objects.target_num"<<dynamic_objects.target_num;
    }

    ///
    ///数据预处理，距离滤波，滤去车身周围的点，太近的点以及太远的点，因为其噪声比较多
    ///
    std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
      LocalTrajectoryBuilder::AddRangefinderData(const ::ivcommon::Time time,
        const Eigen::Vector3f& origin, const sensor::PointCloud& ranges)
    {
      accumulated_range_data_ = sensor::RangeData{ Eigen::Vector3f::Zero(), {}, {} };                 ///数据初始化
      const sensor::RangeData range_data_in_first_tracking = sensor::RangeData{ origin, ranges, {} }; ///数据初始化
      for (const Eigen::Vector3f& hit : range_data_in_first_tracking.returns)       {
        const Eigen::Vector3f delta = hit - range_data_in_first_tracking.origin;
        if (std::fabs(delta[0]) < 1.5 && std::fabs(delta[1]) < 5.0)         {
          continue;
        } ///滤去车身周围的点
        const float range = delta.norm();
        ///
        ///滤去车身周围的点，太近的点以及太远的点，因为其噪声比较多
        ///
        if (range >= options_.min_range())         {
          if (range <= options_.max_range())           {
            accumulated_range_data_.returns.push_back(hit);
          }
        }
      }

      return AddAccumulatedRangeData(time, accumulated_range_data_); //add the accumulatedrangedata at the current pose
    }
    ///
    ///点云稀疏化以及获取点云位姿
    ///
    std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
      LocalTrajectoryBuilder::AddAccumulatedRangeData(const ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking)
    {

      const sensor::RangeData filtered_range_data = { range_data_in_tracking.origin,
                                                     sensor::VoxelFiltered(range_data_in_tracking.returns, options_.voxel_filter_size()),
                                                     {} }; ///点云稀疏化

      ///
      ///若点云为空则返回
      ///
      if (filtered_range_data.returns.empty())       {
        LOG(WARNING) << "Dropped empty range data.";
        return nullptr;
      }

      auto odometry_data_ptr = &odometry_data_; ///获取激光雷达里程计位姿

      ///
      ///若激光雷达里程计位姿为空，则发出警告并返回
      ///
      if (odometry_data_ptr->empty())       {
        LOG(WARNING) << "the LidarOdometry_data is empty! It`s waiting for the LidarOdometry result!";
        return nullptr;
      }
      ///
      ///地图初始化
      ///
      if (active_submap_initial_index == 0)       {
        active_submaps_.activemap_constant.origin_position_pose = odometry_data_ptr->front().pose;
      }
      const ::ivcommon::transform::Rigid3d pose_estimate = active_submaps_.activemap_constant.origin_position_pose.inverse() * odometry_data_ptr->front().pose; //lzz?

      return InsertIntoSubmap(time, filtered_range_data, pose_estimate);
    }
    ///
    ///添加激光雷达里程计位姿
    ///
    void LocalTrajectoryBuilder::AddOdometerData(const ::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& odometer_pose)
    {
      ///
      ///滤除无效信息
      ///
      if (odometer_pose.translation().y() == 0 || odometer_pose.translation().z() == 0)
        LOG(ERROR) << "Lidar odometry received error, odometer_pose:" << odometer_pose;
      if (odometry_data_.empty() || time >= odometry_data_.back().time)
      //  ; ///只加入新的数据
      {
        ::ivcommon::MutexLocker lock(&odometry_data_mutex);
        odometry_data_.push_back(sensor::OdometryData{ time, odometer_pose });
      }
      ///
      ///只保存最新的值
      ///
      if (odometry_data_.size() > 1)       {
        ::ivcommon::MutexLocker lock(&odometry_data_mutex);
        odometry_data_.pop_front();
      }
    }
    ///
    ///添加融合定位信息
    ///
    void LocalTrajectoryBuilder::AddLocationModuleData(::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& location_module_data)
    {
      active_submaps_.activemap_constant.location_module_data.push_back(iv_slam_mapping::sensor::OdometryData{ time, location_module_data });

      while (active_submaps_.activemap_constant.location_module_data.size() > 10)       {
        active_submaps_.activemap_constant.location_module_data.erase(active_submaps_.activemap_constant.location_module_data.begin(),
          active_submaps_.activemap_constant.location_module_data.begin() + 2);
      }
    }
    ///
    ///进行三维概率栅格地图创建以及准可通行区域提取
    ///
    std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
      LocalTrajectoryBuilder::InsertIntoSubmap(const ::ivcommon::Time time, const sensor::RangeData& range_data_in_tracking, const ::ivcommon::transform::Rigid3d& pose_observation)
    {
      ///
      ///建图初始化
      ///若一开始就是先验地图模式，则按自主建图进行初始化
      ///若一开始是在线地图模式，则按激光雷达里程计地图编号进行地图创建
      ///注意：
      ///这里存在两种建图模式，第一种为在线建图模式，在这种模式下，地图创建模块或者说准可通行区域提取模块，
      ///按照激光雷达里程计特征地图的索引号进行建图，以确保准可通行区域的编号与特征地图的编号一致
      ///第二种模式为先验地图模式，在这种模式下，为了保证从先验地图切换到在线地图时地图数据不为空，即有完整的环境障碍信息，
      ///需要在使用先验模式的同时，开启自主建图模式，当从先验模式转换到在线模式时，再将
      ///自主建图时的图直接丢给在线地图即可．
      ///
      if (active_submap_initial_index == 0) {
        if (mapping_mode == 2) {
          active_submaps_.AddSubmap(pose_observation, active_submaps_.autonomousmapping_index); ///自主建图编号为autonomousmapping_index，是负数．
          if (active_submaps_.activemap_constant.use_gps_location_module && active_submaps_.activemap_constant.location_module_data.size() > 0) {
            active_submaps_.submaps_.back()->gps_local_pose = (active_submaps_.activemap_constant.location_module_data.back()).pose;
            if (std::fabs<double>((::ivcommon::ToRos(time) - ::ivcommon::ToRos(active_submaps_.activemap_constant.location_module_data.back().time)).toSec()) > 0.3) {
              LOG(ERROR) << "Location_module_data time is too far from rangedata time!";
            }
          }
          active_submaps_.autonomousmapping_index--;
        }
        else {
          active_submaps_.AddSubmap(pose_observation, current_mapping_indexs[0]);
          if (active_submaps_.activemap_constant.use_gps_location_module && active_submaps_.activemap_constant.location_module_data.size() > 0) {
            active_submaps_.submaps_.back()->gps_local_pose = active_submaps_.activemap_constant.location_module_data.back().pose;
            if (std::fabs<double>((::ivcommon::ToRos(time) - ::ivcommon::ToRos(active_submaps_.activemap_constant.location_module_data.back().time)).toSec()) > 0.3) {
              LOG(ERROR) << "Location_module_data time is too far from rangedata time!";
            }
          }
        }
        active_submap_initial_index++; ///只初始化一次
      }
      ///
      /// Querying the active submaps must be done here before calling
      /// InsertRangeData() since the queried values are valid for next insertion.
      ///
      std::vector<std::shared_ptr<const Submap>> insertion_submaps;
      for (const std::shared_ptr<Submap>& submap : active_submaps_.submaps()) {
        insertion_submaps.push_back(submap);
      }
      ///
      ///进行三维地图创建以及可通行区域提取
      ///
      active_submaps_.InsertRangeData(time, sensor::TransformRangeData(range_data_in_tracking, pose_observation.cast<float>()),
        pose_observation, current_mapping_indexs, mapping_mode);

      ///
      ///根据需要保存当前点云数据，主要服务与全局位姿优化中的闭环检测
      ///
      if (options_.range_data_write()) {
        boost::thread rangedata_write_thread(boost::bind(&LocalTrajectoryBuilder::RangeDataWriter, this, range_data_in_tracking, active_submaps_.inserted_rangedata_index));
        rangedata_write_thread.detach();
        active_submaps_.inserted_rangedata_index++;
      }
      ///
      ///返回地图创建结果
      ///
      return std::unique_ptr<InsertionResult>(new InsertionResult{ time, range_data_in_tracking, pose_observation, std::move(insertion_submaps) });
    }
    ///
    ///根据需要保存当前点云数据，主要服务与全局位姿优化中的闭环检测
    ///
    void LocalTrajectoryBuilder::RangeDataWriter(const iv_slam_mapping::sensor::RangeData& range_data_, int& inserted_rangedata_index_)
    {
      iv_slam_mapping::sensor::CompressedRangeData temCompressedRangeData = sensor::Compress(range_data_);
      sensor::proto::CompressedRangeData tem_CompressedRangeData_proto = sensor::ToProto(temCompressedRangeData);

      clock_t rangedata_write_start = clock();
      std::string submap_file_name = "";
      std::string file_modle_name = "rangedata";
      submap_file_name = ::ivcommon::file_directory_generate(active_submaps_.file_time_name, file_modle_name);
      std::stringstream tem_stringstream;
      tem_stringstream.clear();
      tem_stringstream << inserted_rangedata_index_;
      submap_file_name += tem_stringstream.str();
      submap_file_name += ".proto";
      ::ivcommon::io::ProtoStreamWriter writer(submap_file_name);
      writer.WriteProto(tem_CompressedRangeData_proto);
      clock_t rangedata_write_stop = clock();
      double rangedata_write_elapsed = (double)(rangedata_write_stop - rangedata_write_start) / CLOCKS_PER_SEC;
    }

  } // namespace mapping_3d
} // namespace iv_slam_mapping

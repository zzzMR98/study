//
// Created by zzh on 19-3-5.
//
#include <cmath>

#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include "tf2_ros/transform_listener.h"
#include <glog/logging.h>
#include "velodyne/mytime.h"

#include "ivcommon/common/time.h"
#include "ivcommon/common/blocking_queue.h"
#include <deque>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <util/boostudp/boostudp.h>
#include <util/utm/utm.h>
#include <sstream>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/ECUData.h"
#include "sensor_driver_msgs/InsVelocity.h"
#include "covgrid_slam_msgs/LidarOdometryForMapping.h"
#include "covgrid_slam_msgs/GpsByLidarOdometry.h"
#include "covgrid_slam/mapping/pose_extrapolator.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder_options.h"
#include "covgrid_slam/sensor/data.h"
#include "ivcommon/common/time_conversion.h"
#include "point_types.h"

class Node
{
public:
    Node()
    {
        std::string add_submap_dir;
        ros::param::get("~add_submap_dir", add_submap_dir);
        std::string tempstr;
        //将字符串读到input中
        std::stringstream input(add_submap_dir);
        //依次输出到result中，并存入res中
        while(input>>tempstr)
            submap_dirs_.push_back(tempstr);
        LOG(INFO)<<submap_dirs_.size()<<" submap directories detected";
        std::string configuration_directory,configuration_basename;
        ros::param::get("~configuration_directory",configuration_directory);
        ros::param::get("~configuration_basename",configuration_basename);

        options_ = mapping3d::LoadOptions(configuration_directory,configuration_basename);//Lua : save true read false. to make new save dir
        options_.mutable_submaps_options()->set_update_common_map(false);
        options_.mutable_submaps_options()->set_savemap_flag(false);
        options_.mutable_submaps_options()->set_readmap_flag(true);// load submaplists
        options_.mutable_submaps_options()->set_update_flag(false);
        options_.mutable_submaps_options()->set_automode_flag(false);
        options_.mutable_submaps_options()->set_readmap_dir(submap_dirs_.back());
        submap_dirs_.pop_back();
//        LOG(WARNING)<<"option's savemap flag is "<<options_.mutable_submaps_options()->savemap_flag();
//        LOG(WARNING)<<"option's readmap flag is "<<options_.mutable_submaps_options()->readmap_flag();

        auto init_submapManager_ = std::make_shared<mapping3d::SubmapManager>(*options_.mutable_submaps_options(), mapping3d::PosewithGps());
        totalsize_ = init_submapManager_->submap_lists().lists.size();
        global_init_pose_ = init_submapManager_->submap_lists().global_pose;
        gpsdata_ = init_submapManager_->submap_lists().gpsdata;

        options_.mutable_submaps_options()->set_savemap_flag(true);
        options_.mutable_submaps_options()->set_readmap_flag(false);// save mode
        mapping3d::PosewithGps tempPosewithGPS;
        tempPosewithGPS = {global_init_pose_, gpsdata_};
        new_submapManager_ = std::make_shared<mapping3d::SubmapManager>(*options_.mutable_submaps_options(), tempPosewithGPS);

        for(auto& it : init_submapManager_->submap_lists().lists)
        {
            auto submap_ptr = init_submapManager_->GetCompleteSubmap(it.first);
            LOG(WARNING)<<" saving submap index "<< submap_ptr->header().index;
            new_submapManager_->SaveSubmap(submap_ptr);
        }
        LOG(WARNING)<<"load init_submaps success ";

        options_.mutable_submaps_options()->set_savemap_flag(false);
        options_.mutable_submaps_options()->set_readmap_flag(true);// set back to load submaplists
    }
    ~Node() = default;

    void AddSubmapList()
    {
        for(const auto& it : submap_dirs_)
        {
            auto addOption = options_.mutable_submaps_options();
            addOption->set_readmap_dir(it);
            auto addSubmapManager = std::make_shared<mapping3d::SubmapManager>(*addOption, mapping3d::PosewithGps());

            for(auto mapitem : addSubmapManager->submap_lists().lists)
            {
                std::shared_ptr<mapping3d::Submap> submap_ptr = addSubmapManager->GetCompleteSubmap(mapitem.first);
                ivcommon::transform::Rigid3d temp_local_pose = global_init_pose_.inverse() * addSubmapManager->global_init_pose().pose *
                                                    submap_ptr->local_pose();
                submap_ptr->SetLocalpose(temp_local_pose);
                mapping::SubmapHeader temp_header = submap_ptr->header();
                temp_header.local_pose = temp_local_pose;
                temp_header.index += totalsize_;
                std::set<int> tempset;
                tempset.swap(temp_header.linkindexs);
                for(auto& temp_linkindex : tempset)
                    temp_header.linkindexs.insert(temp_linkindex + totalsize_);
                tempset.clear();
                submap_ptr->SetHeader(temp_header);
                // submap finish
                LOG(WARNING)<<" saving submap index "<< submap_ptr->header().index;
                new_submapManager_->SaveSubmap(submap_ptr);

            }
            totalsize_ += addSubmapManager->submap_lists().lists.size();

        }
        new_submapManager_->SaveSubmapList();

    }


private:
    mapping3d::proto::LocalTrajectoryBuilderOptions options_;/**< 雷达里程计的相关参数 */
    std::shared_ptr<mapping3d::SubmapManager> new_submapManager_;
    int totalsize_;
    ivcommon::transform::Rigid3d global_init_pose_;
    sensor::GpsInsData gpsdata_;
    std::vector<std::string> submap_dirs_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multimapinteg");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
    google::InstallFailureSignalHandler();

    Node node;
    node.AddSubmapList();

    return 0;
}

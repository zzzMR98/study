#ifndef TRAVERSaBLE_AREA_NODE_CONSTANT_H
#define TRAVERSaBLE_AREA_NODE_CONSTANT_H
#include "string.h"
namespace traversable_area_extraction{
// constexpr char KNagetiveObjectTopicName[] = "negativeOgm";
// // constexpr std::string KPositiveObjectTopicName = "";
// constexpr char KStiffObjectTopicName[] = "stiffwaterogm";
// // constexpr std::string KWaterObjectTopicName = "";
// constexpr char KSlopeObjectTopicName[] = "slope6yOgm";
// constexpr char KUnevenAreaTopicName[] = "road_map";
// constexpr char KPrimaryTraversableAreaTopicName[] = "traversable_area_topic";
// constexpr char KTraversableAreaVehiclePoseTopicName[] = " vehicle_global_pose_topic";
// constexpr char KLidarOdometryTopicName[] = "lidar_odometry_for_mapping";
// constexpr std::string KUnevenAreaTopicName = "";
const std::string KNegativeObjectTopicName = "negativeOgm";
// constexpr std::string KPositiveObjectTopicName = "";
const std::string KStiffObjectTopicName = "stiffwaterogm";
// constexpr std::string KWaterObjectTopicName = "";
const std::string KSlopeObjectTopicName = "slope6yOgm";
const std::string KUnevenAreaTopicName = "road_map";
    const std::string KBackObstacleTopicName = "backOgm";
const std::string KDynamicObjectTopicName = "MovingTarget";
const std::string KPrimaryTraversableAreaTopicName = "traversible_area_topic";
const std::string KTraversableAreaVehiclePoseTopicName = "vehicle_global_pose_topic";
const std::string KLidarOdometryTopicName = "lidar_odometry_for_mapping";

const std::string KFinalTraversableAreaTopicName = "final_traversable_area_topic";
const std::string KFinalTraversableAreaVehiclePoseTopicName = "final_vehicle_global_pose_topic";
const std::string KSingleTraversableAreaTopicName = "single_traversable_area_topic";
const std::string KOptimizedFinalTraversableAreaTopicName = "final_traversable_area_optimized_topic";
const std::string KOptimizedSingleTraversableAreaTopicName = "single_traversable_area_optimized_topic";


constexpr int kInfiniteSubscriberQueueSize = 0;//编译期常量
constexpr double KHitProbability = 0.9;
constexpr double KNullProbability = 0.1;
constexpr int kBufferSize = 2;
constexpr int kObstacletypes = 11;

}//namespace traversable_area_extraction

#endif

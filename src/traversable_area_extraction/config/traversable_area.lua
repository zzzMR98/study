options = {
    integrate_negativeobeject = true,--是否融合正负障碍
    integtate_stiffobeject = false,--是否融合悬崖
    integtate_slopeobeject = false,--是否融合斜坡
    integtate_unevenareaobeject = false,--是否融合非平坦区域
    integtate_mapping_traversable_area = true;--是否融合准通行区域正障碍
    wiping_dynamicobejectflag = false,--是否提出动态障碍物
    map_frame = "global_earth_frame",
    tracking_frame = "vehicle_frame",
    traversable_area_publish_period_sec = 0.1,
    vehicle_width = 2.2;
    known_radius = 30;
    display_on = true;
    wipe_history_obstacle = true,--当前检测到某处无障碍物时是否抹去该位置的历史障碍物信息
    priormap_write = true,--保存先验地图
    load_priormap = false,--读取先验地图
    priormap_file_name_time = "20201219_151012"--会保存在home路径下，然后需要知道那个文件夹的名称
    -- KNegativeObjectTopicName = "negativeOgm";
    -- KStiffObjectTopicName = "stiffwaterogm";
    -- --KWaterObjectTopicName = "";
    -- KSlopeObjectTopicName = "slope6yOgm";
    -- KUnevenAreaTopicName = "road_map";
    --     KBackObstacleTopicName = "backOgm";
    -- KDynamicObjectTopicName = "MovingTarget";
    -- KPrimaryTraversableAreaTopicName = "traversible_area_topic";
    -- KTraversableAreaVehiclePoseTopicName = "vehicle_global_pose_topic";
    -- KLidarOdometryTopicName = "lidar_odometry_for_mapping";
    
    -- KFinalTraversableAreaTopicName = "final_traversable_area_topic";
    -- KFinalTraversableAreaVehiclePoseTopicName = "final_vehicle_global_pose_topic";
    -- KSingleTraversableAreaTopicName = "single_traversable_area_topic";
    -- KOptimizedFinalTraversableAreaTopicName = "final_traversable_area_optimized_topic";
    -- KOptimizedSingleTraversableAreaTopicName = "single_traversable_area_optimized_topic";
    -- kInfiniteSubscriberQueueSize = 0;
    -- KHitProbability = 0.9;
    -- KNullProbability = 0.1;
    -- kBufferSize = 2;
    -- kObstacletypes = 11;
}
return options
   

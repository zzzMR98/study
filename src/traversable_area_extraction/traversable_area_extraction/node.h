#ifndef TRAVERSABLE_AREA_NODE_H
#define TRAVERSABLE_AREA_NODE_H

// system
#include <map>
#include <deque>
#include <mutex>
#include "math.h"

// ROS
#include "ros/ros.h"

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

// Eigen
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// other dependencies
#include "ivcommon/common/time_conversion.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/time.h"
#include "ivcommon/common/blocking_queue.h"
#include "ivcommon/common/file_directory_generation.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "ivcommon/transform/utm/utm.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "covgrid_slam_msgs/LidarOdometryForMapping.h"
#include "iv_slam_ros_msgs/TraversableArea.h"
#include "iv_slam_ros_msgs/PrimarytraversableArea.h"
#include "iv_slam_ros_msgs/Traversablevehiclepose.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "negative_msgs/NegativeOGM.h"
#include "obstacle_msgs/ObstacleOGM.h"
#include "slopeogm_msgs/SlopeOGM.h"
#include "stiff_msgs/stiffwater.h"
#include "uneven_area_msgs/HeightMap.h"
#include "iv_dynamicobject_msgs/moving_target_send.h"

// custom
#include "node_constant.h"
#include "node_options.h"
#include "data_structure.h"
#include "bayes_updator.h"

///地图定义
/** 栅格数值含义：
 0：未知(0-黑色)		1：可通行(100-灰色)		2：障碍物(110-红色)
 3：正障碍(170-白色)	4：负障碍(130-黄色)		5：悬崖(250-蓝色)
 6：水（绿色）		7：正斜坡(70-紫色) 		８：负斜坡(70-紫色)
 8:车辆(170-红色)   9:非平坦区域　0<1值越小约不平坦，０．１５一下为非平摊区域（靛色）
  分辨率：0.2m
  X方向：201个栅格
  Y方向：351个栅格
 车辆位置：100,100*/

namespace traversable_area_extraction {

class Node {
public:
    Node(ros::NodeHandle& nh, TraversableAreaOption& traversable_area_option);
    ~Node();
    void Init();
    void StartSubscriber();

private:
    /**
     * @brief 将上一帧地图转换成当前位姿的准可通行区域坐标系
     * 
     * @param mapdata 当前帧准可通行区域地图 //障碍物地图 zmr
     * 
     */
    void ConvertToNowCoordinationMat(cv::Mat& area_img, const DataType datatype_, const MapHeader& mapdata);//jkj 0728
    /**
     * @brief 将上一帧的准可通行区域地图 traversable_area_data[DataType::KPrimaryTraversableArea] 与
     * 当前帧的准可通行区域地图 mapdata 融合，以更新 traversable_area_data[DataType::KPrimaryTraversableArea]
     * 
     * 将障碍物地图内容转换到准可通行区域图中 zmr
     * 
     * @param mapdata 当前帧准可通行区域地图 //障碍物地图 zmr
     * @param tem_TraversableAreaData 当前帧准可通行区域地图
     */
    void MapDataCovertToTraversableArea(const MapData& mapdata, TraversableAreaData& tem_TraversableAreaData);
    /**
     * @brief 处理准可通行区域消息转化得到的地图数据，将其更新到全局变量
     *  traversable_area_data[DataType::KPrimaryTraversableArea] 里
     *
     * @param mapdata 准可通行区域消息转化得到的 MapData 类型的地图数据
     */
    void TraversableAreaProcessor(const MapData& mapdata);
    /**
     * @brief 处理匹配上位姿的障碍物信息，对于正负障碍，近处不更新
     *
     * @param datatype_ 障碍物类型
     * @param mapdata 障碍物栅格地图
     * @param mapheader 准可通行区域地图的header
     */
    void CommonObjectProcessor(DataType  datatype_, const MapData mapdata,
        const MapHeader& mapheader);
    /**
     * @brief 用于调度各障碍物信息，并舍弃过时数据或无效数据
     *
     * @param datatype_ 障碍物类型
     */
    void CommonObjectDispatch(DataType datatype_);
    /**
     * @brief 将当前帧的障碍物图转换成准可通行区域的位姿
     *
     * @param datatype_ 障碍物类型
     * @param mapheader
     */
    void CommonObjectSwitcher(DataType datatype_, const MapHeader& mapheader);
    /**
     * @brief 根据当前准可通行区域信息更新可通行区域各层尺寸
     *
     * @param datatype_ 障碍物类型
     * @param mapheader
     */
    void MapSizeUpdater(DataType datatype_, const MapHeader& mapheader);//根据当前准可通行区域信息更新可通行区域各层尺寸
    void ProcessDynamicObject(DataType datatype_, traversable_area_extraction::DynamicObject dynamic_object);//处理动态障碍物信息
    void HandleNegativeObjectMessage(
        const negative_msgs::NegativeOGM::ConstPtr& msg);//接收正负障碍物信息
    void HandleBackOgmMessage(const obstacle_msgs::ObstacleOGM::ConstPtr& msg);//接收后雷达障碍物信息
    void HandleSlopeObjectMessage(const slopeogm_msgs::SlopeOGM::ConstPtr& msg);//接收斜坡障碍物信息
    void HandleStiffObjectMessage(const stiff_msgs::stiffwater::ConstPtr& msg);//接收悬崖障碍物信息
    void HandleUnevenAreaObjectMessage(
        const uneven_area_msgs::HeightMap::ConstPtr& msg);//接收道路边沿提取信息
    void HandleLidarOdometryMessage(
        const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr& msg);//接收雷达里程计信息
    void HandlePrimaryTraversableAreaMessage(
        const iv_slam_ros_msgs::PrimarytraversableArea::ConstPtr& msg);//接收准可通行区域信息
    void ConvertFromMsg2Mapdata(const iv_slam_ros_msgs::PrimarytraversableArea::ConstPtr& msg, MapData& tem_map);///<将接收到的各地图消息转换为通用地图格式
    void HandleTraversableAreaVehiclePose(
        const iv_slam_ros_msgs::Traversablevehiclepose::ConstPtr& msg);//接收可通行区域车辆位姿信息
    void HandleDynamicObjectMessage(
        const iv_dynamicobject_msgs::moving_target_send::ConstPtr& msg);///接收动态障碍物信息
    void PublishFinalTraversableArea(const ros::WallTimerEvent& event);///发布最终可通行区域
    void PublishSingleTraversableArea();//发布单帧可通行区域
    void SingleTraversableAreaProcessor(DataType datatype_
        , MapData& fusemap, const MapData& map);///单帧可通行区域处理
    void traversable_area_optimization(cv::Mat& map, float resolution,
        float vehicle_width_, SendDataType senddatatype);///<优化可通行区域，先以车宽半径膨胀，后用多边形填充
    void known_area_extraction(cv::Mat& srcimage, cv::Point center, int radius);
    void  TraversableAreaOptimization(const iv_slam_ros_msgs::TraversableArea& received_traversable_area
        , ::ivcommon::transform::Rigid3d global_vehicle_pose
        , const std::string traversable_area_topic_name
        , cv::Mat &traversable_area_img);//可通行区域优化
    void Visualization_Traversible_Area(const DataType& datatype_);
    void PriorMapProcessor(int priormap_index, MapData& current_primarytraversablearea);///<根据前端感知信息调用先验地图信息并发布
    void LoadPriorMapProfile();///<加载先验地图信息
    void PriorMapWriter();///<保存先验地图
    void PriorMapheadersWriter();///<缓存先验地图头信息
    void visualizeTraversableArea();
    void HandleGPSMsg(const sensor_driver_msgs::GpswithHeading::ConstPtr& gps_msg);
    int PriormapMatchBasedOnLocation(int last_map_idx);
    float poseSquareDistance(double x, double y, std::vector<double> &b);

    ros::NodeHandle nh_;
    std::vector<::ros::Subscriber> subscribers_;
    TraversableAreaOption traversable_area_option_;
    std::map<DataType, std::deque<MapData>> roadblock_data;///<用于可通行区域的各数据缓存 关联容器
    std::map<DataType, std::deque<MapData>> roadblock_data2;///<用于单帧信息处理的各数据缓存
    std::deque<LidarOdometryData> lidar_odometry_data;
    /*std::map<DataType,TraversableAreaData>*/
    TraversableAreaData traversable_area_data[kObstacletypes];
    ::ivcommon::Mutex mutex_lidarodometry;
    ::ivcommon::Mutex mutex_roadblock_data;
    ::ivcommon::Mutex mutex_pending_pub_data;
    ::ivcommon::Mutex mutex_priormap_write;
    ::ivcommon::Mutex mutex_latest_vechicle_pose;
    ::ivcommon::Mutex mutexs_eachtype[kObstacletypes];
    std::vector<::ros::WallTimer> wall_timers_;

    std::map<std::string, ros::Publisher> publishers;
    /*std::map<DataType,TraversableAreaData>*/
    TraversableAreaData pending_pub_traversable_area_data[kObstacletypes];///<用于存储可通行区域各类障碍物信息  kObstacletypes=11

    int thread_num = 0;
    ::ivcommon::Time map_init_time;
    double a = 6378137;
    double e2 = 0.0818192 * 0.0818192; //e的平方
    ::ivcommon::transform::GridZone gps_zone;
    ::ivcommon::transform::Hemisphere hemi;
    LidarOdometryData latest_vechicle_pose;
    string global_frame_str = "global_earth_frame";
    bool traversable_area_inited;
    bool traversable_area_finished;
    bool priormapprocessor_running;
    bool map_size_updating[kObstacletypes];
    bool map_size_switching[kObstacletypes];
    ::ivcommon::transform::Rigid3d global_origin_pose;//这个是第一帧的位姿，也就是之后所有local pose坐标系的参考位姿
    int latest_map_index;
    std::string initial_localtime;
    std::map<int, MapHeader> traversabalmap_headers;//存的都是local pose

    int last_priormapindex;
    int integrate_current_primarymapdata;
    int last_priormapwrite_index;
    PriorMapGlobalvalue priormap_globalvalue;
    bool TraversableAreaProcessor_running;
    bool lidar_odometry_data_observed;
    bool common_processsor_running[kObstacletypes];
    bool use_location_module;
    cv::Mat show_obstacle_img, traversable_area_data_img;
    std::shared_ptr<boost::thread> visualize_thread;
    bool keep_visualize_thread_running;
    cv::Mat final_traversable_area_img, final_dilated_traversable_area_img,
        single_traversable_area_img, single_dilated_traversable_area_img,
        primary_traversable_area_img;
    std::timed_mutex final_traversable_area_img_mtx, final_dilated_traversable_area_img_mtx,
        single_traversable_area_img_mtx, single_dilated_traversable_area_img_mtx,//带超时的互斥量
        primary_traversable_area_img_mtx;
    bool use_gps_to_match_when_lidarodometry_invalid; // 雷达里程计地图匹配失效时依靠GPS进行地图匹配
    std::deque<std::vector<double> > gps_deque;
    std::vector<std::vector<double> > priormap_xy;
};

} //namespace traversabal_area_extraction
#endif

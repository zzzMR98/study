#ifndef TRAVERSIBLE_AREA_EXTRACTION_DATA_H
#define TRAVERSIBLE_AREA_EXTRACTION_DATA_H
#include "common/make_unique.h"
#include "common/time.h"
#include "common/make_unique.h"
#include "common/blocking_queue.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include "iv_slam_ros_msgs/TraversableArea.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cxcore.h>
namespace traversable_area_extraction{

    enum DataType{KPrimaryTraversableArea,KPositive,KNegative,KStiff,KWater,
        KPositiveSlope,KNegativeSlope,KUnevenArea,KLidarOdometry,KBackOGM,KRefinePositive};
    enum SendDataType{KUnknown=0/*未知*/
        ,KPassibility=1/*已知可通行*/
        ,KObstacle=2/*粗糙障碍物（高度差０.５ｍ的正障碍以及其他类型障碍物）*/
        ,KNearUnknown=3/*近距离（３０ｍ内）未知区域*/
        ,KRefineObstacle=4/*精细正障碍物（只有高度差０．３－０．５ｍ障碍物，不包含大于高度差０．５ｍ的障碍物及其他类型障碍物）*/
        ,KWaterObstacle=5/*水障碍*/
    };
    struct CellData{
        int observation_num ;
        double possibility;
        CellData(){
            observation_num = 0;
            // HSH
            // possibility = 0.5;
            possibility = -1;
        }
    };
    struct MapHeader{
        ::ivcommon::Time time;//using Time = UniversalTimeScaleClock::time_point
        int index;
        bool finished;
        double resolution;
        int width;
        int height;
        int pose_index_x;
        int pose_index_y;//应该是在栅格图中车辆的位置
        ::ivcommon::transform::Rigid3d pose;//当前帧里程计位姿 相对位姿 zmr
        bool use_location_module;
        ::ivcommon::transform::Rigid3d location_module_pose;

        MapHeader(){
            time = ::ivcommon::Time::min();
            index = -1;
            resolution = -1;
            finished = false;
            use_location_module = false;
            width = -1;
            height = -1;
            pose_index_x = -1;
            pose_index_y = -1;
            pose = ::ivcommon::transform::Rigid3d::Identity();

            location_module_pose = ::ivcommon::transform::Rigid3d::Identity();
        }
    };

    struct MapData{
        MapHeader header;
        std::vector<int> data;
    };


    struct DynamicObject{
        struct MovingTarget{
            struct HistoryTraj{
                Eigen::Vector3d center_point;
                std::vector<Eigen::Vector3d> points;
            };
            int history_num;
            bool is_updated;
            std::vector<HistoryTraj> history_traj;
        };

        int target_num;
        ::ivcommon::Time time;
        std::vector<MovingTarget> moving_target;
    };

    struct TraversableAreaData
    {
        MapHeader header;
        std::vector<CellData> data;
    };

    struct LidarOdometryData
    {

        ::ivcommon::Time time;
        ::ivcommon::transform::Rigid3d pose;//local pose
        Eigen::Vector3d GPS;//x is the longitude .y is latitude .z is altitude.车辆后轴中心经纬海拔   角度制
        std::vector<int> indexs;//当前地图编号
        short int mode;//地图创建状态
    };
    struct PriorMapGlobalvalue
    {

        IplImage * prior_map  ;//单通道
        IplImage * Display_Image_ ;//三通道
        iv_slam_ros_msgs::TraversableArea traversable_area_msg;
        iv_slam_ros_msgs::TraversableArea last_traversable_area_msg;
        ::ivcommon::transform::Rigid3d current_mappose;//全局
        bool priormapmode_opened;
        MapHeader last_traversable_area_header;
        PriorMapGlobalvalue()
        {
            prior_map = nullptr ;
            Display_Image_  = nullptr;
            priormapmode_opened = false;
            current_mappose = ::ivcommon::transform::Rigid3d::Identity();
        }
    };


}//namespace traversable_area_extraction
#endif

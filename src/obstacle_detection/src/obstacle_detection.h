/*
 * obstacle_detection.h
 *
 *  Created on: 2018年5月4日
 *      Author: zhubc
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/mouse_event.h>
#include "velodyne/data_types.hpp"
#include "velodyne/myhdl_grabber.h"
#include <cmath>

#define MAX_LAYER 64
#define Z_MAX					2.3
#define Z_MIN					-2
#define UNKNOWN					0  //未知
#define PASSABLE				1  //可通行
#define RIGIDNOPASSABLE			3  //正障碍物
#define NEGATIVENOPASSABLE		4  //负障碍物
#define FIRSTNEGATIVENOPASSABLE		5
#define REFINERIGIDNOPASSABLE			10
#define pi 3.1415926
/*!
*\brief 在使用veldyne64线雷达时需要对其角度进行对其及修正，因此要用到极坐标
*\param distance 表示点的距离
*\param index 表示点的位置
*/
struct PolarPointDI{
    double distance;
    int index;
};

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef std::vector<geometry_msgs::Pose> Radarpose;
using namespace pcl::visualization;
using namespace std;

using namespace boost;
/*!
 * \brief 正负障碍检测类
 */
class Obstacle_Detection
{
	public:
	/*!
	 * \brief 构造函数，需要对grabber_H初始化
	 * \param grabber_H 这个是在调用雷达线束排序的时候会被使用到
	 */
	Obstacle_Detection(pcl::VelodyneGrabber& grabber_H)
	: grabber_H_(grabber_H)
	{}
	/*!
	 * \brief 高度差检测
	 * \param pointcloud 输入点云
	 * \param heightdiffpointcloud 输出点云
	 * \param ogm_data 栅格地图
	 * \param resolution 栅格分辨率
	 * \param heightdiffthreshold 高度差阈值
	 * \param countthreshold 栅格中符合特征点的个数要求
	 * \param definition 栅格中符合高度差阈值对应的状态
	 */
	void heightdiffOgmDetection(Cloud& pointcloud,Cloud& heightdiffpointcloud
	        		,OGMData<unsigned char>& ogm_data
					,double resolution,double heightdiffthreshold,int countthreshold=0,int definition=0);

	/*!
    *
    * \brief 将毫米波雷达检测目标转换成栅格地图
    * \param pointcloud 输入毫米波雷达目标
    * \param ogm_data 栅格地图
    *
    */
	void obstacleDetectionForMMWRadar( const Radarpose &pose_xyz,OGMData<unsigned char>& ogm_data );

	/**
	 * @brief 在地面是平面的情况下，将高于lower_height_bound的点都认为是障碍物，同时用hyper_height_bound排除悬空障碍物
	 * 
	 * @param pointcloud 输入点云
	 * @param ogm_data 栅格地图
	 * @param resolution 栅格分辨率
	 * @param lower_height_bound 高度下限阈值
	 * @param hyper_height_bound 高度上限阈值
	 * @param heightdiffthreshold 较小的高度差
	 * @param definition 对应较小高度差的障碍物类别
	 * @param heightdiffhigherthreshold 较大的高度差
	 * @param definition2 对应较大高度差的障碍物类别
	 */
	void obstacleDetectionForFlatFloor(Cloud& pointcloud,OGMData<unsigned char>& ogm_data,
			double resolution,float lower_height_bound,float hyper_height_bound,double heightdiffthreshold,int definition, 
			double heightdiffhigherthreshold,int definition2);
	/*!
	 * \brief 道路边沿检测（越野环境下误检太多，没有用到）
	 * \param pointcloud 输入点云
	 * \param borderpointcloud 输出点云
	 * \param ogm_data 栅格地图
	 * \param LASER_LAYER 点云线数
	 * \param x_offset 雷达在车体坐标系的x
	 * \param y_offset 雷达在车体坐标系的y
	 */
	static void border_detection(const Cloud& pointcloud,Cloud& borderpointcloud,
	        		OGMData<unsigned char>& ogm_data,int LASER_LAYER,double x_offset,double y_offset);
	/*!
	 * \brief 将点云转换成栅格地图
	 * \param pointcloud 输入点云
	 * \param ogm_data 点云对应的栅格地图
	 * \param countthreshold 每个栅格中特征点个数（大于此阈值才认为此栅格具有这种属性）
	 */
	static void cloud2OGMForRsLidar(const Cloud& pointcloud,OGMData<unsigned char>& ogm_data,int countthreshold=5);
	/*!
	 * \brief 将每个栅格中点的个数大于阈值的栅格做好标记（已知区域）
	 * \param pointcloud 输入点云
	 * \param ogm_data 点云对应的栅格地图
	 * \param countthreshold 栅格中点的个数的阈值
	 */
	static void purecloud2OGM(const Cloud& pointcloud,OGMData<unsigned char>& ogm_data,int countthreshold=0);
	/*!
	 * \brief 针对于velodyne32线雷达的负障碍检测
	 * \param pointcloud 输入点云
	 * \param LASER_LAYER 雷达线数
	 */
	void negativeDetection32(Cloud& pointcloud, int LASER_LAYER);
	/*!
	 * \brief 针对于velodyne64线雷达的负障碍检测
	 * \param pointcloud 输入点云
	 */
	void negativeDetection64(Cloud& pointcloud);
	/*!
	 * \brief 针对于速腾聚创侧面安装的16线雷达（左侧）的负障碍检测
	 * \param pointcloud 输入点云
	 */
	void negativeDetectionForLeftRslidar(Cloud& pointcloud);
	/*!
	 * \brief 针对于速腾聚创侧面安装的16线雷达（右侧）的负障碍检测
	 * \param pointcloud 输入点云
	 */
	void negativeDetectionForRightRslidar(Cloud& pointcloud);
	/*!
	 * \brief 对负障碍的预处理（实际使用时未用到，只是调试时用）
	 * \param initialCloud 输入点云
	 * \param outputCloud 输出点云
	 */
	void pretreatForNegative(Cloud& initialCloud, Cloud& outputCloud);
	/*!
	 * \brief 对负障碍检测得到的栅格地图进行图像膨胀处理
	 * \param inputogm 输入栅格
	 * \param outputogm 输出栅格
	 */
	static void ogmDilation(OGMData<unsigned char>& inputogm, OGMData<unsigned char>& outputogm);
	/*!
	 * \brief 将栅格地图显示出来
	 * \param windowname 窗口名称
	 * \param ogmdata 栅格地图
	 */
	static void showOGM(const char* windowname , const OGMData<unsigned char>& ogmdata);
	 pcl::VelodyneGrabber& grabber_H_;/*< 在使用velodyne雷达时需要获取雷达线束的排列关系，需要调用该类*/
	 PolarPointDI polaraxismat_[MAX_LAYER][3601];/*< 点云极坐标*/

};

/*!
* \file lidarodometrymain.cpp
* \brief 雷达里程计入口程序文件
*
*该文件是雷达里程计入口程序文件，主要包括传感器信息的接受、里程计结果的发送、数据的保存等。
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/07/27
*/


#include <cmath>

#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
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
#include <util/playback/iv_data_playback.h>
#include <util/utm/utm.h>
#include <sstream>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "velodyne/HDL32Structure.h"
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
#include "distortioncorrection.hpp"
#define LASER_NUM 64
/// \brief 雷达里程计的入口节点类
///
/// 主要包括雷达里程计定位主类对象的调用，传感器信息的接受、里程计结果的发送、数据的保存等。
class Node{
public:
/// \brief 构造函数
#if LASER_NUM==64
  typedef velodyne_pointcloud::PointXYZIR LPoint;
#elif LASER_NUM==32
  typedef octopus_lidar_util::PPoint LPoint;
#endif

  Node(tf2_ros::Buffer* const tf_buffer):
    systemInited_(false), imu_static_inited_(false), tf_buffer_(tf_buffer)
  {
    string configuration_directory,configuration_basename;
    ros::param::get("~configuration_directory",configuration_directory);
    ros::param::get("~configuration_basename",configuration_basename);

    processthreadfinished_ = false;
    options_ = mapping3d::LoadOptions(configuration_directory,configuration_basename);
//    loadintensitycalibration("/home/jkj/catkin_ws/intensitycalibration.txt");

    ros::ServiceClient configclient = node_handle_.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
    sensor_driver_msgs::startconfig configsrv;

    while(!configclient.call(configsrv))
     {
       ros::Duration(0.01).sleep();
     }

    configstr_ = configsrv.response.configstr;
    log_on_ = false;
    xml_conf_.Parse(configstr_.c_str(),"GetGPSData");
    xml_conf_.GetSystemParam("log_path",log_path_);
    xml_conf_.GetSystemParam("log_on",log_on_);
    xml_conf_.GetSystemParam("system_start_time",log_start_time_);
    Eigen::Vector3d gpspos;
    xml_conf_.GetModuleParam("pos_x",gpspos[0]);
    xml_conf_.GetModuleParam("pos_y",gpspos[1]);
    xml_conf_.GetModuleParam("pos_z",gpspos[2]);
    Eigen::Vector3d gpsyawpitchroll;
    xml_conf_.GetModuleParam("yaw",gpsyawpitchroll[0]);
    xml_conf_.GetModuleParam("pitch",gpsyawpitchroll[1]);
    xml_conf_.GetModuleParam("roll",gpsyawpitchroll[2]);
    Eigen::Quaterniond tempquat = ivcommon::transform::RollPitchYaw(
    	gpsyawpitchroll[2]*M_PI/180,gpsyawpitchroll[1]*M_PI/180,gpsyawpitchroll[0]*M_PI/180);
    Eigen::Quaterniond gpsquat = tempquat.normalized();
//    gpsquat.x() = tempquat.y();
//    gpsquat.y() = -tempquat.x();
    gpstrans_ = ivcommon::transform::Rigid3d(gpspos,gpsquat); //ins to rear


    withgps_ = false;
    bool sendflag = false;
    std::string local_ip = "127.0.0.1";
    int local_port = 9911;
    std::string remote_ip = "127.0.0.1";
    int remote_port = 9912;
    ros::param::get("~sendflag",sendflag);
    from_velodyne_driver_ = false;
    iskitti_ = false;
    withgps_ = false;
    withecu_ = false;
    withinsvelocity_ = false;
    ros::param::get("~from_velodyne_driver",from_velodyne_driver_);
    ros::param::get("~iskitti",iskitti_);
    ros::param::get("~withgps",withgps_);
    ros::param::get("~withecu",withecu_);
    ros::param::get("~withinsvelocity",withinsvelocity_);
    detect_door_ = false;
    if(sendflag)
      {
      ros::param::get("~detect_door",detect_door_);
      ros::param::get("~local_ip",local_ip);
      ros::param::get("~local_port",local_port);
      ros::param::get("~remote_ip",remote_ip);
      ros::param::get("~remote_port",remote_port);
      }
    rosmessagelaunch();

    sendposedata_.open(local_ip,static_cast<unsigned int>(local_port),remote_ip,static_cast<unsigned int>(remote_port));
  }
/// \brief 析构函数

  ~Node()
  {
    processthreadfinished_ = true;
    savecloud_.stopQueue();

    if(showthread_!=nullptr && showthread_->joinable())
    	showthread_->join();
    // LOG(INFO)<<"show thread end";
    if(savethread_!=nullptr && savethread_->joinable())
    	savethread_->join();
    // LOG(INFO)<<"save thread end";
    if(processthread_!=nullptr && processthread_->joinable())
    	processthread_->join();
    delete localmapping_;
  }

  void loadintensitycalibration(std::string filename)
  {
	  std::ifstream file(filename) ;
	  double temp;
	  int i=0;
	  while(file>>intensityaverage[i]>>intensityvar[i])
	  {
		  LOG(INFO)<<"intensityaverage["<<i<<"]"<<intensityaverage[i]
		  <<" "<<"intensityvar["<<i<<"]"<<intensityvar[i];
		  i++;
	  }
	  file.close();
  }
/// \brief 建立ros接受与发送消息
  void rosmessagelaunch()
  {
	if(from_velodyne_driver_)
	{
		subLaserCloud_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2,boost::bind(&Node::laserCloudHandler,this,_1) );
		pubLaserCloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_corrected", 2);
	}
	else
	{
		subLaserCloud_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("lidar_cloud_calibrated", 2,boost::bind(&Node::laserCloudHandler,this,_1) );
	    if(withgps_)
	      subGps_ = node_handle_.subscribe<sensor_driver_msgs::GpswithHeading> ("gpsdata", 50, boost::bind(&Node::gpsHandler,this,_1));
	}

    pubLaserCloudGrid_ = node_handle_.advertise<sensor_msgs::PointCloud2>("laser_cloud_grid", 2);
	pubLaserCloud_undistortion_ = node_handle_.advertise<sensor_msgs::PointCloud2>("lidar_cloud_undistortion", 2);
    if(!options_.without_imu())
    	subImu_ = node_handle_.subscribe<sensor_msgs::Imu> ("imudata", 50, boost::bind(&Node::imuHandler,this,_1));


    pubLaserOdometry_ = node_handle_.advertise<sensor_driver_msgs::OdometrywithGps> ("lidar_odometry_to_earth", 5);
    pubLidarOdometryForMapping_ = node_handle_.advertise<covgrid_slam_msgs::LidarOdometryForMapping> ("lidar_odometry_for_mapping", 5);
    pubGpsByLidarOdometry_ = node_handle_.advertise<covgrid_slam_msgs::GpsByLidarOdometry> ("gps_by_lidar_odometry", 5);

    if(withecu_)
      subEcu_ = node_handle_.subscribe<sensor_driver_msgs::ECUData> ("ecudata", 50, boost::bind(&Node::ecuHandler,this,_1));

    subInsVelocity_ = node_handle_.subscribe<sensor_driver_msgs::InsVelocity> ("insvelocity", 50, boost::bind(&Node::insvelocityHandler,this,_1));
    subFusePose_ = node_handle_.subscribe<sensor_driver_msgs::GpswithHeading> ("sensor_fusion_output", 50, boost::bind(&Node::fuseposeHandler,this,_1));
  }

/// \brief 用gps初始化里程计起点
///
/// 有无gps的判断，有gps消息接收则对起点进行初始化，否则用里程计起点初始化为零
  void initgps()
  {

	  mapping3d::PosewithGps global_init_pose;
	  mapping3d::PosewithGps global_init_pose_only_yaw =
              {::ivcommon::transform::Rigid3d::Rotation(
                      Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()))
                      ,};
	  auto init_time = ::ivcommon::FromRos(ros::Time::now());
    if(withgps_)
      {
    	LOG(INFO)<<"waiting for gpsdata";
	auto gpsdata = gpsdatas_.PopWithTimeout(::ivcommon::FromSeconds(10));

	if(gpsdata==nullptr)
	  {
	    LOG(ERROR)<<"can't catch gps data!";
	    withgps_ = false;
	    global_init_pose = {::ivcommon::transform::Rigid3d::Rotation(
				Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()))
				,} ;

	  }
	else
	  {
	    while(gpsdatas_.Size()>0||gpsdata->position.norm()<0.1)
	      gpsdata = gpsdatas_.Pop();
	    while(gpsdata->position.norm()<0.1)
	      {
		LOG(ERROR)<<"gps data error , total are zero";
		gpsdata = gpsdatas_.Pop();
	      }

	    init_time = gpsdata->time;
	    Eigen::Vector3d translation;
	    translation[0] = gpsdata->position[0];
	    translation[1] = gpsdata->position[1];
	    translation[2] = gpsdata->position[2];
//	    Eigen::Quaterniond temprotation = ivcommon::transform::RollPitchYaw(0,0,gpsdata->heading*M_PI/180);

        Eigen::Quaterniond rotation = ivcommon::transform::RollPitchYaw(gpsdata->roll*M_PI/180,gpsdata->pitch*M_PI/180,gpsdata->heading*M_PI/180);
        auto temprotation = ivcommon::transform::RollPitchYaw(0,0,gpsdata->heading*M_PI/180);
//	    rotation.x() = temprotation.y();
//	    rotation.y() = -temprotation.x();
//	    LOG(INFO)<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.w()<<" ";
	    ivcommon::transform::Rigid3d aftshaft = ivcommon::transform::Rigid3d(translation,rotation) * gpstrans_.inverse();
	    auto after_shaft_only_yaw = ivcommon::transform::Rigid3d(translation,temprotation) * gpstrans_.inverse();
	    sensor::GpsInsData tempgpsdata;
	    tempgpsdata.position[0] = aftshaft.translation()[0];
	    tempgpsdata.position[1] = aftshaft.translation()[1];
	    tempgpsdata.position[2] = aftshaft.translation()[2];
	    tempgpsdata.heading = ivcommon::transform::GetYaw(aftshaft)*180/M_PI;
	    grid_to_geographic(a,e2,zone,hemi,
			       tempgpsdata.position[1],tempgpsdata.position[0] + 500000
			       ,&tempgpsdata.latitude,&tempgpsdata.longitude);
	    tempgpsdata.latitude *= 180/M_PI;
	    tempgpsdata.longitude *= 180/M_PI;
	    tempgpsdata.altitude = aftshaft.translation()[2];
	    global_init_pose = {aftshaft,tempgpsdata};
        global_init_pose_only_yaw = {after_shaft_only_yaw, tempgpsdata};
	  }

      }
    else
      {
    	global_init_pose = {::ivcommon::transform::Rigid3d::Rotation(
					    Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ())),};
      }
	LOG(INFO)<<"construct localmapping_";
    localmapping_ = new mapping3d::LocalTrajectoryBuilder(options_, global_init_pose, init_time, global_init_pose_only_yaw, options_.without_imu());//

    LOG(INFO)<<"finish init gps";
    tftimer_ = node_handle_.createTimer(ros::Duration(0.01), boost::bind(&Node::timertfCallback,this));

  }
  /// \brief 用于发送全局起点位置的tf回调函数
  ///
  /// 用于定时回调，发送里程计起点的tf

  void timertfCallback()
  {
		const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose;

		double latitude,longitude;
	    grid_to_geographic(a,e2,zone,hemi,
	    		globalpose.translation()[1],globalpose.translation()[0] + 500000
			       ,&latitude,&longitude);

		double tx = globalpose.translation()[0];
		double ty = globalpose.translation()[1];
		double tz = globalpose.translation()[2];

		geometry_msgs::Quaternion geoQuat;
		geoQuat.x = globalpose.rotation().x();
		geoQuat.y = globalpose.rotation().y();
		geoQuat.z = globalpose.rotation().z();
		geoQuat.w = globalpose.rotation().w();
		tf::TransformBroadcaster tfBroadcaster;
		tf::StampedTransform laserOdometryTrans;
		laserOdometryTrans.frame_id_ = "global_earth_frame";
		laserOdometryTrans.child_frame_id_ = "global_init_frame";
		laserOdometryTrans.stamp_ = ros::Time::now();
		laserOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
		laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
		tfBroadcaster.sendTransform(laserOdometryTrans);
  }

  /// \brief 用于雷达点云数据接受
  /// \param 点云消息
  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
  //void laserCloudHandler(const CloudConstPtr& laserCloudIn)
  {
	if(systemInited_)
		PointCloudMsgs_.Push(laserCloudMsg);



//    LOG(INFO)<<"recieve cloud";
    LOG(INFO)<<"recievetime:"<<(ros::Time::now() - ros::Time(laserCloudMsg->header.stamp)).toSec();
    if(PointCloudMsgs_.Size()>2)
      PointCloudMsgs_.Pop();

  }

  /// \brief 用于imu数据接受
  /// \param imu数据
  /// 给local_mapping
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
  {
    sensor::ImuData imu_data;
    imu_data.linear_acceleration.x() = imuIn->linear_acceleration.x;
    imu_data.linear_acceleration.y() = imuIn->linear_acceleration.y;
    imu_data.linear_acceleration.z() = imuIn->linear_acceleration.z;

    imu_data.angular_velocity.x() = imuIn->angular_velocity.x;
    imu_data.angular_velocity.y() = imuIn->angular_velocity.y;
    imu_data.angular_velocity.z() = imuIn->angular_velocity.z;

    imu_data.time = ::ivcommon::FromRos(ros::Time(imuIn->header.stamp));

    static Eigen::Vector3d sum_linear_acc = Eigen::Vector3d::Zero();
    static Eigen::Vector3d sum_angular_vel = Eigen::Vector3d::Zero();
    static int count = 0;
    static Eigen::Vector3d Gravity(0,0,9.80);

    if(count < 50)
	{
    	Eigen::Quaterniond tempQ = ivcommon::transform::RollPitchYaw(gpsdata_last_.roll*M_PI/180, gpsdata_last_.pitch*M_PI/180, gpsdata_last_.heading*M_PI/180);
    	auto linear_acc_without_G = imu_data.linear_acceleration - tempQ.inverse() * Gravity;
		sum_linear_acc += linear_acc_without_G;
		sum_angular_vel += imu_data.angular_velocity;
		count++;
	}
	else
	{
//		LOG(WARNING)<< " accelerometer bias is "<< sum_linear_acc / count;
//		LOG(WARING)<< " gyros bias is "<< sum_angular_vel / count;
		imu_static_inited_ = true;
	}



//    if(options_.without_imu())
    {
        imu_data.angular_velocity = gpstrans_.rotation()*(imu_data.angular_velocity - sum_angular_vel / count);
        imu_data.linear_acceleration = gpstrans_.rotation()*(imu_data.linear_acceleration - sum_linear_acc / count);//均进行一个修正，在gpsdata（GPS与惯导融合）的基础上
    }

//    imudatas_.Push(::ivcommon::make_unique<sensor::ImuData>(imu_data));
    if(systemInited_&&!options_.without_imu())
    	localmapping_->AddImuData(imu_data);
  }

  /// \brief 用于惯导输出速度数据接受
  /// \param 惯导速度消息
  /// 给local_mapping
  void insvelocityHandler (const sensor_driver_msgs::InsVelocity::ConstPtr& insvelocityIn)
  {
	    sensor::InsVelocityData insvelocity_data;
	    insvelocity_data.time = ::ivcommon::FromRos(ros::Time(insvelocityIn->header.stamp));
	    insvelocity_data.linear_velocity.x() = insvelocityIn->linear_velocity.x;
	    insvelocity_data.linear_velocity.y() = insvelocityIn->linear_velocity.y;
	    insvelocity_data.linear_velocity.z() = insvelocityIn->linear_velocity.z;

	    insvelocity_data.angular_velocity.x() = insvelocityIn->angular_velocity.x;
	    insvelocity_data.angular_velocity.y() = insvelocityIn->angular_velocity.y;
	    insvelocity_data.angular_velocity.z() = insvelocityIn->angular_velocity.z;
	    {
	    	insvelocity_data.angular_velocity = gpstrans_.rotation()*insvelocity_data.angular_velocity ;
	    	insvelocity_data.linear_velocity = gpstrans_.rotation()*insvelocity_data.linear_velocity ;
	    }
	    if(systemInited_&&withinsvelocity_)
	    	localmapping_->AddInsVelocityData(insvelocity_data);

	    static std::string filename = log_path_ + "/insvelocitydata.txt";
	    static std::ofstream insvelocityfile(filename.c_str());
	    insvelocityfile<<ToRos(insvelocity_data.time).toSec()-log_start_time_
	    <<std::fixed<<std::setprecision(5)
		<<" "<<insvelocity_data.linear_velocity.x()
		<<" "<<insvelocity_data.linear_velocity.y()
		<<" "<<insvelocity_data.linear_velocity.z()
		<<" "<<insvelocity_data.angular_velocity.x()
		<<" "<<insvelocity_data.angular_velocity.y()
		<<" "<<insvelocity_data.angular_velocity.z()
		<<std::endl;
  }
  /// \brief 接受惯导或GPS输出的位姿信息
  /// \param 位姿消息
  void gpsHandler(const sensor_driver_msgs::GpswithHeading::ConstPtr& gpsIn)
  {
    sensor::GpsInsData gps_data;

    gps_data.heading =  gpsIn->heading;
    gps_data.pitch =  gpsIn->pitch;
    gps_data.roll =  gpsIn->roll;
    gps_data.latitude = gpsIn->gps.latitude;
    gps_data.longitude = gpsIn->gps.longitude;
    gps_data.altitude = gpsIn->gps.altitude;
    if(fabs(gps_data.heading)<0.000001&&fabs(gps_data.altitude)<0.000001&&fabs(gps_data.longitude)<0.000001)
    	return;
    geographic_to_grid(a, e2, gps_data.latitude*M_PI/180, gps_data.longitude*M_PI/180
		       , &zone, &hemi, &gps_data.position.y(), &gps_data.position.x());
    gps_data.position.x() -= 500000;
    gps_data.position.z() = gpsIn->gps.altitude;
    gps_data.time = ::ivcommon::FromRos(ros::Time(gpsIn->gps.header.stamp));
    gpsdatas_.Push(::ivcommon::make_unique<sensor::GpsInsData>(gps_data));

    if(systemInited_)
    {
        Eigen::Vector3d rollpitchyaw(gps_data.roll*M_PI/180,gps_data.pitch*M_PI/180,gps_data.heading*M_PI/180);

        auto tempquat = ivcommon::transform::RollPitchYaw(rollpitchyaw[0],rollpitchyaw[1],rollpitchyaw[2]);
        Eigen::Vector3d nowtranslation(gps_data.position.x(),gps_data.position.y(),gps_data.position.z());
        ivcommon::transform::Rigid3d nowpose = ::ivcommon::transform::Rigid3d(nowtranslation,tempquat) * gpstrans_.inverse(); //rear axle pose
        localmapping_->AddInsData({gps_data.time,nowpose});
    }
    if(gpsdatas_.Size()>1)
      gpsdatas_.Pop();
    logGpsData(gps_data);
    gpsdata_last_ = gps_data;
  }

  /// \brief 用于融合定位结果的接受
  /// \param 融合定位结果消息
  void fuseposeHandler(const sensor_driver_msgs::GpswithHeading::ConstPtr& fuseposeIn)
  {
    sensor::GpsInsData fusepose_data;

    fusepose_data.heading =  fuseposeIn->heading;
    fusepose_data.pitch =  fuseposeIn->pitch;
    fusepose_data.roll =  fuseposeIn->roll;
    fusepose_data.latitude = fuseposeIn->gps.latitude;
    fusepose_data.longitude = fuseposeIn->gps.longitude;
    fusepose_data.altitude = fuseposeIn->gps.altitude;
    if(fabs(fusepose_data.heading)<0.000001&&fabs(fusepose_data.altitude)<0.000001&&fabs(fusepose_data.longitude)<0.000001)
    	return;
    geographic_to_grid(a, e2, fusepose_data.latitude*M_PI/180, fusepose_data.longitude*M_PI/180
		       , &zone, &hemi, &fusepose_data.position.y(), &fusepose_data.position.x());
    fusepose_data.position.x() -= 500000;
    fusepose_data.position.z() = fuseposeIn->gps.altitude;
    fusepose_data.time = ::ivcommon::FromRos(ros::Time(fuseposeIn->gps.header.stamp));

    if(systemInited_)
    {
        Eigen::Vector3d rollpitchyaw(fusepose_data.roll*M_PI/180,fusepose_data.pitch*M_PI/180,fusepose_data.heading*M_PI/180);

        auto tempquat = ivcommon::transform::RollPitchYaw(rollpitchyaw[0],rollpitchyaw[1],rollpitchyaw[2]);
        Eigen::Vector3d nowtranslation(fusepose_data.position.x(),fusepose_data.position.y(),fusepose_data.position.z());
        ivcommon::transform::Rigid3d nowpose = ::ivcommon::transform::Rigid3d(nowtranslation,tempquat);//* gpstrans_.inverse()
        localmapping_->AddFusePoseData({fusepose_data.time,nowpose});
        localmapping_->setNeedToReset(fuseposeIn->mode);
    }

    logFusePoseData(fusepose_data);
  }

  /// \brief 用于丰田底层ecu数据的接受
  /// \param ecu数据消息
  void ecuHandler(const sensor_driver_msgs::ECUData::ConstPtr& ecuIn)
  {
    sensor::EcuData ecu_data;
    ecu_data.time= ::ivcommon::FromRos(ros::Time(ecuIn->header.stamp));
    ecu_data.f_shift = ecuIn->f_shift;
    ecu_data.f_shift1 = ecuIn->f_shift1;
    ecu_data.fDeForwardVel = ecuIn->fDeForwardVel;

    ecu_data.fDeForwardVel=ecuIn->fDeForwardVel;
    ecu_data.fFLRWheelAverAngle=ecuIn->fFLRWheelAverAngle;
    ecu_data.fForwardVel=ecuIn->fForwardVel;

    ecu_data.petral_pressure=ecuIn->petral_pressure;
    ecu_data.pressure_back=ecuIn->pressure_back;
    ecu_data.FrontLeftWheelSpeed=ecuIn->FrontLeftWheelSpeed;
    ecu_data.FrontRightWheelSpeed=ecuIn->FrontRightWheelSpeed;
    ecu_data.BackLeftWheelSpeed=ecuIn->BackLeftWheelSpeed;//1
    ecu_data.BackRightWheelSpeed=ecuIn->BackRightWheelSpeed;

//    LOG(INFO)<<"f_shift:"<<int(ecu_data.f_shift);
//	LOG(INFO)<<"fDeForwardVel:"<<ecu_data.fDeForwardVel;
//	LOG(INFO)<<"fFLRWheelAverAngle:"<<ecu_data.fFLRWheelAverAngle;
//	LOG(INFO)<<"fForwardVel:"<<ecu_data.fForwardVel;
    if(systemInited_)
    	localmapping_->AddEcuData(ecu_data);

    static std::string filename = log_path_ + "/ecudata.txt";
    static std::ofstream ecufile(filename.c_str());
    ecufile<<ToRos(ecu_data.time).toSec()-log_start_time_
	<<" "<<int(ecu_data.f_shift)
	<<std::fixed<<std::setprecision(5)
    <<" "<<ecu_data.fFLRWheelAverAngle
	<<" "<<ecu_data.fForwardVel
	<<std::endl;
//    ecudatas_.Push(::ivcommon::make_unique<sensor::EcuData>(ecu_data));
//    if(ecudatas_.Size()>10)
//    	ecudatas_.Pop();
  }

  /// \brief 矫正点云运动畸变
  ///
  /// 矫正点云运动畸变
  /// \param pcl_point_cloud 输入点云
  /// \param linearvelocity 线速度 0 1 2分别对应x y z轴的线速度
  /// \param angularvelocity 角轴向量形式的速度 模代表速度大小，nomalize后代表角轴放心
  /// \return 矫正运动畸变后的点云
  sensor::PointCloud
    distortionCorrection(const sensor::PointCloud& point_cloud,
  		  const Eigen::Vector3d& linearvelocity,const Eigen::Vector3d& angularvelocity, double period)
    {
  	  auto result = point_cloud;

  	  double backtime = period;
      Eigen::Vector3d temp_linearvel = linearvelocity;
      temp_linearvel.x() = 0;
//      temp_linearvel.y() = 0;
      temp_linearvel.z() = 0;
//      auto temp_angularvel = Eigen::Vector3d::Zero();
  	  for(auto& point:result)
  	  {
  		  double time = point.ringandtime - int(point.ringandtime);
  		  double deltat = backtime - time;
  //		  CHECK_GE(deltat,0);

  		  Eigen::Vector3d translation = temp_linearvel * deltat;
  		  Eigen::Vector3d angle = angularvelocity * deltat;

  		  auto rotation = DistortionCorrection::AngleAxisVectorToRotationQuaternion<double>(angle);
  		  point = {(rotation.inverse()*(point - translation)),point.ringandtime,point.intensity};
  	  }

  	  return result;
    }
  /// \brief 该类主程序线程
  ///
  /// 主程序线程，调用雷达里程计相关接口
  void processthread()
  {
    static ::ivcommon::transform::posestamped last_pose;
    initgps();
    systemInited_ = true;
    while(!processthreadfinished_)
      {
    	if(!(imu_static_inited_ || options_.without_imu()))
    		continue;
//		checkgps();

//		LOG(INFO)<<"time diff:"<<::ivcommon::now().time_since_epoch().count()<<"\t"<<::ivcommon::FromRos(ros::Time::now()).time_since_epoch().count();

		sensor_msgs::PointCloud2ConstPtr laserCloudMsg=PointCloudMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.1));
		if(laserCloudMsg==nullptr)
		{
			LOG_EVERY_N(WARNING, 10)<<"can't get lidar data.";
			continue;
		}

		std_msgs::Header msg_header = laserCloudMsg->header;

//		if(from_velodyne_driver_)
//			msg_header.stamp = ros::Time::now();
		::ivcommon::Time this_time = ::ivcommon::FromRos(ros::Time(msg_header.stamp));
		LOG(INFO)<<ros::Time::now()<<"\t"<<msg_header.stamp;
		LOG(INFO)<<"got pointcloud time:"<<::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time);
		if(::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time)>0.1)//保证雷达消息是及时收到的一帧（10hz）
		{
			LOG(WARNING)<<"overtime:"<<::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time);
			continue;
		}
		static int totalnum = 0;
	    if(iskitti_&&totalnum==0)
	    {
	    	std::string posefilename;
	    	ros::param::get("~posefilename",posefilename);
	    	setinitvelocity(posefilename);

	    }
//	    if(totalnum<=1)
//	    	setinitvelocity2("/home/jkj/catkin_ws/velocity.txt");

		sensor::RangeData rangedata;

		vector<Eigen::Vector3d> origins;

		if(from_velodyne_driver_)
		{
			pcl::PointCloud<LPoint> velodyne_point_cloud;
			pcl::fromROSMsg(*laserCloudMsg, velodyne_point_cloud);
			auto trans = LookupToTracking(::ivcommon::FromRos(ros::Time(msg_header.stamp.toSec()))
					,laserCloudMsg->header.frame_id);

#if LASER_NUM==64
			Eigen::Vector3d linearvelocity,angularvelocity;
			localmapping_->getvelocity(linearvelocity, angularvelocity);

//			angularvelocity.x() = 0;
//			angularvelocity.y() = 0;
	//		  angle.z() = 0;
//			linearvelocity.z() = 0;
			//	  linearvelocity[0] = 0;
			//	  linearvelocity[1] = 30;
			//	  linearvelocity[2] = 0;
			linearvelocity = trans->rotation().inverse() * linearvelocity;
			angularvelocity = trans->rotation().inverse() * angularvelocity;
			pcl::PointCloud<LPoint> velodyne_point_cloud_corrected //= velodyne_point_cloud;
											= DistortionCorrection::distortionCorrection64(velodyne_point_cloud,linearvelocity, angularvelocity,scanPeriod);
			pcl::transformPointCloud(velodyne_point_cloud_corrected, velodyne_point_cloud_corrected
					, trans->translation().cast<float>(), trans->rotation().cast<float>());
			sensor_msgs::PointCloud2 cloud_msg;
			pcl::toROSMsg(velodyne_point_cloud_corrected, cloud_msg);

			cloud_msg.header = msg_header;
			cloud_msg.header.stamp = ros::Time(cloud_msg.header.stamp.toSec());
			cloud_msg.header.frame_id = "vehicle_frame";
			pubLaserCloud_.publish(cloud_msg);
			analysisCloud(velodyne_point_cloud_corrected,rangedata.returns,origins);
#elif LASER_NUM==32
			pcl::transformPointCloud(velodyne_point_cloud, velodyne_point_cloud
					, trans->translation().cast<float>(), trans->rotation().cast<float>());
			Eigen::Vector3d linearvelocity,angularvelocity;
			localmapping_->getvelocity(linearvelocity, angularvelocity);

			angularvelocity.x() = 0;
			angularvelocity.y() = 0;
	//		  angle.z() = 0;
			linearvelocity.z() = 0;
			//	  linearvelocity[0] = 0;
			//	  linearvelocity[1] = 30;
			//	  linearvelocity[2] = 0;
			pcl::PointCloud<LPoint> velodyne_point_cloud_corrected
											= DistortionCorrection::distortionCorrection(velodyne_point_cloud,linearvelocity, angularvelocity);

			sensor_msgs::PointCloud2 cloud_msg;
			pcl::toROSMsg(velodyne_point_cloud_corrected, cloud_msg);
			cloud_msg.header = msg_header;
			cloud_msg.header.frame_id = "vehicle_frame";
			pubLaserCloud_.publish(cloud_msg);

			analysisCloud(velodyne_point_cloud_corrected,rangedata.returns,origins);
#endif
			Eigen::Vector3d origin = Eigen::Vector3d::Zero();


//			LOG(WARNING)<<laserCloudMsg->header.frame_id<<" "<<trans->DebugString();
			rangedata.origin = origin;
//			rangedata = sensor::TransformRangeData(rangedata,
//											trans->cast<float>());
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
			pcl::fromROSMsg(*laserCloudMsg, pcl_point_cloud);
			sensor::PointCloud point_cloud;
			analysisCloud(pcl_point_cloud,point_cloud,origins);
			Eigen::Vector3d linearvelocity,angularvelocity;
			localmapping_->getvelocity(linearvelocity, angularvelocity);
			// LOG(ERROR) << "linear velocity : " << linearvelocity;
            // LOG(ERROR) << "angular velocity : " << angularvelocity;
//            linearvelocity = Eigen::Vector3d::Zero();

			rangedata.returns //= point_cloud;
								= distortionCorrection(point_cloud,linearvelocity, angularvelocity,scanPeriod);

			Eigen::Vector3d origin = Eigen::Vector3d::Zero();
			for(const auto& temporigin:origins)
			{
				origin += temporigin;
			}
			origin /=origins.size();
			rangedata.origin = origin;
		}




//		if(detect_door_)//hailiang
//			opendoor_flag_ = LidarProcess::DoorDetection(pcl_point_cloud,5,1.3,3.3,0,5);
		LOG(INFO)<<"start";
		std::unique_ptr<mapping3d::LocalTrajectoryBuilder::InsertionResult>  insertionresult
		= localmapping_->AddRangefinderData(::ivcommon::FromRos(msg_header.stamp),rangedata.origin,rangedata.returns);

		posePostProcess();
		publishGpsByLidarOdometry(msg_header);
		if(insertionresult!=nullptr)
			publishLidarOdometryForMapping(msg_header,insertionresult);
		publishOdometry(msg_header);


		const ivcommon::transform::posestamped pose = localmapping_->pose_estimate();
//		sendPoseTransform(last_pose,pose);
//		sendLidarGps();
//		sendGpsByLidarOdometry();


		if(options_.submaps_options().readmap_flag())
		  lidargpsstatus_ = sensor::LidarGpsStatus::valid;
		else
		  lidargpsstatus_ = sensor::LidarGpsStatus::invalid;
		if(insertionresult!=nullptr)
		  insertionresults_.Push(std::move(insertionresult));
//		else if(options_.submaps_options().readmap_flag())
//		{
//			auto matchstatus = localmapping_->match_status();
//			if(matchstatus.first == mapping3d::MatchStatus::failed)
//				continue;
//		}

        pcl::PointCloud<pcl::PointXYZI>::Ptr undistortion_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        sensor_msgs::PointCloud2 ros_cloud_undistortion;
        for(const auto& point : rangedata.returns)
        {
            pcl::PointXYZI temp_point;
            temp_point.getVector3fMap() = point.cast<float>();
            undistortion_cloud->push_back(temp_point);
        }
        pcl::toROSMsg(*undistortion_cloud, ros_cloud_undistortion);
        ros_cloud_undistortion.header = msg_header;
        pubLaserCloud_undistortion_.publish(ros_cloud_undistortion);

		double costtime = ::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now())-::ivcommon::FromRos(msg_header.stamp));
		static double totaltime = 0;

		static int lostnum = 0;
		static ::ivcommon::Time last_time=this_time;
		totaltime += costtime;

		totalnum++;
		if(::ivcommon::ToSeconds(this_time - last_time)>0.15)
		{
			lostnum += std::max(0,int(::ivcommon::ToSeconds(this_time - last_time)/0.1 - 1+0.5));
		}
		last_time=this_time;
		double averagetime = totaltime / totalnum;
//		LOG(INFO)<<"cost time:"<<costtime;
		LOG(INFO)<<"average time:"<<averagetime;
		LOG(INFO)<<"lost rate:"<<lostnum<<"/"<<lostnum+totalnum;
//		LOG(INFO)<<"end";



		if(log_on_)
		{
		  logData();
//		  savecloud_.Push(::ivcommon::make_unique<std::pair<pcl::PointCloud<pcl::PointXYZI>,ivcommon::transform::posestamped>>(std::make_pair(
//			  pcl_point_cloud,pose
//		  )));
		  //savecloud(pcl_point_cloud,pose);
		}
		  last_pose = pose;
      }
    LOG(INFO)<<"localmapping_->Finish()";
    localmapping_->Finish();

  }

  /// \brief 初始速度设置
  ///
  /// \param filename 文件内存储初始相邻时刻位姿信息，可以推出初始速度
  void setinitvelocity(std::string filename)
  {
	  std::ifstream fs(filename.c_str());
	  Eigen::Matrix3d rotationtemp;
	  Eigen::Vector3d traslationtemp;
      Eigen::Matrix3d convertmatrix;
      convertmatrix<<1,0,0,
    		  	  	 0,1,0,
					 0,0,-1;

	  fs>>rotationtemp(0,0)>>rotationtemp(0,1)>>rotationtemp(0,2)>>traslationtemp[0]>>
			  rotationtemp(1,0)>>rotationtemp(1,1)>>rotationtemp(1,2)>>traslationtemp[1]>>
			  rotationtemp(2,0)>>rotationtemp(2,1)>>rotationtemp(2,2)>>traslationtemp[2];

      Eigen::Quaterniond rotation(convertmatrix*rotationtemp*convertmatrix);
      Eigen::Quaterniond rotation0(rotation.w(),rotation.x(),rotation.z(),rotation.y());
      Eigen::Vector3d traslation0;
      traslation0<<traslationtemp[0],traslationtemp[2],-traslationtemp[1];

	  fs>>rotationtemp(0,0)>>rotationtemp(0,1)>>rotationtemp(0,2)>>traslationtemp[0]>>
			  rotationtemp(1,0)>>rotationtemp(1,1)>>rotationtemp(1,2)>>traslationtemp[1]>>
			  rotationtemp(2,0)>>rotationtemp(2,1)>>rotationtemp(2,2)>>traslationtemp[2];

      rotation = Eigen::Quaterniond(convertmatrix*rotationtemp*convertmatrix);
      Eigen::Quaterniond rotation1(rotation.w(),rotation.x(),rotation.z(),rotation.y());
//      LOG(INFO)<<rotation1.matrix();
      Eigen::Vector3d traslation1;
      double queue_delta = 0.1;
      traslation1<<traslationtemp[0],traslationtemp[2],-traslationtemp[1];
      Eigen::Vector3d linear_velocity =
          (traslation1 - traslation0) / queue_delta;//求线速度和角速度
      Eigen::Vector3d angular_velocity_ =
          ivcommon::transform::RotationQuaternionToAngleAxisVector(
        		  rotation0.inverse() * rotation1) /queue_delta;
      localmapping_->setvelocity(linear_velocity,angular_velocity_);


  }
  /// \brief 初始速度设置
  ///
  /// \param filename 文件内存储初始速度
  void setinitvelocity2(std::string filename)
  {
	  std::ifstream fs(filename.c_str());
	  Eigen::Vector3d traslationtemp;
	  Eigen::Vector3d yawpitchrolltemp;


	  fs>>traslationtemp[0]>>traslationtemp[1]>>traslationtemp[2]>>yawpitchrolltemp[0]>>yawpitchrolltemp[1]>>yawpitchrolltemp[2];

      localmapping_->setvelocity(traslationtemp,yawpitchrolltemp);

  }
  /// \brief 该函数弃用
  void checkgps()
  {
    if(options_.submaps_options().automode_flag()&&
	!options_.submaps_options().readmap_flag()&&gpsdatas_.Size()>0)
      {
	std::unique_ptr<sensor::GpsInsData> gpsdata = gpsdatas_.Pop();
	Eigen::Vector3d nowtranslation;
	nowtranslation[0] =  gpsdata->position[0];
	nowtranslation[1] =  gpsdata->position[1];
	nowtranslation[2] =  gpsdata->position[2];

	::ivcommon::transform::Rigid3d nowpose = ivcommon::transform::Rigid3d(nowtranslation,
	Eigen::AngleAxis<double>(gpsdata->heading * M_PI/180,Eigen::Vector3d::UnitZ())) * gpstrans_.inverse();

//	    LOG(WARNING)<<nowtranslation;
//	    LOG(WARNING)<<localmapping_->map_entrance_global_pose().translation();
//	localmapping_->matchPoseWithSubmap({gpsdata->time,nowpose});

      }
  }
  /// \brief 点云解析
  ///
  /// 将收到的多雷达的点云进行分离，并进行初步处理，转换为雷达里程计使用的点云类型
  /// \param pcl_point_cloud 输入点云
  /// \param point_cloud 输出点云
  /// \param origins 每个雷达中心位置
  void analysisCloud(pcl::PointCloud<pcl::PointXYZI>& pcl_point_cloud
		     ,sensor::PointCloud& point_cloud, vector<Eigen::Vector3d>& origins)
  {

	  point_cloud.reserve(pcl_point_cloud.size());
	if(iskitti_)
	  {
	      origins.push_back(Eigen::Vector3d(0,0,0));
	      for(int index=0;index<pcl_point_cloud.size();index++)
		{
		  sensor::Point tempoint;
		  tempoint[0] = pcl_point_cloud.points[index].x;
		  tempoint[1] = pcl_point_cloud.points[index].y;
		  tempoint[2] = pcl_point_cloud.points[index].z;

		  tempoint.ringandtime = pcl_point_cloud.points[index].intensity+0.09999;

		  point_cloud.push_back(tempoint);
			Eigen::Vector3d origin;
			origin[0] = 0;
			origin[1] = 0;
			origin[2] = 0;
			origins.push_back(origin);
//		  pcl::PointXYZI point;
//
//		  point=pcl_point_cloud.points[index];
//		  point.x = pcl_point_cloud.points[index].x;
//
//		  point.y = pcl_point_cloud.points[index].y;
//		  point.z = pcl_point_cloud.points[index].z;

//		  pcl_point_cloud.points[index].x = point.x;
//		  pcl_point_cloud.points[index].y = point.y;
//		  pcl_point_cloud.points[index].z = point.z;
		}

	  }
	else
	  {
	    int cloudSize = pcl_point_cloud.size();
	    if(cloudSize<64)
	    {
	    	LOG(ERROR)<<"analysis error:"<<"cloudSize<64";
	    	return;
	    }

	    int cloudnum = cloudSize%16;// LIDAR number
	    vector<int> startnum;
	    vector<int> lasernum;
	    int totallasernum = 0; // total line number
	    int N_SCANS = 32; //line number in specific LIDAR

		for(int i=0;i<cloudnum;i++)
		{
			Eigen::Vector3d origin;
			origin[0] = pcl_point_cloud[cloudSize-cloudnum+i].x;
			origin[1] = pcl_point_cloud[cloudSize-cloudnum+i].y;
			origin[2] = pcl_point_cloud[cloudSize-cloudnum+i].z;
			origins.push_back(origin);
			//LOG(ERROR)<<"origin "<<origin;
			int temp = pcl_point_cloud[cloudSize-cloudnum+i].range;//起始点
			LOG(INFO)<<"start point number:"<<temp;
			startnum.push_back(temp);
			int templasernum = pcl_point_cloud[cloudSize-cloudnum+i].azimuth;
			LOG(INFO)<<"lasernum:"<<templasernum;
			lasernum.push_back(templasernum);
			totallasernum += templasernum;
		}
		if(cloudnum>0)
		{
			cloudSize -= cloudnum;
		//pcl_point_cloud.erase(pcl_point_cloud.begin()+cloudSize,pcl_point_cloud.end());
		}
		else
		{
			startnum.push_back(0);
			lasernum.push_back(N_SCANS);
			totallasernum = N_SCANS;
		}

		int offsetscan=0;

		for(int k=0 ;k<startnum.size();k++)
		{
//			if(k!=0)
//				continue;
			N_SCANS = lasernum.at(k);
			int offsetnum = startnum.at(k);//点云初始索引
			offsetscan += k==0?0:lasernum.at(k-1);
			int cloudpointnum = startnum.size()-k==1?cloudSize-startnum.at(k):startnum.at(k+1)-startnum.at(k);//当前点云数量
            LOG(INFO)<<"cloudpointnum :"<<cloudpointnum;
            LOG(INFO)<<"startnum.size() :"<<startnum.size();
			if(offsetnum+cloudpointnum>cloudSize)
			{
				LOG(ERROR)<<"analysis error:"<<"cloudpointnum>cloudSize";
				return;
			}

			if(N_SCANS>64)
			{
				LOG(ERROR)<<"analysis error:"<<"N_SCANS>64";
				return;
			}


			std::map<int,int> indexmap;// LIDAR produce N_SCANS points at a timestamp, this map indecates the relationship
			// betwwen point index and the location of point in N_SCANS
 			bool hasinfo = 0;
			for(int j = 0;j<N_SCANS;j++)
			  {
				CHECK_LT(offsetnum+cloudpointnum-N_SCANS+j,pcl_point_cloud.size())
						<<" "<<offsetnum<<" "<<cloudpointnum<<" "<<N_SCANS<<" "<<j;
				pcl::PointXYZI& temppoint = pcl_point_cloud.points.at(offsetnum+cloudpointnum-N_SCANS+j);
				if(temppoint.range<-0.15)  //存储点云信息的点
				{
					int index = temppoint.x;
					indexmap[index] = j;
					hasinfo = true;
				  	//LOG(ERROR)<<"indexmap"<<index<<":"<<indexmap[index];
				}
				else
				  indexmap[j] = j;
			  }
			if(hasinfo)
			  cloudpointnum -= N_SCANS;
//			LOG(INFO)<<"cloudpointnum="<<cloudpointnum;

			int interval = 1;
			if(N_SCANS==64)
			  interval = 1;
			for (int i = 0; i < (cloudpointnum/N_SCANS); i++)
			{
				for(int laser_j=0;laser_j<N_SCANS;laser_j+=interval)
				{

				  int index_j=indexmap[laser_j];

				  int index=i*N_SCANS+index_j+offsetnum;

//				  if(fabs(pcl_point_cloud.points[index].x)<1.5)
//                        continue;

                                  if(fabs(pcl_point_cloud.points[index].y) < 4 && fabs(pcl_point_cloud.points[index].x) < 2.5)
                        continue;

				  if(pcl_point_cloud.points[index].range<0.5 && pcl_point_cloud.points[index].intensity>- 0.0001)
					  continue;
				  pcl::PointXYZI& point = pcl_point_cloud.points[index];

				  sensor::Point tempoint(point.getVector3fMap().cast<double>(), //laser_j+offsetscan + 0.099999);
						  laser_j+offsetscan + scanPeriod*(i/double(cloudpointnum/N_SCANS)));
//				  float tempy = tempoint.y();
//				  tempoint.y() = -tempoint.x();
//				  tempoint.x() = tempy;
//				  tempoint[0] = point.x;
//				  tempoint[1] = point.y;
//				  tempoint[2] = point.z;
//				  tempoint.intensity = laser_j+offsetscan + 0.1*(i/double(cloudpointnum/N_SCANS));
				  point_cloud.push_back(tempoint);
				}
			}
		}
	  }
//	LOG(INFO)<<"slam point num:"<<point_cloud.size();
  }

  std::unique_ptr<::ivcommon::transform::Rigid3d> LookupToTracking(
      const ::ivcommon::Time time, const string& frame_id) const {
    ::ros::Duration timeout(0.1);
    std::unique_ptr<::ivcommon::transform::Rigid3d> frame_id_to_tracking;
    try {
      const ::ros::Time latest_tf_time =
          tf_buffer_
              ->lookupTransform("vehicle_frame", "velodyne", ::ros::Time(0.),
                                timeout)
              .header.stamp;
      const ::ros::Time requested_time = ToRos(time);
      if (latest_tf_time >= requested_time) {
        // We already have newer data, so we do not wait. Otherwise, we would wait
        // for the full 'timeout' even if we ask for data that is too old.
        timeout = ::ros::Duration(0.);
      }
  //    LOG(INFO)<<latest_tf_time<<" "<<trajectory_options_.tracking_frame<<" "<<frame_id;
      auto trans = tf_buffer_->lookupTransform(
    		  "vehicle_frame", frame_id, requested_time, timeout);
      ivcommon::transform::Rigid3d temp(Eigen::Vector3d(trans.transform.translation.x, trans.transform.translation.y
    		  	  	  	  	  	  	  	  , trans.transform.translation.z),
    		  	  	  	  	  Eigen::Quaterniond(trans.transform.rotation.w, trans.transform.rotation.x
    		  	  	  	  			  , trans.transform.rotation.y,trans.transform.rotation.z));
  //    LOG(INFO)<<temp<<" "<<trans.header.stamp<<" "<<requested_time;
      return ::ivcommon::make_unique<::ivcommon::transform::Rigid3d>(temp);
    }
    catch (const tf2::TransformException& ex) {
      LOG(WARNING) << ex.what();
    }
    return nullptr;
  }

//  /// \brief 求角度的交集范围
//  ///输入输出范围均为 [-pi,pi]
//  /// \param anglesection1 第一段角度范围
//  /// \param anglesection2 第二段角度范围
//  /// \return 角度交集范围
//  std::pair<double,double>  intersection_angle(const std::pair<double,double> anglesection1,const std::pair<double,double> anglesection2)
//  {
//	  bool overpi = (anglesection1.first < anglesection1.second);
//  }



  /// \brief 点云解析
  ///
  /// 将收到的多雷达的点云进行分离，并进行初步处理，转换为雷达里程计使用的点云类型
  /// \param pcl_point_cloud 输入点云
  /// \param point_cloud 输出点云
  /// \param origins 每个雷达中心位置
  void analysisCloud(pcl::PointCloud<LPoint>& pcl_point_cloud
		     ,sensor::PointCloud& point_cloud, vector<Eigen::Vector3d>& origins)
  {
	  point_cloud.reserve(pcl_point_cloud.size());

	  origins.push_back(Eigen::Vector3d(0,0,0));
	  std::vector<pcl::PointCloud<LPoint>> pointcloud_vec;
	  pointcloud_vec.resize(lasernum);
	  for(int index=0;index<pcl_point_cloud.size();index++)
	  {
		  if(fabs(pcl_point_cloud[index].y)<0.0001&&fabs(pcl_point_cloud[index].x)<0.0001)
			  continue;
		  int ring = pcl_point_cloud[index].ring;
		  pointcloud_vec[ring].push_back(pcl_point_cloud[index]);
	  }

//	  int max_size=0;
//	  int max_size_ring = 0;
//	  for(int ring = 0;ring < lasernum;ring++)
//	  {
//		  if(pointcloud_vec[ring].size()<=0)
//			  continue;
//		  if(max_size < pointcloud_vec[ring].size())
//		  {
//			  max_size = pointcloud_vec[ring].size();
//			  max_size_ring = ring;
//		  }
//	  }
//	  double begin_angle = std::atan2(pointcloud_vec[max_size_ring].front().y,pointcloud_vec[max_size_ring].front().x);
//	  double end_angle = std::atan2(pointcloud_vec[max_size_ring].back().y,pointcloud_vec[max_size_ring].back().x);
//	  LOG(INFO)<<"ring:"<<max_size_ring<<"\tpoint size:"<<pointcloud_vec[max_size_ring].size()
//			  <<"\tbegin_angle:"<<begin_angle<<"\tend_angle:"<<end_angle
//			  <<"\tbegin_angle-end_angle:"<<begin_angle-end_angle;

	  for(int j=0;j<lasernum;j++)
	  {
		  for(int index=0;index<pointcloud_vec[j].size();index++)
		{
#if LASER_NUM==64
		  if(pointcloud_vec[j].points[index].y>-3&&pointcloud_vec[j].points[index].y<3
				  &&pointcloud_vec[j].points[index].x<2&&pointcloud_vec[j].points[index].x>-2)
			  continue;
#elif LASER_NUM==32
		  if(pointcloud_vec[j].points[index].y>-3&&pointcloud_vec[j].points[index].y<3
				  &&pointcloud_vec[j].points[index].x<3&&pointcloud_vec[j].points[index].x>-1)
			  continue;

		  if(pointcloud_vec[j].points[index].y<0&&pointcloud_vec[j].points[index].x<0
				  &&std::sqrt(pointcloud_vec[j].points[index].x*pointcloud_vec[j].points[index].x
						  +pointcloud_vec[j].points[index].y*pointcloud_vec[j].points[index].y)<20
				  &&std::sqrt(pointcloud_vec[j].points[index].x*pointcloud_vec[j].points[index].x
										  +pointcloud_vec[j].points[index].y*pointcloud_vec[j].points[index].y)>4
				  &&pointcloud_vec[j].points[index].y/(pointcloud_vec[j].points[index].x)>1)
			  continue;
#endif
		  sensor::Point tempoint;
		  tempoint[0] = pointcloud_vec[j].points[index].x;
		  tempoint[1] = pointcloud_vec[j].points[index].y;
		  tempoint[2] = pointcloud_vec[j].points[index].z;

		  tempoint.ringandtime = pointcloud_vec[j].points[index].ring + 0.0999999;

		  tempoint.intensity = pointcloud_vec[j].points[index].intensity;
		  double range = tempoint.norm();
		  int laser = pointcloud_vec[j].points[index].ring;
		  if(lasernum == 64)
		  {
		  if(tempoint.intensity<intensityaverage[laser]-30)
			  tempoint.intensity = 0.5;
		  else if(tempoint.intensity>intensityaverage[laser]+30)
			  tempoint.intensity = 0.9;
		  else if(tempoint.intensity>intensityaverage[laser]+10&&range<12)
			  tempoint.intensity = 0.9;
		  else
			  tempoint.intensity = 0.5;
		  }
		  else
		  {
			  if(tempoint.intensity<intensityaverage[laser] + intensityvar[laser]/4)
				  tempoint.intensity = 0.5;
		  }

		  point_cloud.push_back(tempoint);
			Eigen::Vector3d origin;
			origin[0] = 0;
			origin[1] = 0;
			origin[2] = 0;
			origins.push_back(origin);

		}
	  }

//	LOG(INFO)<<"slam point num:"<<point_cloud.size();
  }
  /// \brief 里程计位姿后处理
  ///
  /// 里程计位姿后处理，转换到gps/惯导坐标系，转换成经纬度信息
  void posePostProcess()
  {
    const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose; //rear axle estimated pose
    if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
      return;
    static ivcommon::transform::Rigid3d last_pose = localpose;
    static ivcommon::transform::Rigid3d last_pose_calibrated = localpose;
    ivcommon::transform::Rigid3d pose_diff = last_pose.inverse()*localpose;
    Eigen::Vector3d angle_calibration(0,0.0,0.00*M_PI/180);//(0.0011*M_PI/180, -0.0022*M_PI/180, 0.006*M_PI/180);
    angle_calibration = angle_calibration*pose_diff.translation()[1];//??
    Eigen::Quaterniond anglecalibration = ivcommon::transform::RollPitchYaw(angle_calibration[0],angle_calibration[1],angle_calibration[2]);
    auto pose_calibrated = last_pose_calibrated*pose_diff * ivcommon::transform::Rigid3d::Rotation(anglecalibration);
    const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * (pose_calibrated * gpstrans_); // estimated INS pose
    sensor::GpsInsData gpsdata;
    lidarodometry_gpsdata_.time = localmapping_->pose_estimate().time;
    lidarodometry_gpsdata_.position[0] = globalpose.translation()[0];
    lidarodometry_gpsdata_.position[1] = globalpose.translation()[1];
    lidarodometry_gpsdata_.position[2] = globalpose.translation()[2];
    Eigen::Quaterniond tempquat = globalpose.rotation();

    Eigen::Vector3d rollpitchyaw= ivcommon::transform::toRollPitchYaw(tempquat);
    lidarodometry_gpsdata_.roll = rollpitchyaw[0] * 180/M_PI;
    lidarodometry_gpsdata_.pitch = rollpitchyaw[1] * 180/M_PI;
    lidarodometry_gpsdata_.heading = rollpitchyaw[2] * 180/M_PI;
//    LOG(INFO)<<lidarodometry_gpsdata_.heading<<" "<<rollpitchyaw[0]*180/M_PI<<" "<<rollpitchyaw[1]*180/M_PI<<" "<<rollpitchyaw[2]*180/M_PI;

    grid_to_geographic(a,e2,zone,hemi,
		       lidarodometry_gpsdata_.position[1],lidarodometry_gpsdata_.position[0] + 500000
		       ,&lidarodometry_gpsdata_.latitude,&lidarodometry_gpsdata_.longitude);

    lidarodometry_gpsdata_.latitude *= 180/M_PI;
    lidarodometry_gpsdata_.longitude *= 180/M_PI;
    last_pose = localpose;
    last_pose_calibrated = pose_calibrated;
  }

  /// \brief 发布里程计消息
  ///
  /// \param msg_header 点云消息头
  void publishOdometry(const std_msgs::Header& msg_header)
  {
	const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
	if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
	  return;
    static ivcommon::transform::Rigid3d last_pose = localpose;
    static ivcommon::transform::Rigid3d last_pose_calibrated = localpose;
    ivcommon::transform::Rigid3d pose_diff = last_pose.inverse()*localpose;
    Eigen::Vector3d angle_calibration(0*M_PI/180,0*M_PI/180, 0*M_PI/180);//(-0.000*M_PI/180,0.006*M_PI/180, 0.002*M_PI/180);//(0.0011*M_PI/180, -0.0022*M_PI/180, 0.003*M_PI/180);
    angle_calibration = angle_calibration*pose_diff.translation()[1];
    Eigen::Quaterniond anglecalibration = ivcommon::transform::RollPitchYaw(angle_calibration[0],angle_calibration[1],angle_calibration[2]);
    auto pose_calibrated = last_pose_calibrated*pose_diff * ivcommon::transform::Rigid3d::Rotation(anglecalibration);
    last_pose = localpose;
    last_pose_calibrated = pose_calibrated;
//	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * pose_calibrated;
	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * localpose;
	double latitude,longitude;
    grid_to_geographic(a,e2,zone,hemi,
    		globalpose.translation()[1],globalpose.translation()[0] + 500000
		       ,&latitude,&longitude);

	sensor_driver_msgs::OdometrywithGps laserOdometry;
	laserOdometry.header.stamp = msg_header.stamp;
	laserOdometry.header.frame_id = "global_earth_frame";
	laserOdometry.header.frame_id = "global_earth_frame";
	laserOdometry.odometry.header = laserOdometry.header;
	laserOdometry.gps.header = laserOdometry.header;
	laserOdometry.odometry.child_frame_id = "vehicle_lidar_odometry_frame";

	double tx = globalpose.translation()[0];
	double ty = globalpose.translation()[1];
	double tz = globalpose.translation()[2];

	geometry_msgs::Quaternion geoQuat;
	geoQuat.x = globalpose.rotation().x();
	geoQuat.y = globalpose.rotation().y();
	geoQuat.z = globalpose.rotation().z();
	geoQuat.w = globalpose.rotation().w();
	//



	laserOdometry.odometry.pose.pose.orientation.x = geoQuat.x;
	laserOdometry.odometry.pose.pose.orientation.y = geoQuat.y;
	laserOdometry.odometry.pose.pose.orientation.z = geoQuat.z;
	laserOdometry.odometry.pose.pose.orientation.w = geoQuat.w;
	laserOdometry.odometry.pose.pose.position.x = tx;
	laserOdometry.odometry.pose.pose.position.y = ty;
	laserOdometry.odometry.pose.pose.position.z = tz;

	laserOdometry.gps.longitude = longitude*180/M_PI;
	laserOdometry.gps.latitude = latitude*180/M_PI;
	laserOdometry.gps.altitude = tz;
	//		laserOdometry.
	pubLaserOdometry_.publish(laserOdometry);
	LOG(INFO)<<"pubLaserOdometry_";
	//		LOG(INFO)<<pose.pose.translation();
	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform laserOdometryTrans;
	laserOdometryTrans.frame_id_ = "global_earth_frame";
	laserOdometryTrans.child_frame_id_ = "vehicle_lidar_odometry_frame";
	laserOdometryTrans.stamp_ = laserOdometry.header.stamp;
	laserOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
	laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
	tfBroadcaster.sendTransform(laserOdometryTrans);
//	LOG(WARNING)<<std::fixed<<std::setprecision(3)<<"laserOdometrytime:"<<msg_header.stamp.toSec();
  }

  /// \brief 发布用于建图的里程计消息
  ///
  /// \param msg_header 点云消息头
  /// \param insertionresult 子地图
  void publishLidarOdometryForMapping(const std_msgs::Header& msg_header,const std::unique_ptr<mapping3d::LocalTrajectoryBuilder::InsertionResult>& insertionresult)
  {
  	const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
  	if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
  	  return;

	static ivcommon::transform::Rigid3d last_pose = localpose;
	static ivcommon::transform::Rigid3d last_pose_calibrated = localpose;
	::ivcommon::transform::Rigid3d pose_diff = last_pose.inverse()*localpose;
	Eigen::Vector3d angle_calibration(0*M_PI/180,0*M_PI/180, 0*M_PI/180);//(-0.000*M_PI/180,0.006*M_PI/180, 0.002*M_PI/180);//(0.0011*M_PI/180, -0.0022*M_PI/180, 0.003*M_PI/180);
	angle_calibration = angle_calibration*pose_diff.translation()[1];
	Eigen::Quaterniond anglecalibration = ivcommon::transform::RollPitchYaw(angle_calibration[0],angle_calibration[1],angle_calibration[2]);
	auto pose_calibrated = last_pose_calibrated*pose_diff * ivcommon::transform::Rigid3d::Rotation(anglecalibration);
	last_pose = localpose;
	last_pose_calibrated = pose_calibrated;
  	const ivcommon::transform::Rigid3d globalpose =localmapping_->global_init_pose().pose * pose_calibrated;
//  	LOG(WARNING)<<"localpose:"<<localpose;
//  	LOG(WARNING)<<"pose_calibrated:"<<pose_calibrated;
  //	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * localpose;
//  	LOG(INFO)<<"globalpose double:"<<globalpose.DebugString();
//  	LOG(INFO)<<"globalpose float :"<<globalpose.cast<float>().DebugString();
//  	auto angle = ivcommon::transform::toRollPitchYaw(globalpose.rotation());
//  	LOG(INFO)<<"RollPitchYaw double:"<<" "<<angle[0]*180/M_PI<<" "<<angle[1]*180/M_PI<<" "<<angle[2]*180/M_PI;
//  	angle = ivcommon::transform::toRollPitchYaw(globalpose.cast<float>().rotation().cast<double>());
//  	  	LOG(INFO)<<"RollPitchYaw float:"<<" "<<angle[0]*180/M_PI<<" "<<angle[1]*180/M_PI<<" "<<angle[2]*180/M_PI;
  	double latitude,longitude;
    grid_to_geographic(a,e2,zone,hemi,
      		globalpose.translation()[1],globalpose.translation()[0] + 500000
  		       ,&latitude,&longitude);
    covgrid_slam_msgs::LidarOdometryForMapping submaptrigger;

    submaptrigger.header.frame_id = "global_earth_frame";
    submaptrigger.header.stamp = msg_header.stamp;
    submaptrigger.odometry.header = submaptrigger.header;
    submaptrigger.gps.header = submaptrigger.header;
    submaptrigger.odometry.child_frame_id = "vehicle_lidar_odometry_frame";

  	double tx = globalpose.translation()[0];
  	double ty = globalpose.translation()[1];
  	double tz = globalpose.translation()[2];

  	geometry_msgs::Quaternion geoQuat;
  	geoQuat.x = globalpose.rotation().x();
  	geoQuat.y = globalpose.rotation().y();
  	geoQuat.z = globalpose.rotation().z();
  	geoQuat.w = globalpose.rotation().w();
  	//

  	submaptrigger.odometry.pose.pose.orientation.x = geoQuat.x;
  	submaptrigger.odometry.pose.pose.orientation.y = geoQuat.y;
  	submaptrigger.odometry.pose.pose.orientation.z = geoQuat.z;
  	submaptrigger.odometry.pose.pose.orientation.w = geoQuat.w;
  	submaptrigger.odometry.pose.pose.position.x = tx;
  	submaptrigger.odometry.pose.pose.position.y = ty;
  	submaptrigger.odometry.pose.pose.position.z = tz;

  	submaptrigger.gps.longitude = longitude*180/M_PI;
  	submaptrigger.gps.latitude = latitude*180/M_PI;
  	submaptrigger.gps.altitude = tz;
  	//		laserOdometry.
  	LOG(WARNING)<<"submap num:"<<insertionresult->insertion_submaps.size();
  	for(auto submap:insertionresult->insertion_submaps)
  		submaptrigger.indexs.push_back(submap->index());
  	if(options_.submaps_options().update_flag())
  	{
  		if(insertionresult->insertion_submaps.front()->finished())
  			submaptrigger.mode = 1;
  		else
  			submaptrigger.mode = 0;
  	}
  	else
  		submaptrigger.mode = 2;
  	pubLidarOdometryForMapping_.publish(submaptrigger);

  //	LOG(WARNING)<<std::fixed<<std::setprecision(3)<<"laserOdometrytime:"<<msg_header.stamp.toSec();
  }

  /// \brief 发布雷达里程计的gps格式的位置
  ///
  /// \param msg_header 点云消息头
    void publishGpsByLidarOdometry(const std_msgs::Header& msg_header)
   {
   	const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
   	if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
   	  return;

   	const ivcommon::transform::Rigid3d globalpose = localmapping_->PoseLikeGps();
   //	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * localpose;
   	double latitude,longitude;
     grid_to_geographic(a,e2,zone,hemi,
       		globalpose.translation()[1],globalpose.translation()[0] + 500000
   		       ,&latitude,&longitude);
     covgrid_slam_msgs::GpsByLidarOdometry gpsbylidarodometry;

     gpsbylidarodometry.odometry.header.frame_id = "global_earth_frame";
     gpsbylidarodometry.header.stamp = msg_header.stamp;
     gpsbylidarodometry.odometry.header = gpsbylidarodometry.header;
     gpsbylidarodometry.gps.header = gpsbylidarodometry.header;
     gpsbylidarodometry.odometry.child_frame_id = "vehicle_lidar_odometry_frame";

   	double tx = globalpose.translation()[0];
   	double ty = globalpose.translation()[1];
   	double tz = globalpose.translation()[2];

   	geometry_msgs::Quaternion geoQuat;
   	geoQuat.x = globalpose.rotation().x();
   	geoQuat.y = globalpose.rotation().y();
   	geoQuat.z = globalpose.rotation().z();
   	geoQuat.w = globalpose.rotation().w();
   	//

   	gpsbylidarodometry.odometry.pose.pose.orientation.x = geoQuat.x;
   	gpsbylidarodometry.odometry.pose.pose.orientation.y = geoQuat.y;
   	gpsbylidarodometry.odometry.pose.pose.orientation.z = geoQuat.z;
   	gpsbylidarodometry.odometry.pose.pose.orientation.w = geoQuat.w;
   	gpsbylidarodometry.odometry.pose.pose.position.x = tx;
   	gpsbylidarodometry.odometry.pose.pose.position.y = ty;
   	gpsbylidarodometry.odometry.pose.pose.position.z = tz;

   	gpsbylidarodometry.gps.longitude = longitude*180/M_PI;
   	gpsbylidarodometry.gps.latitude = latitude*180/M_PI;
   	gpsbylidarodometry.gps.altitude = tz;
   	//		laserOdometry.

   	if(options_.submaps_options().update_flag())
   	{
   	    gpsbylidarodometry.mode = 0;
   	}
   	else
   		gpsbylidarodometry.mode = 2;
   	pubGpsByLidarOdometry_.publish(gpsbylidarodometry);


    static std::string filename = log_path_ + "/gpsbyodometry.txt";
    static std::ofstream posefile(filename.c_str());
    posefile<<std::fixed<<std::setprecision(3)<<msg_header.stamp.toSec()-log_start_time_
	<<" "<<globalpose.translation()[0]
	<<" "<<globalpose.translation()[1]
	<<" "<<globalpose.translation()[2]
	<<" "<<lidarodometry_gpsdata_.heading
	<<std::setprecision(7)
	<<" "<<lidarodometry_gpsdata_.latitude
	<<" "<<lidarodometry_gpsdata_.longitude
	<<std::setprecision(7)
    <<" "<<ros::Time::now().toSec() - msg_header.stamp.toSec()
	<<" "<<lidarodometry_gpsdata_.pitch
	<<" "<<lidarodometry_gpsdata_.roll
	<<" "<<localmapping_->matched_probability()
	<<" "<<gpsbylidarodometry.mode
	<<std::endl;
   //	LOG(WARNING)<<std::fixed<<std::setprecision(3)<<"laserOdometrytime:"<<msg_header.stamp.toSec();
   }

/// \brief rviz子地图显示
///
  void showSubmap()
  {
    tf::TransformBroadcaster tfBroadcaster;

    tf::StampedTransform laserSubmapTrans;
    bool updated = false;
    while(!processthreadfinished_)
      {
	if(updated)
	  tfBroadcaster.sendTransform(laserSubmapTrans);
	std::unique_ptr<mapping3d::LocalTrajectoryBuilder::InsertionResult> insertionresult
				= insertionresults_.PopWithTimeout(::ivcommon::FromSeconds(0.05));

	if (insertionresult==nullptr)
	  continue;
	while(insertionresults_.Size()>0&&!insertionresult->insertion_submaps.at(0)->finished())
	  insertionresult = insertionresults_.Pop();
	if(pubLaserCloudGrid_.getNumSubscribers()==0)
	  {
	    usleep(10000);
	  }
	else
	  {
//	    LOG(INFO)<<"showsubmap";
	    updated = true;
	    pcl::PointCloud<pcl::PointXYZI> point_cloud;

//	    const mapping3d::HybridGrid& grid =
//		  insertionresult->insertion_submaps.at(0)->low_resolution_hybrid_grid();
//	    for(auto cell:grid)
//	      {
//		pcl::PointXYZI temppoint;
//		if(mapping::ValueToProbability(value)>0.5)
//		  {
//		    const Eigen::Vector3d& point = grid.GetCenterOfCell(cell.first);
//		    temppoint.x = point[0];
//		    temppoint.y = point[1];
//		    temppoint.z = point[2];
//		    temppoint.intensity = 0;
//		  }
//		point_cloud.push_back(temppoint);
//	      }

	    bool showintensitygrid = false;
	    if(!showintensitygrid)
	    {
			const mapping3d::FeatureHybridGrid& featuregrid =
			  insertionresult->insertion_submaps.at(0)->feature_hybrid_grid();
			int gridsize = featuregrid.grid_size();
			for(auto it = mapping3d::FeatureHybridGrid::Iterator(featuregrid); !it.Done(); it.Next())
			  {

			auto index = it.GetCellIndex();
			auto value = it.GetValue();
			float probability = mapping::ValueToProbability(value.probability);//featuregrid.GetProbability(cell.first);


			if(probability<0.501f||value.type==mapping3d::FeatureElement::Type::KInit
				||value.type==mapping3d::FeatureElement::Type::KVoid)
			  continue;
			const Eigen::Vector3d point = featuregrid.GetCenterOfCell(index);
			pcl::PointXYZI temppoint;
			if(value.type==mapping3d::FeatureElement::Type::KFlat)
			  {

				temppoint.x = point[0];
				temppoint.y = point[1];
				temppoint.z = point[2];
				temppoint.intensity = 1;
				Eigen::Vector3d n = value.covresult.V.col(0);
				double angle = (std::acos(n[2]/n.norm()))*180/M_PI;

				double xdelta = value.covresult.covmat(0,0);
				double ydelta = value.covresult.covmat(1,1);
				double zdelta = value.covresult.covmat(2,2);
				if(angle > 90)
				  angle = 180 -angle;
				if(angle >30&&zdelta> 2*xdelta && zdelta> 2*ydelta &&zdelta>0.04)
				{
					  temppoint.intensity = 5;
	//				  LOG(INFO)<<value.covresult.covmat;
				}
				else
				  temppoint.intensity = 0;
			  }
			else if(value.type==mapping3d::FeatureElement::Type::KLine)
			  {
				temppoint.x = point[0];
				temppoint.y = point[1];
				temppoint.z = point[2];
				temppoint.intensity = 2;
			  }
			else if(value.type==mapping3d::FeatureElement::Type::KCluster)
			  {
				temppoint.x = point[0];
				temppoint.y = point[1];
				temppoint.z = point[2];
				temppoint.intensity = 3;
				Eigen::Vector3d n = value.covresult.V.col(0);
				double angle = (std::acos(n[2]/n.norm()))*180/M_PI;

				double xdelta = value.covresult.covmat(0,0);
				double ydelta = value.covresult.covmat(1,1);
				double zdelta = value.covresult.covmat(2,2);
				if(angle > 90)
				  angle = 180 -angle;
				if(angle >30&&zdelta> 2*xdelta && zdelta> 2*ydelta && zdelta>0.04)
				{
					  temppoint.intensity = 5;
	//				  LOG(INFO)<<value.covresult.covmat;
				}
				else
				  temppoint.intensity = 0;
			  }


	//		temppoint.intensity = angle;
			temppoint.intensity = probability;
			point_cloud.push_back(temppoint);
			  }
	//	    LOG(INFO)<<point_cloud.size();

	    }
	    else
	    {
		    const mapping3d::IntensityHybridGrid& intensitygrid =
			  insertionresult->insertion_submaps.at(0)->intensity_hybrid_grid();
		    int gridsize = intensitygrid.grid_size();
		    for(auto it = mapping3d::IntensityHybridGrid::Iterator(intensitygrid); !it.Done(); it.Next())
		      {

			auto index = it.GetCellIndex();
			auto value = it.GetValue();
			float intensity = value.average;

			if(intensity<0.001f)
			  continue;
			const Eigen::Vector3d point = intensitygrid.GetCenterOfCell(index);
			pcl::PointXYZI temppoint;

			temppoint.x = point[0];
			temppoint.y = point[1];
			temppoint.z = point[2];
			temppoint.intensity = intensity;

			point_cloud.push_back(temppoint);
		      }
	//	    LOG(INFO)<<point_cloud.size();

	    }
	    sensor_msgs::PointCloud2 laserCloud;

	    pcl::toROSMsg(point_cloud, laserCloud);
	    laserCloud.header.frame_id = "featuresubmap_frame";
	    laserCloud.header.stamp =  ToRos(insertionresult->time);

//	    LOG(INFO)<<"publish lasercloud";
	    pubLaserCloudGrid_.publish(laserCloud);
	  }

	    const ivcommon::transform::Rigid3d& trans = insertionresult->insertion_submaps.at(0)->local_pose();

	    laserSubmapTrans.frame_id_ = "global_init_frame";
	    laserSubmapTrans.child_frame_id_ = "featuresubmap_frame";
	    laserSubmapTrans.stamp_ = ToRos(insertionresult->time);
	    laserSubmapTrans.setRotation(tf::Quaternion(trans.rotation().x(), trans.rotation().y()
							  , trans.rotation().z(), trans.rotation().w()));
	    laserSubmapTrans.setOrigin(tf::Vector3(trans.translation()[0], trans.translation()[1], trans.translation()[2]));

      }
  }

  /// \brief 发送前后帧位姿转换结果
  ///
  /// \param last_pose 上一帧位姿
  /// \param pose 当前帧位姿
  void sendPoseTransform(const ivcommon::transform::posestamped& last_pose ,const ivcommon::transform::posestamped& pose)
  {
    if(last_pose.time == ::ivcommon::Time::min()||pose.time == ::ivcommon::Time::min())
      return;
    static std::string filename = log_path_ + "/transform.txt";
    static std::ofstream posefile(filename.c_str());
    ivcommon::transform::Rigid3d trans = last_pose.pose.inverse()*pose.pose;
    Eigen::Quaterniond tempquat = trans.rotation();
//    tempquat.x() = -trans.rotation().y();
//    tempquat.y() = trans.rotation().x();
    Eigen::Vector3d angle = ivcommon::transform::toRollPitchYaw(tempquat);
    std::stringstream strs;
    strs<<std::fixed<<std::setprecision(6)<<"$LID_ODO"<<","<<ToRos(last_pose.time).toSec()<<","
	<<ToRos(pose.time).toSec()<<","
	<<angle[0]*180/M_PI<<","<<angle[1]*180/M_PI<<","<<angle[2]*180/M_PI<<","
	<<trans.translation()[0]<<","<<trans.translation()[1]<<","<<trans.translation()[2]<<",*";
//    LOG(INFO)<<strs.str().c_str();
    std::string sendstr = strs.str();
    posefile<<sendstr<<std::endl;
    char checkbyte = 0;
    for(const char& c:sendstr)
      {
	checkbyte ^= c;
      }
    sendstr.push_back(checkbyte);

//    ROS_INFO(strs.str().c_str());
    sendposedata_.send(sendstr);
  }

  /// \brief 发送雷达里程计数据
  ///
void sendGpsByLidarOdometry()
{
   	const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
   	::ivcommon::Time time = localmapping_->pose_estimate().time;
   	if(/*localpose.translation().norm()<0.01 ||*/ time == ::ivcommon::Time::min())
   	  return;

   	const ivcommon::transform::Rigid3d globalpose = localmapping_->PoseLikeGps();
   //	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * localpose;
   	double latitude,longitude,heading;
    grid_to_geographic(a,e2,zone,hemi,
       		globalpose.translation()[1],globalpose.translation()[0] + 500000
   		       ,&latitude,&longitude);

    Eigen::Vector3d rollpitchyaw= ivcommon::transform::toRollPitchYaw(globalpose.rotation());

    latitude = latitude * 180/M_PI;
    longitude = longitude * 180/M_PI;
    heading = rollpitchyaw[2] * 180/M_PI;

    std::stringstream strs;

    strs<<"$LID_MATCH_GPS"<<","<<static_cast<int>(lidargpsstatus_)<<","
	<<localmapping_->matching_submap_index()<<","
	<<std::fixed
	<<std::setprecision(4)
	<<(ToRos(time)).toSec()<<","
	<<std::setprecision(8)
	<<latitude<<","
	<<longitude<<","
	<<heading<<","
	<<"*";

    std::string sendstr = strs.str();
    LOG(INFO)<<sendstr.c_str();
    char checkbyte = 0;
    for(const char& c:sendstr)
      {
	checkbyte ^= c;
      }
    sendstr.push_back(checkbyte);

    sendposedata_.send(sendstr);
}
/// \brief 已弃用
///
void sendLidarGps()
  {
    if(lidarodometry_gpsdata_.time ==  ::ivcommon::Time::min())
      return;

    std::stringstream strs;

    strs<<"$LID_GPS"<<","<<static_cast<int>(lidargpsstatus_)<<","
	<<localmapping_->matching_submap_index()<<","
	<<std::fixed
	<<std::setprecision(4)
	<<ToRos(lidarodometry_gpsdata_.time).toSec()<<","
	<<std::setprecision(8)
	<<lidarodometry_gpsdata_.latitude<<","
	<<lidarodometry_gpsdata_.longitude<<","
	<<lidarodometry_gpsdata_.heading<<","
	<<opendoor_flag_<<",*";

    std::string sendstr = strs.str();
    LOG(INFO)<<sendstr.c_str();
    char checkbyte = 0;
    for(const char& c:sendstr)
      {
	checkbyte ^= c;
      }
    sendstr.push_back(checkbyte);

    sendposedata_.send(sendstr);
  }
/// \brief 记录里程计位姿数据
///

  void logData()
  {
    if(lidarodometry_gpsdata_.time == ::ivcommon::Time::min())
      return;
    int mode = 0;
  	if(options_.submaps_options().update_flag())
  	{
  		mode = 0;
  	}
  	else
  		mode = 2;
    static std::string filename = log_path_ + "/trajectory.txt";
    static std::ofstream posefile(filename.c_str());
    posefile<<std::fixed<<std::setprecision(4)<<ToRos(lidarodometry_gpsdata_.time).toSec()-log_start_time_
	<<" "<<lidarodometry_gpsdata_.position[0]
	<<" "<<lidarodometry_gpsdata_.position[1]
	<<" "<<lidarodometry_gpsdata_.position[2]
	<<" "<<lidarodometry_gpsdata_.heading
	<<" "<<lidarodometry_gpsdata_.pitch
	<<" "<<lidarodometry_gpsdata_.roll
	<<" "<<gpsdata_last_.position[0]
    <<" "<<gpsdata_last_.position[1]
    <<" "<<gpsdata_last_.position[2]
    <<" "<<gpsdata_last_.heading
    <<" "<<gpsdata_last_.pitch
    <<" "<<gpsdata_last_.roll

	//    <<" "<<lidarodometry_gpsdata_.position[0] - gpsdata_last_.position[0]
	//    <<" "<<lidarodometry_gpsdata_.position[1] - gpsdata_last_.position[1]
	//    <<" "<<lidarodometry_gpsdata_.position[2] - gpsdata_last_.position[2]
    //	  <<" "<<lidarodometry_gpsdata_.heading - gpsdata_last_.heading

//	<<std::setprecision(7)
//	<<" "<<lidarodometry_gpsdata_.latitude
//	<<" "<<lidarodometry_gpsdata_.longitude
	<<std::setprecision(7)
    <<" "<<ros::Time::now().toSec() - ToRos(lidarodometry_gpsdata_.time).toSec()

//	<<" "<<lidarodometry_gpsdata_.pitch - gpsdata_last_.pitch
//	<<" "<<lidarodometry_gpsdata_.roll - gpsdata_last_.roll
//	<<" "<<localmapping_->matched_probability()
//	<<" "<<mode
	<<std::endl;
  }

  /// \brief 记录里程计位姿数据
  ///

    void logKittiData()
    {
      	const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
      	if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
      	  return;
      static std::string filename = log_path_ + "/trajectorykitti.txt";
      static std::ofstream posefile(filename.c_str());
      posefile<<std::fixed<<std::setprecision(3)<<ToRos(lidarodometry_gpsdata_.time).toSec()-log_start_time_
  	<<" "<<lidarodometry_gpsdata_.position[0]
  	<<" "<<lidarodometry_gpsdata_.position[1]
  	<<" "<<lidarodometry_gpsdata_.position[2]
  	<<" "<<lidarodometry_gpsdata_.heading
  	<<std::setprecision(7)
  	<<" "<<lidarodometry_gpsdata_.latitude
  	<<" "<<lidarodometry_gpsdata_.longitude
  	<<std::setprecision(7)
      <<" "<<ros::Time::now().toSec() - ToRos(lidarodometry_gpsdata_.time).toSec()
  	<<" "<<lidarodometry_gpsdata_.pitch
  	<<" "<<lidarodometry_gpsdata_.roll
  	<<" "<<localmapping_->matched_probability()
  	<<std::endl;
    }
  /// \brief 保存点云线程
  ///
  void savecloudthread()
  {

    static std::string filename = log_path_ + "/pointcloud.txt";
    static std::ofstream posefile(filename.c_str());
    static int i=0;

    while(!processthreadfinished_)
    {
    	auto cloudpair = savecloud_.Pop();
    	if(cloudpair == nullptr)
    		break;
    	pcl::PointCloud<pcl::PointXYZI>& point_cloud = cloudpair->first;
		const ivcommon::transform::posestamped& pose = cloudpair->second;
		std::string pcddir=log_path_ + "/pointcloud";
		std::stringstream sstr;
		sstr<<pcddir<<"/"<<i<<".pcd";
		::ivcommon::SetRecordDir(pcddir);

		pcl::PointCloud<pcl::PointXYZI> save_point_cloud;
		for(int i=0;i< point_cloud.size();i+=2)
		{
			if(point_cloud[i].range > 0.5)
				save_point_cloud.push_back(point_cloud[i]);
		}
		Eigen::Quaterniond tempquat = pose.pose.rotation();


		Eigen::Vector3d rollpitchyaw= ivcommon::transform::toRollPitchYaw(tempquat);

		std::string pcdfilename = sstr.str();
		posefile<<pcdfilename.c_str()<<" "
		<<std::fixed<<std::setprecision(6)<<rollpitchyaw[1]*180/M_PI
		<<" "<<rollpitchyaw[0]*180/M_PI
		<<" "<<rollpitchyaw[2]*180/M_PI
		<<" "<<pose.pose.translation()[0]
		<<" "<<pose.pose.translation()[1]
		<<" "<<pose.pose.translation()[2]
		<<std::endl;
		point_cloud.erase(point_cloud.end()-65,point_cloud.end());
		pcl::io::savePCDFile(pcdfilename, save_point_cloud,true);
		i++;
    }

  }
  /// \brief 记录gps惯导位姿
  ///
  /// \param gpsdata 数据
  void logGpsData(const sensor::GpsInsData& gpsdata)
  {
    if(gpsdata.time == ::ivcommon::Time::min())
      return;
    static std::string filename = log_path_ + "/gpsmetricdata.txt";
    static std::ofstream posefile(filename.c_str());


//    rollpitchyaw= ivcommon::transform::toRollPitchYaw(tempquat);
//
//    double roll = rollpitchyaw[0] * 180/M_PI;
//    double pitch = rollpitchyaw[1] * 180/M_PI;
//    double heading = rollpitchyaw[2] * 180/M_PI;

    posefile<<std::fixed<<std::setprecision(3)<<ToRos(gpsdata.time).toSec()-log_start_time_
	<<" "<<gpsdata.position[0]
	<<" "<<gpsdata.position[1]
	<<" "<<gpsdata.position[2]
	<<" "<<gpsdata.heading
	<<std::setprecision(7)
	<<" "<<gpsdata.latitude
	<<" "<<gpsdata.longitude
	<<" "<<gpsdata.pitch
	<<" "<<gpsdata.roll
	<<std::endl;


  }
  /// \brief 记录融合位姿数据
  ///
  /// \param fuseposedata 融合位姿数据
  void logFusePoseData(const sensor::GpsInsData& fuseposedata)
  {
    if(fuseposedata.time == ::ivcommon::Time::min())
      return;
    static std::string filename = log_path_ + "/fuseposemetricdata.txt";
    static std::ofstream posefile(filename.c_str());


//    rollpitchyaw= ivcommon::transform::toRollPitchYaw(tempquat);
//
//    double roll = rollpitchyaw[0] * 180/M_PI;
//    double pitch = rollpitchyaw[1] * 180/M_PI;
//    double heading = rollpitchyaw[2] * 180/M_PI;

    posefile<<std::fixed<<std::setprecision(3)<<ToRos(fuseposedata.time).toSec()-log_start_time_
	<<" "<<fuseposedata.position[0]
	<<" "<<fuseposedata.position[1]
	<<" "<<fuseposedata.position[2]
	<<" "<<fuseposedata.heading
	<<std::setprecision(7)
	<<" "<<fuseposedata.latitude
	<<" "<<fuseposedata.longitude
	<<" "<<fuseposedata.pitch
	<<" "<<fuseposedata.roll
	<<std::endl;
  }
  /// \brief 该类调用接口
  ///
  void run()
  {
    processthreadfinished_ = false;
    processthread_=new boost::thread(boost::bind(&Node::processthread,this));
    showthread_=new boost::thread(boost::bind(&Node::showSubmap,this));
//    savethread_=new boost::thread(boost::bind(&Node::savecloudthread,this));
  }

  private:
#if LASER_NUM==64
  const float scanPeriod = 0.1;/**< 雷达扫描周期 */
#elif LASER_NUM==32
  const float scanPeriod = 0.1;/**< 雷达扫描周期 */
#endif

ivcommon::transform::Rigid3d gpstrans_;
  ::ros::NodeHandle node_handle_;/**< 公有 */

  ros::Publisher pubLaserCloudGrid_;/**< 发布栅格地图用于显示 */
  ros::Publisher pubLaserOdometry_;/**< 发布里程计信息 */
  ros::Publisher pubLidarOdometryForMapping_;/**< 发布建图用里程计信息 */
  ros::Publisher pubLaserCloud_undistortion_;
  ros::Publisher pubGpsByLidarOdometry_;/**< 发布离线匹配的GPS位姿 */
  ros::Publisher pubLaserCloud_;/**< 订阅点云 */
  ros::Subscriber subLaserCloud_;/**< 订阅点云 */
  ros::Subscriber subImu_;/**< 订阅imu信息 */
  ros::Subscriber subGps_;/**< 订阅gps */
  ros::Subscriber subEcu_;/**< 订阅ecu */
  ros::Subscriber subInsVelocity_;/**< 订阅惯导速度信息 */
  ros::Subscriber subFusePose_;/**<订阅融合定位结果  */

//  ros::Subscriber subConfig;


  mapping3d::proto::LocalTrajectoryBuilderOptions options_;/**< 雷达里程计的相关参数 */
  mapping3d::LocalTrajectoryBuilder* localmapping_;/**< 雷达里程计实现对象 */
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::ImuData>> imudatas_;/**< imu数据队列 */
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::GpsInsData>> gpsdatas_;/**< gps惯导数据队列 */
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::EcuData>> ecudatas_;/**<  ecu数据队列*/
  ::ivcommon::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> PointCloudMsgs_;/**< 点云消息队列 */
  sensor::GpsInsData lidarodometry_gpsdata_;/**< 雷达里程计数据 */
  sensor::GpsInsData gpsdata_last_;

  ::ivcommon::BlockingQueue<std::unique_ptr<std::pair<pcl::PointCloud<pcl::PointXYZI>,ivcommon::transform::posestamped>>> savecloud_;/**< 用于保存pcd */
  boost::thread* processthread_=nullptr;/**< 处理线程 */
  boost::thread* showthread_=nullptr;/**< 显示线程 */
  boost::thread* savethread_=nullptr;/**< 点云保存线程 */
  ::ivcommon::BlockingQueue<std::unique_ptr<mapping3d::LocalTrajectoryBuilder::InsertionResult>> insertionresults_;/**< 地图插入结果 */
  BoostUdp sendposedata_;/**< 发送位姿数据 */
  std::fstream filestream_;/**< 文件流 */
  std::string configstr_;/**< xml配置信息 */
  XmlConf xml_conf_;/**< xml解析类 */
  std::string log_path_;/**<  数据保存路径*/

  double log_start_time_;/**< log初始时间 */
  constexpr static double a=6378137;/**< 用于utm坐标转换 */
  constexpr static double e2= 0.0818192*0.0818192;/**<  用于utm坐标转换，e的平方*/
  GridZone zone =UTM_ZONE_AUTO;/**< utm区域，默认自动模式 */
  Hemisphere hemi = HEMI_NORTH;/**< 半球，默认北半球 */

  sensor::LidarGpsStatus lidargpsstatus_;/**< 雷达gps状态 */
  ros::Timer tftimer_;  /**< 定时器 */
  const tf2_ros::Buffer*  tf_buffer_;/**< tf 转换矩阵获取 */
  int lasernum = LASER_NUM;
  double intensityaverage[64];
  double intensityvar[64];
  bool systemInited_;/**< 系统初始化完成标志 */
  bool imu_static_inited_;///< imu initial bias correction.
  bool opendoor_flag_; /**< 开门标志 */
  bool detect_door_;/**< 探测门外有人标志 */
  bool log_on_;/**< 是否记录数据 */
  bool processthreadfinished_;/**< 处理线程完成标志 */
  bool withgps_;/**< 有gps */
  bool withecu_;/**<  有ecu*/
  bool withinsvelocity_;/**< 有惯导速度 */
  bool iskitti_;/**< kitti数据标志 */
  bool from_velodyne_driver_;/**< 使用velodyne驱动作为信息源标志 */

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidarodometrymain");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  google::InstallFailureSignalHandler();
  constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  Node node(&tf_buffer);
  node.run();
  ros::spin();
  LOG(INFO)<<"main thread end!";
  return 0;
}

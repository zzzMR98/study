// Copyright 2017, RCIV, Beijing Institute of Technology



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
#include "covgrid_slam_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/ECUData.h"
#include "sensor_driver_msgs/InsVelocity.h"
#include "covgrid_slam_msgs/LidarOdometryForMapping.h"
#include "covgrid_slam_msgs/GpsByLidarOdometry.h"
#include "covgrid_slam/mapping/pose_extrapolator.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder_options.h"
#include "covgrid_slam/sensor/data.h"
#include "ivcommon/common/time_conversion.h"

class Node{
public:
  Node():
    systemInited_(false)
  {
    string configuration_directory,configuration_basename;
    ros::param::get("~configuration_directory",configuration_directory);
    ros::param::get("~configuration_basename",configuration_basename);

    processthreadfinished_ = false;
    options_ = mapping3d::LoadOptions(configuration_directory,configuration_basename);
    options_.mutable_submaps_options()->set_update_common_map(false);
    options_.mutable_submaps_options()->set_savemap_flag(false);
    options_.mutable_submaps_options()->set_readmap_flag(false);
    options_.mutable_submaps_options()->set_update_flag(true);
    options_.mutable_submaps_options()->set_automode_flag(false);
    options_.set_without_imu(true);
    options_.mutable_scan_matcher_options()->set_fuse_ins(true);

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

    Eigen::Quaterniond gpsquat = ivcommon::transform::RollPitchYaw(
    	gpsyawpitchroll[2]*M_PI/180,gpsyawpitchroll[1]*M_PI/180,gpsyawpitchroll[0]*M_PI/180);

//    gpsquat.x() = tempquat.y();
//    gpsquat.y() = -tempquat.x();
    gpstrans_ = ivcommon::transform::Rigid3d(gpspos,gpsquat);

    withgps_ = false;
    withgps_ = false;
    withecu_ = false;
    withinsvelocity_ = false;
    ros::param::get("~withgps",withgps_);
    ros::param::get("~withecu",withecu_);
    ros::param::get("~withinsvelocity",withinsvelocity_);

    rosmessagelaunch();

  }

  ~Node()
  {
    processthreadfinished_ = true;
    savecloud_.stopQueue();

    LOG(INFO)<<"savethread_ end";
    if(processthread_!=nullptr)
    	processthread_->join();
    delete localmapping_;
  }

  void rosmessagelaunch()
  {

    subLaserCloud_ = node_handle_.subscribe<sensor_msgs::PointCloud2>
                                    ("lidar_cloud_calibrated", 2,boost::bind(&Node::laserCloudHandler,this,_1) );

    if(!options_.without_imu())
    	subImu_ = node_handle_.subscribe<sensor_msgs::Imu> ("imudata", 50, boost::bind(&Node::imuHandler,this,_1));


    pubLaserOdometry_ = node_handle_.advertise<covgrid_slam_msgs::OdometrywithGps> ("lidar_preciseodometry_to_earth", 5);
    if(withgps_)
      subGps_ = node_handle_.subscribe<sensor_driver_msgs::GpswithHeading> ("gpsdata", 50, boost::bind(&Node::gpsHandler,this,_1));
    if(withecu_)
      subEcu_ = node_handle_.subscribe<sensor_driver_msgs::ECUData> ("ecudata", 50, boost::bind(&Node::ecuHandler,this,_1));

    subInsVelocity_ = node_handle_.subscribe<sensor_driver_msgs::InsVelocity> ("insvelocity", 50, boost::bind(&Node::insvelocityHandler,this,_1));
  }

  void initgps()
  {

	  mapping3d::PosewithGps global_init_pose;
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


	    Eigen::Vector3d translation;
	    translation[0] = gpsdata->position[0];
	    translation[1] = gpsdata->position[1];
	    translation[2] = gpsdata->position[2];
//	    Eigen::Quaterniond temprotation = ivcommon::transform::RollPitchYaw(0,0,gpsdata->heading*M_PI/180);

	    Eigen::Quaterniond temprotation =options_.without_imu()
	    		? ::ivcommon::transform::RollPitchYaw(gpsdata->roll*M_PI/180,gpsdata->pitch*M_PI/180,gpsdata->heading*M_PI/180)
	    		: ::ivcommon::transform::RollPitchYaw(0,0,gpsdata->heading*M_PI/180);
	    Eigen::Quaterniond rotation = temprotation;
//	    rotation.x() = temprotation.y();
//	    rotation.y() = -temprotation.x();
//	    LOG(INFO)<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.w()<<" ";
	    ivcommon::transform::Rigid3d aftshaft = ivcommon::transform::Rigid3d(translation,rotation) * gpstrans_.inverse();
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
	    global_init_pose = {aftshaft,tempgpsdata};
	  }

      }
    else
      {
    	global_init_pose = {::ivcommon::transform::Rigid3d::Rotation(
					    Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ())),};
      }
	LOG(INFO)<<"constuct localmapping_";
    localmapping_ = new mapping3d::LocalTrajectoryBuilder(options_,global_init_pose,::ivcommon::FromRos(ros::Time::now()));
    LOG(INFO)<<"finish init gps";
    tftimer_ = node_handle_.createTimer(ros::Duration(0.1), boost::bind(&Node::timertfCallback,this));

  }

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
//    if(options_.without_imu())
    {
        imu_data.angular_velocity = gpstrans_.rotation()*imu_data.angular_velocity ;
        imu_data.linear_acceleration = gpstrans_.rotation()*imu_data.linear_acceleration ;
    }

//    imudatas_.Push(::ivcommon::make_unique<sensor::ImuData>(imu_data));
    if(systemInited_&&!options_.without_imu())
    	localmapping_->AddImuData(imu_data);
  }


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
        ivcommon::transform::Rigid3d nowpose = ::ivcommon::transform::Rigid3d(nowtranslation,tempquat) * gpstrans_.inverse();
        localmapping_->AddInsData({gps_data.time,nowpose});
    }
    if(gpsdatas_.Size()>1)
      gpsdatas_.Pop();
  }



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

  void processthread()
  {
    static ivcommon::transform::posestamped last_pose;
    initgps();
    systemInited_ = true;
    while(!processthreadfinished_)
      {
//		checkgps();

//		LOG(INFO)<<"time diff:"<<::ivcommon::now().time_since_epoch().count()<<"\t"<<::ivcommon::FromRos(ros::Time::now()).time_since_epoch().count();

		sensor_msgs::PointCloud2ConstPtr laserCloudMsg=PointCloudMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.1));
		if(laserCloudMsg==nullptr)
		{
			LOG_EVERY_N(WARNING, 10)<<"can't get lidar data.";
			continue;
		}

		std_msgs::Header msg_header = laserCloudMsg->header;
		::ivcommon::Time this_time = ::ivcommon::FromRos(ros::Time(msg_header.stamp));
		LOG(INFO)<<"got pointcloud time:"<<::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time);
		if(::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time)>0.1)
		{
			LOG(WARNING)<<"overtime:"<<::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now()) - this_time);
			continue;
		}

		pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;

		pcl::fromROSMsg(*laserCloudMsg, pcl_point_cloud);
		sensor::PointCloud point_cloud;
		vector<Eigen::Vector3d> origins;
		analysisCloud(pcl_point_cloud,point_cloud,origins);
		Eigen::Vector3d origin = Eigen::Vector3d::Zero();
		for(const auto& temporigin:origins)
		{
			origin += temporigin;
		}
		origin /=origins.size();

		LOG(INFO)<<"start";
		std::unique_ptr<mapping3d::LocalTrajectoryBuilder::InsertionResult>  insertionresult
		= localmapping_->AddRangefinderData(::ivcommon::FromRos(msg_header.stamp),origin,point_cloud);

		publishOdometry(msg_header);

		posePostProcess();
		const ivcommon::transform::posestamped pose = localmapping_->pose_estimate();



		double costtime = ::ivcommon::ToSeconds(::ivcommon::FromRos(ros::Time::now())-::ivcommon::FromRos(msg_header.stamp));
		static double totaltime = 0;
		static int totalnum = 0;
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
          (traslation1 - traslation0) / queue_delta;
      Eigen::Vector3d angular_velocity_ =
          ivcommon::transform::RotationQuaternionToAngleAxisVector(
        		  rotation0.inverse() * rotation1) /queue_delta;
      localmapping_->setvelocity(linear_velocity,angular_velocity_);


  }

  void setinitvelocity2(std::string filename)
  {
	  std::ifstream fs(filename.c_str());
	  Eigen::Vector3d traslationtemp;
	  Eigen::Vector3d yawpitchrolltemp;


	  fs>>traslationtemp[0]>>traslationtemp[1]>>traslationtemp[2]>>yawpitchrolltemp[0]>>yawpitchrolltemp[1]>>yawpitchrolltemp[2];

      localmapping_->setvelocity(traslationtemp,yawpitchrolltemp);

  }

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

  void analysisCloud(pcl::PointCloud<pcl::PointXYZI>& pcl_point_cloud
		     ,sensor::PointCloud& point_cloud, vector<Eigen::Vector3d>& origins)
  {

	  point_cloud.reserve(pcl_point_cloud.size());

	    int cloudSize = pcl_point_cloud.size();
	    if(cloudSize<64)
	    {
	    	LOG(ERROR)<<"analysis error:"<<"cloudSize<64";
	    	return;
	    }

	    int cloudnum = cloudSize%16;
	    vector<int> startnum;
	    vector<int> lasernum;
	    int totallasernum = 0;
	    int N_SCANS = 32;

		for(int i=0;i<cloudnum;i++)
		{
			Eigen::Vector3d origin;
			origin[0] = pcl_point_cloud[cloudSize-cloudnum+i].x;
			origin[1] = pcl_point_cloud[cloudSize-cloudnum+i].y;
			origin[2] = pcl_point_cloud[cloudSize-cloudnum+i].z;
			origins.push_back(origin);
			int temp = pcl_point_cloud[cloudSize-cloudnum+i].range;//起始点
			LOG(INFO)<<"start point:"<<temp;
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
			N_SCANS = lasernum.at(k);
			int offsetnum = startnum.at(k);//点云初始索引
			offsetscan += k==0?0:lasernum.at(k-1);
			int cloudpointnum = startnum.size()-k==1?cloudSize-startnum.at(k):startnum.at(k+1)-startnum.at(k);//当前点云数量
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


			std::map<int,int> indexmap;
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
				  //		    LOG(INFO)<<"indexmap"<<index<<":"<<indexmap[index];
				}
				else
				  indexmap[j] = j;
			  }
			if(hasinfo)
			  cloudpointnum -= N_SCANS;
//			LOG(INFO)<<"cloudpointnum="<<cloudpointnum;

			int interval = 1;
//			if(N_SCANS==64)
//			  interval = 2;
			for (int i = 0; i < (cloudpointnum/N_SCANS); i+=interval)
			{
				for(int laser_j=0;laser_j<N_SCANS;laser_j++)
				{

				  int index_j=indexmap[laser_j];

				  int index=i*N_SCANS+index_j+offsetnum;
				  if(pcl_point_cloud.points[index].range<0.5 && pcl_point_cloud.points[index].intensity>- 0.0001)
					  continue;
				  pcl::PointXYZI& point = pcl_point_cloud.points[index];

				  sensor::Point tempoint(point.getVector3fMap().cast<double>(),
						  laser_j+offsetscan + 0.1*(i/double(cloudpointnum/N_SCANS)));
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
//	LOG(INFO)<<"slam point num:"<<point_cloud.size();
  }

  void posePostProcess()
  {
    const ivcommon::transform::Rigid3d localpose = localmapping_->pose_estimate().pose;
    if(/*localpose.translation().norm()<0.01 ||*/ localmapping_->pose_estimate().time == ::ivcommon::Time::min())
      return;


    const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * (localpose * gpstrans_);
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
  }

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
	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * pose_calibrated;// TODO:golbal_init_pose only yaw when using IMU. false
//	const ivcommon::transform::Rigid3d globalpose = localmapping_->global_init_pose().pose * localpose;
	double latitude,longitude;
    grid_to_geographic(a,e2,zone,hemi,
    		globalpose.translation()[1],globalpose.translation()[0] + 500000
		       ,&latitude,&longitude);

	covgrid_slam_msgs::OdometrywithGps laserOdometry;
	laserOdometry.odometry.header.frame_id = "global_earth_frame";
	laserOdometry.header.stamp = msg_header.stamp;
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
	laserOdometry.matched_probability = localmapping_->matched_probability();
	//		laserOdometry.
	pubLaserOdometry_.publish(laserOdometry);

	//		LOG(INFO)<<pose.pose.translation();
//	LOG(WARNING)<<std::fixed<<std::setprecision(3)<<"laserOdometrytime:"<<msg_header.stamp.toSec();
  }


  void logData()
  {
    if(lidarodometry_gpsdata_.time == ::ivcommon::Time::min())
      return;
    static std::string filename = log_path_ + "/preciseodometry.txt";
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

  void run()
  {
    processthreadfinished_ = false;
    processthread_=new boost::thread(boost::bind(&Node::processthread,this));
//    savethread_=new boost::thread(boost::bind(&Node::savecloudthread,this));
  }

  private:
  const float scanPeriod = 0.1;


  ivcommon::transform::Rigid3d gpstrans_;

  ::ros::NodeHandle node_handle_;

  ros::Publisher pubLaserOdometry_;

  ros::Subscriber subLaserCloud_;
  ros::Subscriber subImu_;
  ros::Subscriber subGps_;
  ros::Subscriber subEcu_;
  ros::Subscriber subInsVelocity_;
  ros::Subscriber subFusePose_;
//  ros::Subscriber subConfig;


  mapping3d::proto::LocalTrajectoryBuilderOptions options_;
  mapping3d::LocalTrajectoryBuilder* localmapping_;
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::ImuData>> imudatas_;
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::GpsInsData>> gpsdatas_;
  ::ivcommon::BlockingQueue<std::unique_ptr<sensor::EcuData>> ecudatas_;
  ::ivcommon::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> PointCloudMsgs_;
  sensor::GpsInsData lidarodometry_gpsdata_;

  ::ivcommon::BlockingQueue<std::unique_ptr<std::pair<pcl::PointCloud<pcl::PointXYZI>,ivcommon::transform::posestamped>>> savecloud_;
  boost::thread* processthread_=nullptr;
  std::fstream filestream_;
  std::string configstr_;
  XmlConf xml_conf_;
  std::string log_path_;

  double log_start_time_;
  constexpr static double a=6378137;
  constexpr static double e2= 0.0818192*0.0818192;//e的平方
  GridZone zone =UTM_ZONE_AUTO;
  Hemisphere hemi = HEMI_NORTH;
  std::string send_str_;

  sensor::LidarGpsStatus lidargpsstatus_;
  ros::Timer tftimer_;
  bool systemInited_;
  bool log_on_;
  bool processthreadfinished_;
  bool withgps_;
  bool withecu_;
  bool withinsvelocity_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "preciseodometrymain");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  google::InstallFailureSignalHandler();

  Node node;
  node.run();
  ros::spin();
  return 0;
}

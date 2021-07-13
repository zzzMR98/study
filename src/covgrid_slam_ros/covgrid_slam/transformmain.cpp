
#include <cmath>
#include "ivcommon/transform/rigid_transform.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <list>
#include <glog/logging.h>
#include <util/boostudp/boostudp.h>

#include "velodyne/HDL32Structure.h"
#include "ivcommon/common/blocking_queue.h"
#include "ivcommon/common/make_unique.h"
#include "src/velodyne/proto/lidar_options.pb.h"
#include "sensor_driver_msgs/OdometrywithGps.h"

#include "covgrid_slam/sensor/point_cloud.h"
#include "ivcommon/common/time_conversion.h"
#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906

class PostProcess
{
public:
	typedef std::pair<double,ivcommon::transform::Rigid3d> TimePosePair;
	PostProcess(ros::NodeHandle& nodehandle):currentDiffCloud_(new pcl::PointCloud<pcl::PointXYZI>)
	,currentBorderCloud_(new pcl::PointCloud<pcl::PointXYZI>)
	,ogm_data_back_(20,40,0.2,20,20)
	,nodehandle_(nodehandle)
	,processthread_(NULL)
	,send_flag_(false)
	,borderdetection_flag_ (false)
	,processthreadfinished_ (false)
	{
		init();
	}
	~PostProcess()
	{
	  lidarOdoms_.stopQueue();
	  processthreadfinished_ = true;
	  processthread_->join();
	}

	void init()
	{

		ros::param::get("~send_flag",send_flag_);
		ros::param::get("~borderdetection_flag",borderdetection_flag_);
		heightdiffthreshold_ = 0.3;
		ros::param::get("~heightdiff_threshold",heightdiffthreshold_);
		iskitti_ = false;
		ros::param::get("~iskitti",iskitti_);

		pubLaserOdometry2_ = nodehandle_.advertise<nav_msgs::Odometry> ("integrated_to_init", 5);
		laserOdometry2_.header.frame_id = "global_earth_frame";
		laserOdometry2_.child_frame_id = "vehicle_integrate_frame";

		laserOdometryTrans2_.frame_id_ = "global_earth_frame";
		laserOdometryTrans2_.child_frame_id_ = "vehicle_integrate_frame";

		subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
										 ("lidar_odometry_to_earth", 5, boost::bind(&PostProcess::laserOdometryHandler,this,_1));


		pubLaserCloudTotal_ = nodehandle_.advertise<sensor_msgs::PointCloud2>
											("velodyne_cloud_Total2", 1);
		if(0&&!iskitti_)
		{
			subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
												 ("lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudFullResHandler,this,_1));//经过筛选且转换之后的点云

			subLaserCloudBack_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
												 ("velodyne_cloud_back", 1, boost::bind(&PostProcess::laserCloudBackHandler,this,_1));//经过筛选且转换之后的点云
			processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
		}



		file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);


	}
	void SendData(OGMData<unsigned char>& ogmdata)
	{
		BoostUdp sendogmdata(LOCAL_IP,FROMLADAR_LOCAL_PORT);
		sendogmdata.connectRemoteEndpoint("192.168.0.111",9905);
	//	sendogmdata.connectRemoteEndpoint("127.0.0.1",9905);
		int lenth=2000;
		int package = ogmdata.ogmcell_size/lenth;
		int package_total;

		char ogm_total_data[ogmdata.ogmcell_size];
		int headernum = 3;
		char header[headernum];
		//memcpy(ogm_total_data,ogmdata.ogm,1);
		if(ogmdata.ogmcell_size - lenth * package>0)
		{
			package_total = package + 1;
		}
		for(int i =0;i < package;i++)
		{
			int ogmstart = lenth*i;
			char senddata[lenth+headernum];
			header[0]='a';
			header[1]=i;
			header[2]=package_total;
			memcpy(senddata,header,headernum);
			memcpy(senddata+headernum,ogmdata.ogm+ogmstart,lenth);
			sendogmdata.send(senddata,sizeof(senddata));
		}
		if(ogmdata.ogmcell_size - lenth * package>0)
		{
				int lenth_new = ogmdata.ogmcell_size - lenth * package;
				char senddata[lenth_new];
				header[0] = 0x88;
				header[1] = package;
				header[2] = package_total;
				memcpy(senddata,header,headernum);
				memcpy(senddata+headernum,ogmdata.ogm+lenth*package,lenth_new);
				sendogmdata.send(senddata,sizeof(senddata));
		}
	}

	void laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)
	{
	  double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
	  static double last_stamp = -1;
	//  static geometry_msgs::Quaternion last_geoQuat;
	  static ivcommon::transform::Rigid3d lasttransformodometry;
	//  static float last_trans[6];
	//  double roll, pitch, yaw;
	  geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

	  Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
	  Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
			  laserOdometry->odometry.pose.pose.position.y,
			  laserOdometry->odometry.pose.pose.position.z);

	  ivcommon::transform::Rigid3d transformodometry(translation,roatation);

	  laserOdometry2_.header.stamp = laserOdometry->odometry.header.stamp;
	  laserOdometry2_.pose.pose.orientation.x = geoQuat.x;
	  laserOdometry2_.pose.pose.orientation.y = geoQuat.y;
	  laserOdometry2_.pose.pose.orientation.z = geoQuat.z;
	  laserOdometry2_.pose.pose.orientation.w = geoQuat.w;

	  laserOdometry2_.pose.pose.position.x = laserOdometry->odometry.pose.pose.position.x;
	  laserOdometry2_.pose.pose.position.y = laserOdometry->odometry.pose.pose.position.y;
	  laserOdometry2_.pose.pose.position.z = laserOdometry->odometry.pose.pose.position.z;
	  pubLaserOdometry2_.publish(laserOdometry2_);

	  laserOdometryTrans2_.stamp_ = laserOdometry->odometry.header.stamp;

	  int n = 1;
	  double stamp = laserOdometry->odometry.header.stamp.toSec();
	  LOG(INFO)<<std::fixed<<std::setprecision(2)<<"stamp:"<<stamp;
	  if(last_stamp>0&&stamp-last_stamp>0.15)
	    n = (int)((stamp-last_stamp +0.04)/0.1);

	  for(int i =1;i<=n;i++)
	    {
		  Eigen::Quaterniond tempquat;
		  double rate = i/(double)n;
		  tempquat = lasttransformodometry.rotation().slerp(rate,transformodometry.rotation());
		  Eigen::Vector3d temptrans = (1-rate)*lasttransformodometry.translation()+rate*transformodometry.translation();

		//poses 的坐标系是　前ｚ下ｙ右ｘ
//	      tf::Matrix3x3 rotatemat(tf::Quaternion(tempquat.x(), -tempquat.z(), tempquat.y(), tempquat.w()));
	      Eigen::Quaterniond rotationquat (tempquat.w(),tempquat.x(),-tempquat.z(),tempquat.y());

	      Eigen::Matrix3d rotatemat= rotationquat.matrix();

	      file_<<std::setprecision(7)<<rotatemat(0,0)<<" "<<rotatemat(0,1)<<" "<<rotatemat(0,2)<<" "<<temptrans[0]
	    	<<" "<<rotatemat(1,0)<<" "<<rotatemat(1,1)<<" "<<rotatemat(1,2)<<" "<<-temptrans[2]
	    	<<" "<<rotatemat(2,0)<<" "<<rotatemat(2,1)<<" "<<rotatemat(2,2)<<" "<<temptrans[1]<<std::endl;

//	      file_<<std::setprecision(7)<<rotatemat.getRow(0).getX()<<" "<<rotatemat.getRow(0).getY()<<" "<<rotatemat.getRow(0).getZ()<<" "<<temptrans[0]
//	    	<<" "<<rotatemat.getRow(1).getX()<<" "<<rotatemat.getRow(1).getY()<<" "<<rotatemat.getRow(1).getZ()<<" "<<-temptrans[2]
//	    	<<" "<<rotatemat.getRow(2).getX()<<" "<<rotatemat.getRow(2).getY()<<" "<<rotatemat.getRow(2).getZ()<<" "<<temptrans[1]<<std::endl;

	    }

	  laserOdometryTrans2_.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
	  laserOdometryTrans2_.setOrigin(tf::Vector3(transformodometry.translation().x()
			  , transformodometry.translation().y(), transformodometry.translation().z()));
	  tfBroadcaster2_.sendTransform(laserOdometryTrans2_);
	  last_stamp = stamp;

	  lasttransformodometry = transformodometry;
	  if(!iskitti_)
		  lidarOdoms_.Push(::ivcommon::make_unique<TimePosePair>(timeOdometry,transformodometry));

	}


	void laserCloudBackHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudBack)
	{
	  pcl::PointCloud<pcl::PointXYZI> lasercloudback;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
	  pcl::fromROSMsg(*laserCloudBack, *tempcloud);//获取当前帧点云数据
	  int cloudnum = tempcloud->size() % 16;//包含的点云包数目
	  vector<int> startnum;
	  vector<double> x_offset_total;
	  vector<double> y_offset_total;
	  int lidar_layer;
	  double x_offset,y_offset;
	  CHECK(cloudnum==1);
	  for(int i =0;i<cloudnum;i++)
	  {
	      int flag = (*tempcloud)[tempcloud->size()-cloudnum+i].range;//每一包点云的第一个点的位置
	      (*tempcloud)[tempcloud->size()-cloudnum+i].range = -0.5;
	      x_offset = (*tempcloud)[tempcloud->size()-cloudnum+i].x;//每一包点云中对应的雷达在车体坐标系的x
	      y_offset = (*tempcloud)[tempcloud->size()-cloudnum+i].y;////每一包点云中对应的雷达在车体坐标系的y
	      lidar_layer = (*tempcloud)[tempcloud->size()-cloudnum+i].azimuth;//每一包点云中对应的雷达线束
	      startnum.push_back(flag);
	      x_offset_total.push_back(x_offset);
	      y_offset_total.push_back(y_offset);
	  }
	  for(int i = 0;i < startnum.size();i++)
	  {
	      int length;
	      if(i == startnum.size()-1)
	  	{
	  	  length = tempcloud->size() - cloudnum - startnum.at(i);
		}
	      else
	  	{
		  length = startnum.at(i+1) - startnum.at(i);
	  	}
	  	for(int j = 0;j < length; j++)
	  	  {
	  	  lasercloudback.push_back(tempcloud->at(startnum.at(i)+j));
	  	  }
		OGMData<unsigned char> ogm_data_single_border(20,40,0.2,20,20);
		memset(ogm_data_single_border.ogm,UNKNOWN,ogm_data_single_border.ogmcell_size);
	  	if(borderdetection_flag_)
	  	{
			pcl::PointCloud<pcl::PointXYZI> borderdetection;


			LidarProcess::border_detection(lasercloudback,borderdetection,ogm_data_single_border,lidar_layer,x_offset_total.at(i),y_offset_total.at(i));//对于每一包点云进行路沿检测
	  	}
	  	pcl::PointCloud<pcl::PointXYZI> diffdetection;
		OGMData<unsigned char> ogm_data_single_temp_close(20,40,0.2,20,20);
		LidarProcess::heightdiffOgmDetection(lasercloudback,diffdetection,ogm_data_single_temp_close,0.2,heightdiffthreshold_,1);
		for(int i =0;i < ogm_data_back_.ogmcell_size;i++)
		{
		  if(ogm_data_single_temp_close.ogm[i] == RIGIDNOPASSABLE
		      ||(ogm_data_single_border.ogm[i]==RIGIDNOPASSABLE))
		    ogm_data_back_.ogm[i] = RIGIDNOPASSABLE;
		  else
		    ogm_data_back_.ogm[i] = PASSABLE;
		}
		ogm_data_back_.updateStamp();
	  }
	}

	void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
	{
	  lidarCloudMsgs_.Push(laserCloudFullRes2);
	  if(lidarCloudMsgs_.Size()>1)
		  lidarCloudMsgs_.Pop();

	}

	void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys)
	{

		  //////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////
		  int cloudnum = inputcloud->size() % 16;//包含的点云包数目
		  vector<int> startnum;

		  for(int i =0;i<cloudnum;i++)
		  {
			  pcl::PointXYZI originpoint;
			  int flag = (*inputcloud)[inputcloud->size()-cloudnum+i].range;//每一包点云的第一个点的位置
			  (*inputcloud)[inputcloud->size()-cloudnum+i].range = -0.5;
			  originpoint.x = (*inputcloud)[inputcloud->size()-cloudnum+i].x;//每一包点云中对应的雷达在车体坐标系的x
			  originpoint.y = (*inputcloud)[inputcloud->size()-cloudnum+i].y;////每一包点云中对应的雷达在车体坐标系的y
			  originpoint.z = (*inputcloud)[inputcloud->size()-cloudnum+i].z;////每一包点云中对应的雷达在车体坐标系的z
			  originpoint.intensity = (*inputcloud)[inputcloud->size()-cloudnum+i].azimuth;//每一包点云中对应的雷达线束
			  startnum.push_back(flag);
			  lidarpropertys.push_back(originpoint);
		  }
		  for(int i = 0;i < startnum.size();i++)
		  {
			int length;
			pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloudptr(new pcl::PointCloud<pcl::PointXYZI>);//每一包点云

			if(i == startnum.size()-1)
			{
				length = inputcloud->size() - cloudnum - startnum.at(i);
			}
			else
			{
				length = startnum.at(i+1) - startnum.at(i);
			}

			lasercloudptr->insert(lasercloudptr->begin(),inputcloud->begin()+startnum.at(i),inputcloud->begin()+startnum.at(i)+length);
			outputclouds.push_back(lasercloudptr);

		  }

	}

	void process()
	{


		while(!processthreadfinished_)
		{
			const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.1));
			if(cloudmsg == nullptr)
				continue;

			double timeLaserCloudFullRes = cloudmsg->header.stamp.toSec();
			LOG(INFO)<<std::fixed<<std::setprecision(3)<<"cloudtime:"<<timeLaserCloudFullRes;
			LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
			Eigen::Vector3f origin=Eigen::Vector3f::Zero();
			{
			  //坐标转换的两个函数只能用给里程计坐标系的点云使用
			  //路沿检测的函数以及根据高度差判断的函数都是基于车体坐标系下使用的
			  pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
			  pcl::PointCloud<pcl::PointXYZI>::Ptr bordercloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧路沿检测点云（里程计坐标系）
			  pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
			  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
			  std::vector<pcl::PointXYZI> lidarpropertys;
			  analysisCloud(tempcloud,outputclouds,lidarpropertys);

			  //队列清空
			  if(diffCloudQueue_.size()>=TotalCloudNum)
				  diffCloudQueue_.pop_front();
			  if(BorderCloudQueue_.size()>=TotalCloudNum)
				  BorderCloudQueue_.pop_front();
			  //点云数据初始化
			  totalDiffCloud_.clear();
			  totalBorderCloud_.clear();
			  currentBorderCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
			  currentDiffCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
			  //////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////

			  for(int i = 0;i < lidarpropertys.size();i++)
			  {
				int length;
				pcl::PointCloud<pcl::PointXYZI>& lasercloud = *outputclouds[i];//每一包点云

				if(borderdetection_flag_)
				{
					pcl::PointCloud<pcl::PointXYZI> borderdetection;

					OGMData<unsigned char> ogm_data_single_border(80,40,0.2,20,20);

					LidarProcess::border_detection(lasercloud,borderdetection,ogm_data_single_border
							,int(lidarpropertys[i].intensity),lidarpropertys[i].x,lidarpropertys[i].y);//对于每一包点云进行路沿检测
					OGMFilter ogm_data_border_filter(80,40,0.1,20,20);
					for(int k = 0;k<borderdetection.size();k++)
					{
					 pcl::PointXYZI& temppoint = borderdetection.points[k];
					 //将检测出是路沿的点转成全局推入队列
					 if(temppoint.passibility <= 0.1&&ogm_data_border_filter.addPoint(temppoint.x,temppoint.y))
					 {
						 totalBorderCloud_.push_back(temppoint);
						 currentBorderCloud_->push_back(temppoint);//存放的是全局坐标系下当前帧路沿检测的点
					  }
					}
				}

				pcl::PointCloud<pcl::PointXYZI> tempcloudvehicle;

				//		  OGMData<unsigned char> ogm_data_single_temp(80,40,0.2,20,20);
				OGMData<unsigned char> ogm_data_single_temp_close(40,40,0.2,20,20);
				LidarProcess::heightdiffOgmDetection(lasercloud,tempcloudvehicle,ogm_data_single_temp_close,0.2,heightdiffthreshold_,1);
				OGMData<unsigned char> ogm_data_single_temp_far(40,40,0.2,-20,20);
				LidarProcess::heightdiffOgmDetection(lasercloud,tempcloudvehicle,ogm_data_single_temp_far,0.2,heightdiffthreshold_,0);
				//		  OGMData<unsigned char>::jointogm(ogm_data_single_temp_close,ogm_data_single_temp_far,ogm_data_single_temp);
				//	  LidarProcess::showOGM("heightdiff_single",ogm_data_single_temp);


				OGMFilter ogm_data_filter(80,40,0.1,20,20);

				for(int i = 0; i < tempcloudvehicle.size();i++)
				{
					pcl::PointXYZI& temppoint = tempcloudvehicle.points[i];

					if(temppoint.passibility < 0.1&&ogm_data_filter.addPoint(temppoint.x,temppoint.y))
					{
					  totalDiffCloud_.push_back(temppoint);

					  currentDiffCloud_->push_back(temppoint);

					}
				}

			  }

			}
			LOG(INFO)<<"wait lidarodom";
			auto timeposepair = lidarOdoms_.Pop();
			if(timeposepair==nullptr)
				continue;
			if(timeposepair->first-timeLaserCloudFullRes>0.005)
			{
				lidarOdoms_.Push_Front(std::move(timeposepair));
				continue;
			}

			while((timeposepair->first-timeLaserCloudFullRes)<-0.005)
				timeposepair = lidarOdoms_.Pop();

			LOG(INFO)<<"got lidarodom";
			if(timeposepair==nullptr)
				continue;

			if(fabs(timeposepair->first-timeLaserCloudFullRes)<0.005)
			{
//				sensor::RangeData rangedata={origin,FromRos(ros::Time(timeLaserCloudFullRes)),point_cloud,{}};
//				sensor::RangeData transform_rangedata = sensor::TransformRangeData(rangedata,timeposepair->second.cast<float>());
//				active_submaps_.InsertRangeData(transform_rangedata,Eigen::Quaterniond::Identity());
			  ivcommon::transform::Rigid3d transformodometry = timeposepair->second;
			  ivcommon::transform::Rigid3d transformodometry_inverse = transformodometry.inverse();
			  //历史帧数据在车体坐标系下的处理
			  for(std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it = diffCloudQueue_.begin();it != diffCloudQueue_.end();it++)
			  {
				  pcl::PointCloud<pcl::PointXYZI> tempcloud;
				  pcl::transformPointCloud(*(*it),tempcloud,transformodometry_inverse.translation()
						  ,transformodometry_inverse.rotation());
				  totalDiffCloud_ +=tempcloud;
			  }
			  //路沿点历史数据
			  OGMData<unsigned char> ogm_data_border(80,40,0.2,20,20);
			  memset(ogm_data_border.ogm,UNKNOWN,ogm_data_border.ogmcell_size);
			  if(borderdetection_flag_)
			  {
				  for(std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it = BorderCloudQueue_.begin();it != BorderCloudQueue_.end();it++)
				  {
					  pcl::PointCloud<pcl::PointXYZI> tempcloud;
					  pcl::transformPointCloud(*(*it),tempcloud,transformodometry_inverse.translation()
							  ,transformodometry_inverse.rotation());
					  totalBorderCloud_ += tempcloud;
				  }


				  pcl::transformPointCloud(*currentBorderCloud_,*currentBorderCloud_,transformodometry.translation()
							 ,transformodometry.rotation()); //里程计转成全局


				  LidarProcess::cloud2OGM(totalBorderCloud_,ogm_data_border,2);
				  LidarProcess::showOGM("total_border_frame",ogm_data_border);
				//////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////

			  }
			  pcl::transformPointCloud(*currentDiffCloud_,*currentDiffCloud_,transformodometry.translation(),transformodometry.rotation());

			  OGMData<unsigned char> ogm_data(80,40,0.2,20,20);
			//	pcl::PointCloud<pcl::PointXYZI> laserCloudvehicle=totalDiffCloud_;
			  LOG(INFO)<<"totalDiffCloud_ size="<<totalDiffCloud_.size();

			  //		  OGMData<unsigned char> ogm_data_single_temp(80,40,0.2,20,20);
			  OGMData<unsigned char> ogm_data_temp_close(40,40,0.2,20,20);
			  LidarProcess::cloud2OGM(totalDiffCloud_,ogm_data_temp_close,1);
			  OGMData<unsigned char> ogm_data_temp_far(40,40,0.2,-20,20);
			  LidarProcess::cloud2OGM(totalDiffCloud_,ogm_data_temp_far,0);
			  OGMData<unsigned char>::jointogm(ogm_data_temp_close,ogm_data_temp_far,ogm_data);
			//	  LidarProcess::cloud2OGM(totalDiffCloud_,ogm_data,2);
			//	  LidarProcess::showOGM("heightdiff_ogm",ogm_data);

			  OGMData<unsigned char> ogm_final_data(80,40,0.2,20,20);
			  for(int i =0;i < ogm_final_data.ogmcell_size;i++)
			  {
				if(ogm_data_border.ogm[i]==RIGIDNOPASSABLE
				||ogm_data.ogm[i]==RIGIDNOPASSABLE)
					ogm_final_data.ogm[i] = RIGIDNOPASSABLE;
				else
					ogm_final_data.ogm[i] = PASSABLE;
			  }

			  if(ogm_data_back_.timeIsValid())
				{
				for(int i =0;i < ogm_data_back_.ogmcell_size;i++)
				  {
				if(ogm_data_back_.ogm[i] == RIGIDNOPASSABLE)
				  ogm_final_data.ogm[i] = RIGIDNOPASSABLE;
				  }
				}
			  if(send_flag_)
				SendData(ogm_final_data);

			  LidarProcess::showOGM("totalogm",ogm_final_data);


			cv::waitKey(1);
			// cvReleaseMat(&slopemat);

			diffCloudQueue_.push_back(currentDiffCloud_);
			BorderCloudQueue_.push_back(currentBorderCloud_);

			//
			//	sensor_msgs::PointCloud2 laserCloudTotalMsg;
			//	pcl::toROSMsg(showcloud, laserCloudTotalMsg);
			//	laserCloudTotalMsg.header.stamp = ros::Time().fromSec(timeOdometry);
			//	laserCloudTotalMsg.header.frame_id = "/camera_init";
			//	pubLaserCloudTotal_.publish(laserCloudTotalMsg);
			LOG(INFO)<<"stoptime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
			}
		}

	}


	static constexpr int TotalCloudNum = 3;
	static constexpr int showCloudNum = 3;
	double heightdiffthreshold_;
	std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr> diffCloudQueue_;
	std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr> BorderCloudQueue_;

	pcl::PointCloud<pcl::PointXYZI> totalDiffCloud_;//历史帧点云+当前帧点云的融合（里程计坐标系）
	pcl::PointCloud<pcl::PointXYZI> totalBorderCloud_;//历史帧路沿检测+当前帧路沿检测点云的融合（里程计坐标系）
	pcl::PointCloud<pcl::PointXYZI>::Ptr currentDiffCloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr currentBorderCloud_;
	ros::Publisher pubLaserCloudTotal_;

	::ivcommon::BlockingQueue<std::unique_ptr<TimePosePair>> lidarOdoms_;
	::ivcommon::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;

	ros::Subscriber subLaserOdometry_ ;


	ros::Subscriber subLaserCloudFullRes_ ;//经过筛选且转换之后的点云

	ros::Subscriber subLaserCloudBack_ ;//经过筛选且转换之后的点云
	ros::Publisher pubLaserOdometry2_ ;
	tf::TransformBroadcaster tfBroadcaster2_;
	nav_msgs::Odometry laserOdometry2_;
	tf::StampedTransform laserOdometryTrans2_;

	std::fstream file_;
	boost::thread* processthread_;
//	mapping::ActiveStatisticsSubmaps active_submaps_;

	OGMData<unsigned char> ogm_data_back_;
	ros::NodeHandle& nodehandle_;
	bool send_flag_;
	bool borderdetection_flag_;
	bool processthreadfinished_ ;
	bool iskitti_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  PostProcess postprocess(nh);
  ros::spin();


  return 0;
}

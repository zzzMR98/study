/*!
* \file getbackvelodynedata.cpp
* \brief 雷达数据获取程序
*
*该文件是后置velodyne雷达数据获取程序
*各函数功能请参照getmultivelodynedata.cpp
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/23
*/
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <ros/package.h>
#include <glog/logging.h>
#include <sstream>
#include <fstream>

#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/date_time.hpp>
#include <boost/lambda/lambda.hpp>
#include <opencv2/opencv.hpp>
#include <util/playback/iv_data_playback.h>
#include <util/xmlconf/xmlconf.h>

#include "velodyne/myhdl_grabber.h"
#include "sensor_driver_msgs/startconfig.h"

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


class Lidardata{
public:
  Lidardata(const std::string& configstr):
    configstr_(configstr)
    , hdl_read_pcd_thread_ (NULL)
    , cloud_r_(new Cloud)
  {
    inited_ = false;
    init();
  };
  ~Lidardata()
  {
    close();
  }

  void close ()
  {

    cloud_connection_r_.disconnect ();

    if(replay_ != 2)
      {
    	grabber16_r_->stop();
      }

  }

  void cloud_callback (const CloudConstPtr& cloud,int id)
  {
	  stampr_ = cloud->header.stamp/1000000.0;
	  if(id==0)
	  {
		cloud_r_ = cloud;
		if(playback_.RecordIsOn())
			Record();
	  }

	  else if(id==1)
		  processcloud_r_ = *cloud;
	  if(id==1||replay_==2)
		  updated_ = true;

	  LOG(INFO)<<"callback";
  }


  void Record()
  {
    static int index=0;
    static std::string filename;
    if(cloud_r_)
    {
      playback_.BeginSaveLine();
      filename.clear();
      filename=playback_.MakeRecordFileName(index,".pcd");
      playback_<<stampr_<<filename;



      filename=playback_.GetRecordPath()+filename;
      pcl::io::savePCDFile(filename, *cloud_r_,true);
      index++;
      playback_.EndSaveLine();
    }

  }

  void getdatafrompcd()
  {
    std::string filename;

    LOG(INFO)<<"getdatafrompcd";
    while(inited_&&playback_.BeginLoadLine()==true)
      {

	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>);
	playback_>>stampr_>>filename;
	LOG(INFO)<<filename<<"\ttime:"<<stampr_;
	filename=playback_.GetPlaybackPath()+filename;
	pcl::io::loadPCDFile(filename,*laserCloud);
	pcl_conversions::toPCL(ros::Time::now(), laserCloud->header.stamp);//us

	cloud_callback(laserCloud,0);
//		    laserCloud.reset (new pcl::PointCloud<pcl::PointXYZI> ());
	playback_.EndLoadLine();
      }
  }
  bool init()
  {
    lasernum_ = 16;
    if(!xmlconfig_.Parse(configstr_.c_str(), "iv_lidar_back"))
    {
  	  std::cout<<"iv_lidar_back  is not exist in config xml  file"<<std::endl;
    }
    else
      {
	ConfigParam();
	float minDistanceThreshold = 1;
	float maxDistanceThreshold = 100;
	float velodyneheight = 1.2;

	float rigid_heightdiffthreshold = 0.2;

	playback_.Setup(xmlconfig_);
	if(playback_.PlaybackIsOn())
	{
		replay_=playback_mode;
	}
	else
	replay_=0;
	//replay_=1;

	 std::string pkgPath = ros::package::getPath("sensor_driver");
	if(lasernum_==32)
	  hdlCalibration_ = "";//"S2_64db.xml";//
	else if(lasernum_==64)
//		  hdlCalibration_ =pkgPath+ "/config/S2_64db.xml";//
	  hdlCalibration_ =pkgPath+ "/config/64S3db.xml";//塔河比赛丰田
	if(replay_==1 || replay_==2)
	{
		boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(hdlIP_);
		unsigned short  listen_port = hdlPortr_;
		unsigned short listen_port2 = hdlPortl_;

		if(replay_==1)
		{
			if(!xmlconfig_.GetModuleParam("pcapfile",pcapFile_))
			{
				std::cout<<"pcapfile is incorrect"<<std::endl;
			}
			else
			std::cout<<"pcapfile is "<<pcapFile_<<std::endl;

		}
		else
		  pcapFile_ = "";

		//对于第一个雷达的赋值，端口号和本地Ip
		grabber16_r_=new pcl::VelodyneGrabber(hdlCalibration_,pcapFile_,lasernum_);
		grabber16_r_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_r_->setMinimumDistanceThreshold(minDistanceThreshold);
		//grabber16_r_->filterPackets(listen_ip,listen_port);


	}
	else
	{

		std::cout<<"realtime start"<<std::endl;
		boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(hdlIP_);
		unsigned short  listen_port = hdlPortr_;
		unsigned short  listen_port2 = hdlPortl_;
		std::cout<<listen_port<<std::endl;
		grabber16_r_=new pcl::VelodyneGrabber(listen_ip, listen_port, hdlCalibration_, lasernum_);
		grabber16_r_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_r_->setMinimumDistanceThreshold(minDistanceThreshold);


	}
	pcl::get_transform_matrix(calibvaluer_,transform_matrix_calibration_R2V_);

	boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,int)> cloud_cb_r
	    = boost::bind(&Lidardata::cloud_callback,this,_1,_2);
	cloud_connection_r_ = grabber16_r_->registerCallback(cloud_cb_r);

        updated_ = false;
	inited_ = true;
        if(replay_ != 2)
          {
        	grabber16_r_->setCalibration(calibvaluer_);
            grabber16_r_->start ();
          }
        else
          {
            hdl_read_pcd_thread_ = new boost::thread (boost::bind (&Lidardata::getdatafrompcd, this));
          }


	return true;
      }
    return false;
  }
  void ConfigParam()
  {
  	bool autodriving;
  	//对于config.xml里面的参数的赋值
  	if(!xmlconfig_.GetModuleParam("lidar_port",hdlPortr_))
	{
		std::cout<<"port num is incorrect"<<std::endl;
	}
  	std::cout<<"port num is "<<hdlPortr_<<std::endl;


	if(!xmlconfig_.GetModuleParam("lidar_ip",hdlIP_))
	{
		std::cout<<"ip num is incorrect"<<std::endl;
	}
	else
	std::cout<<"ip num is "<<hdlIP_<<std::endl;

	if(!xmlconfig_.GetModuleParam("playback_on",playback_on_))
	{
		std::cout<<"playback_on_ is incorrect"<<std::endl;
	}
	else
	std::cout<<"playback_on_ is "<<playback_on_<<std::endl;

	if(!xmlconfig_.GetModuleParam("playback_mode",playback_mode))
	{
		std::cout<<"playback_mode is incorrect"<<std::endl;
	}
	else
	std::cout<<"playback_mode is "<<playback_mode<<std::endl;


	if(!xmlconfig_.GetModuleParam("alfa_r",calibvaluer_.alfa))
	{
		std::cout<<"alfa_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"alfa_r is "<<calibvaluer_.alfa<<std::endl;

	if(!xmlconfig_.GetModuleParam("beta_r",calibvaluer_.beta))
	{
		std::cout<<"beta_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"beta_r is "<<calibvaluer_.beta<<std::endl;

	if(!xmlconfig_.GetModuleParam("gama_r",calibvaluer_.gama))
	{
		std::cout<<"gama_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"gama_r is "<<calibvaluer_.gama<<std::endl;

	if(!xmlconfig_.GetModuleParam("x_offset_r",calibvaluer_.x_offset))
	{
		std::cout<<"x_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"x_offset_r is "<<calibvaluer_.x_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("y_offset_r",calibvaluer_.y_offset))
	{
		std::cout<<"y_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"y_offset_r is "<<calibvaluer_.y_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("z_offset_r",calibvaluer_.z_offset))
	{
		std::cout<<"z_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"z_offset_r is "<<calibvaluer_.z_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("lasernum",lasernum_))
	{
		std::cout<<"lasernum is incorrect"<<std::endl;
	}
	else
	std::cout<<"lasernum is "<<lasernum_<<std::endl;

  }




  static void reordercloud(Cloud& pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
  {
    const Cloud tempcloud = pointcloud;
    for(int laser_j=0 ; laser_j<lasernum ;laser_j++)
	{
	  int oriindex_j=indexmaptable[laser_j].number;
	  for(int i=0;i<tempcloud.size()/lasernum;i++)
	    {
	      int index = i*lasernum+laser_j;
	      int oriindex = i*lasernum+oriindex_j;
	      pointcloud.at(index) = tempcloud.at(oriindex);
	    }
	}
  }

  static void addcloudinfo(Cloud& pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
  {
    Cloud tempcloud;
    tempcloud.resize(lasernum);
    for(int laser_j=0 ; laser_j<lasernum ;laser_j++)
	{
	  int oriindex_j=indexmaptable[laser_j].number;
	  tempcloud.at(oriindex_j).x = laser_j+0.1;
	  tempcloud[oriindex_j].y = indexmaptable[laser_j].angle;
	  tempcloud[oriindex_j].range = -0.2;
	  tempcloud[oriindex_j].passibility = 1; //第二个点云的起点
	}
    pointcloud += tempcloud;
  }

  void mixcloud()
  {
    totalcloud_ .swap(processcloud_r_);
    pcl::PointXYZI temppoint;
    temppoint.x = calibvaluer_.x_offset;
    temppoint.y = calibvaluer_.y_offset;
    temppoint.z = calibvaluer_.z_offset;
    temppoint.azimuth=lasernum_; //线数
    temppoint.range = 0.5; //第二个点云的起点
    temppoint.passibility = 1;
    totalcloud_.push_back(temppoint);

    std::swap(totalcloud_.header , processcloud_r_.header);

  }


bool isvalid()
{
  return  updated_;
}

void resetcloudstate()
{
  updated_ = false;
}

static void  regionGrow(cv::Mat src,cv::Mat &matDst, cv::Point2i pt, float th)
{

   cv::Point2i ptGrowing;                      //待生长点位置
   int nGrowLable = 0;                             //标记是否生长过
   float nSrcValue = 0;                              //生长起点灰度值
   float nCurValue = 0;                              //当前生长点灰度值

   //生长方向顺序数据
  // int DIR[28][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0},{-5,0}, {5,0} ,{-6,0}, {6,0} ,{-7,0}, {7,0} ,{-8,0}, {8,0},{-9,0}, {9,0},{-10,0}, {10,0},{-11,0}, {11,0},{0,1}, {0,-1},{-1,1}, {-1,-1},{1,1}, {1,-1}};  //由近到远进行搜索
   int DIR[10][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0}, {0,1}, {0,-1}};  //由近到远进行搜索


   nSrcValue = src.at<float>(pt);            //记录生长点的灰度值
   matDst.at<uchar>(pt)=0;
   if (nSrcValue == 0)
   {
	matDst.at<uchar>(pt)=255;
       return;
   }

       //分别对八个方向上的点进行生长
       for (int i = 0; i<10; ++i)
       {
	   ptGrowing.x = pt.x + DIR[i][0];
	   ptGrowing.y = pt.y + DIR[i][1];
	   //检查是否是边缘点
	   if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x > (src.cols-1) || (ptGrowing.y > src.rows -1))
	       continue;


	       nCurValue = src.at<float>(ptGrowing);
/*
	       if (abs(nSrcValue - nCurValue) < th)
		{
		   matDst.at<uchar>(pt)=255;
		   return;
		}

*/
	       if(nCurValue == 0)
	       {
		   continue;
	       }

	       if(nSrcValue <= 2  )//在阈值范围内则生长
		  {
		    if (fabs(nSrcValue - nCurValue) < 0.2)
		     {
			matDst.at<uchar>(pt) += 1;
		     }

		     if(matDst.at<uchar>(pt)>=3)           //
		      {
			  matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			  return;
		      }
		  }
	       if(nSrcValue > 2 && nSrcValue <= 5   )
		 {
		   if (fabs(nSrcValue - nCurValue) < 0.5)
		    {
		       matDst.at<uchar>(pt) += 1;
		    }

		    if(matDst.at<uchar>(pt)>=2)           //
		     {
			 matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			 return;
		     }
		  }
	       if(nSrcValue >5)
		  {

		   if (fabs(nSrcValue - nCurValue) < 1)
		    {
		       matDst.at<uchar>(pt) += 1;
		    }

		    if(matDst.at<uchar>(pt)>=1)           //
		     {
			 matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			 return;
		     }
		  }



      }

 }




static void removeOutlier(pcl::PointCloud<pcl::PointXYZI> pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
 {
   int col_count = pointcloud.size()/lasernum;
   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
   pointcloud_filter->clear();
   pointcloud_filter->height = lasernum;
   pointcloud_filter->width = col_count;
   pointcloud_filter->is_dense = false;
   pointcloud_filter->resize(pointcloud_filter->height * pointcloud_filter->width);

   cv::Mat mat_depth = cv::Mat(cv::Size( col_count,lasernum), CV_32FC1, cv::Scalar(0));
   for(int i=0;i<col_count;i++)
   {
      for(int j=0;j<lasernum;j++)
      {
	    int oriindex_j=indexmaptable[j].number;
	    int oriindex = i*lasernum+oriindex_j;
	    if(pointcloud.points[oriindex].range<0)
	    {
		    pointcloud_filter->at(i, j) = pointcloud.points[oriindex];
		    mat_depth.at<float>(j , i) = 0;
	    }
	    else
	    {
		    pointcloud_filter->at(i, j) = pointcloud.points[oriindex];
		    mat_depth.at<float>(j , i) = pointcloud.points[oriindex].range;
	    }
      }
   }

    cv::Mat matMask = cv::Mat::zeros(mat_depth.size(), CV_8UC1);   //创建一个空白区域，填充为黑色
    //int removeNum =0 , nannum = 0;
    for(int row = 0; row < mat_depth.rows; row++)
      {
         for(int col = 0 ; col < mat_depth.cols ; col++ )
        {
           float d = mat_depth.at<float>(row,col);
           if( d > 10)
           {
               matMask.at<uchar>(row,col) = (uchar)255;
               continue;
           }
           //if(d == 0)
           //{
           //    nannum++;
           //}
           cv::Point2i pt(col,row);
           regionGrow(mat_depth,matMask,pt,1);
        }
     }
    //ROS_INFO("NAN: %d", nannum);
     for(int row = 0; row < matMask.rows; row++)
      {
         for(int col = 0 ; col < matMask.cols ; col++ )
         {
             if(matMask.at<uchar>(row,col) != 255)
             {
                //removeNum++;
        	 pointcloud_filter->at(col,row).x =0.1;
        	 pointcloud_filter->at(col,row).y =0.1;
        	 pointcloud_filter->at(col,row).z =0.1;
        	 pointcloud_filter->at(col,row).azimuth = -1000;
        	 pointcloud_filter->at(col,row).range = -0.1;
        	 pointcloud_filter->at(col,row).passibility = 1.0;
                //pointcloud.at(col,row).intensity= 255;
             }
             int index = row + col*matMask.rows;  //行列倒过来
             pointcloud.points[index] = pointcloud_filter->at(col,row);

         }
       }
 }

 void precomputecloud(Cloud& pointcloud,pcl::CalibrationValue& calibvalue)
 {
  int pointnum = pointcloud.points.size();
  if(replay_!=2)
  {
	for (int i = 0; i < pointnum; i++)
	{
		pcl::PointXYZI& temppoint = pointcloud.points[i];
		float azimuth = 90 - temppoint.azimuth  + calibvalue.gama;
		while(azimuth > 360.0) azimuth -=360.0;
		while(azimuth <=0) azimuth +=360.0;
		temppoint.azimuth=azimuth;
		temppoint.passibility = 1.0;
	}
  }
  else
  {
	  for (int i = 0; i < pointnum; i++)
	  {
	      //range < 0.5 ��Ч
		pcl::PointXYZI& temppoint = pointcloud.points[i];
	    double error=0.000001;
	    if(fabs(temppoint.x-0.1)<error&&fabs(temppoint.y-0.1)<error&&fabs(temppoint.z-0.1)<error)
	    {
		    temppoint.azimuth = -1000;
		    temppoint.range = -0.1;
		    temppoint.passibility = 1.0;
	    }
		else
		{
		double tempdistance=sqrt(temppoint.x * temppoint.x +
			temppoint.y * temppoint.y );
		temppoint.range = tempdistance;
//		LOG(INFO)<<temppoint.azimuth;
		float azimuth=atan2(temppoint.y,temppoint.x)*180/M_PI;
//		LOG(INFO)<<azimuth;
		azimuth = azimuth + calibvalue.gama;
		while(azimuth > 360.0) azimuth -=360.0;
		while(azimuth <=0) azimuth +=360.0;
		temppoint.azimuth=azimuth;

//		int mat_i=azimuth*10;

		temppoint.passibility = 1.0;
		}
	  }
  }

 }

 static void filtercloudonvehicle(Cloud& pointcloud,pcl::CalibrationValue& calibvalue)
 {
  for (int i = 0; i < pointcloud.points.size(); i++)
  {
      //range < 0.5 ��Ч
	  pcl::PointXYZI& temppoint = pointcloud.points[i];
      if(temppoint.range>0&&((temppoint.x > -fabs(1.5) && temppoint.x < fabs(1.5) &&
	  temppoint.y < 2 && temppoint.y > -2)))
      {
	  temppoint.range =- 0.01;
      }
//      else
//      {
//	float azimuth = processcloud->points[i].azimuth;
//	int mat_i=azimuth*10;
//	PolarPointDI temppolarpoint;
//	temppolarpoint.distance=processcloud->points[i].range;
//	temppolarpoint.index=i;
//	polaraxismat_[i%LASER_LAYER][mat_i]=temppolarpoint;
//      }
  }
 }

 void preprocess()
 {
   if(replay_==2)
   {
	   processcloud_r_=*cloud_r_;
	   precomputecloud(processcloud_r_,calibvaluer_);
	   if(0) //是否移除外点
	     {
	       removeOutlier(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);

	     }
	   else
	     {
	       //addcloudinfo(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);
	     }

	   pcl::transformPointCloud (processcloud_r_, processcloud_r_,transform_matrix_calibration_R2V_);
	   filtercloudonvehicle(processcloud_r_,calibvaluer_);
   }


   addcloudinfo(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);


 }
 const Cloud& gettotalcloud()
 {
   return totalcloud_;
 }
private:
  IvDataPlayback playback_;
  XmlConf xmlconfig_;
  double stamp_;
  std::string filename_;
  int lasernum_;
  pcl::CalibrationValue calibvaluer_;

  bool inited_;
  int replay_;
  bool show_window_;
  bool playback_on_;
  int playback_mode;
  bool record_on;
  bool oneframecalibmode_;
  bool saveoneframe_;
  std::string pcapFile_;
  std::string hdlCalibration_;
  std::string hdlIP_;
  int hdlPortr_;
  int hdlPortl_;
  std::string configstr_;
  pcl::VelodyneGrabber* grabber16_r_;

  bool updated_;
  CloudConstPtr cloud_r_;
  Cloud processcloud_r_;
  Cloud totalcloud_;
  boost::signals2::connection cloud_connection_r_;

  Eigen::Matrix4f transform_matrix_calibration_R2V_;

  boost::thread *hdl_read_pcd_thread_;
  double stampr_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "getbackvelodynedata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
//  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

  ros::Publisher pubLaserCloud;
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("lidar_cloud_back_calibrated", 2);

  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
   {
     ros::Duration(0.01).sleep();
   }
  bool fixframe = false;
  ros::param::get("~fix_frame",fixframe);
  std::string startconfig = configsrv.response.configstr;
  Lidardata lidar(startconfig);
  ros::Rate rate(1000);
  bool status = ros::ok();
  while(status)
    {
      if(lidar.isvalid())
        {
	  lidar.preprocess();
	  lidar.mixcloud();
	  lidar.resetcloudstate();
	  const Cloud& totalcloud= lidar.gettotalcloud();
	  LOG(INFO)<<"totalcloud.size="<<totalcloud.size();
	  sensor_msgs::PointCloud2 cloudmsg;
	  pcl::toROSMsg(totalcloud, cloudmsg);
	  LOG(INFO)<<" time:"<<(ros::Time::now() - ros::Time(cloudmsg.header.stamp)).toSec();
	  if(fixframe)
		  cloudmsg.header.frame_id = "global_init_frame";
	  else
		  cloudmsg.header.frame_id = "vehicle_frame";
	  pubLaserCloud.publish(cloudmsg);
	  LOG(INFO)<<"pub pointcloud";
        }
      else
	{
	  usleep(1000);
	}
//      rate.sleep();

      status = ros::ok();
    }

  return 1;

}

#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <glog/logging.h>
#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kittidata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  std::string sequence;
  ros::param::get("~sequence",sequence);
  std::string  basedir ;
  ros::param::get("~datasetdir",basedir);

  ros::Publisher pubLaserCloud;
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("lidar_cloud_calibrated", 2);
  //std::string  basedir = "/media/jkj/file/dataset/dataset";

  std::stringstream velodirname;
  velodirname<<basedir<<("/sequences/")<<sequence<<"/velodyne/";
  std::string velodir = velodirname.str();
  velodirname.clear();
  velodirname.str("");
  velodirname<<basedir<<("/sequences/")<<sequence<<"/calib.txt";
  std::string calibfilename =velodirname.str();
  std::ifstream calibfs(calibfilename.c_str());
  std::string line;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d traslation;
  while(getline(calibfs,line))
  {
	  std::stringstream sstr(line);
	  std::string header;
	  sstr>>header;
	  if(header == "Tr:")
	  {

	      sstr>>rotation(0,0)>>rotation(0,1)>>rotation(0,2)>>traslation[0]>>
				  rotation(1,0)>>rotation(1,1)>>rotation(1,2)>>traslation[1]>>
				  rotation(2,0)>>rotation(2,1)>>rotation(2,2)>>traslation[2];


	  }

  }
  ros::Rate rate(10);
  bool status = ros::ok();
  int count = 0;
  char filename[128];
  sleep(5);
  sprintf(filename,"%06d.bin",count);
  std::string filetotalpath = velodir + filename;
  int firstnum = 0;
  double stamp = ros::Time::now().toSec();
  while(status)
    {
      pcl::PointCloud<pcl::PointXYZI> velocloud[64];
      pcl::PointCloud<pcl::PointXYZI> pointcloud;
      pcl::PointCloud<pcl::PointXYZI> firsthalfcloud;
      pcl::PointCloud<pcl::PointXYZI> secondhalfcloud;
      std::fstream velofile;
      velofile.open(filetotalpath.c_str(),std::ios::binary|std::ios::in);
      if(!velofile)
	{
	  LOG(INFO)<<"not found file"<<filetotalpath.c_str();
	  sleep(5);
	  break;
	}
      Eigen::Vector3d valuevec;
      float value[4];
      float pointnum=0;
      int layer = 0;
      float azimuth;
      float azimuth_init;
      float last_azimuth_diff=0;
      float last_verticalangle;
      bool flag_firsthalf = true;
      while(!velofile.eof())
	{

	  velofile.read(reinterpret_cast<char *>(&value),sizeof(value));
	  valuevec<<value[0],value[1],value[2];
//	  LOG(INFO)<<value[0]<<" "<<value[1]<<" "<<value[2];
	  valuevec = rotation * valuevec + traslation;
	  pcl::PointXYZI temppoint;
	  temppoint.x = valuevec[0];
	  temppoint.y = valuevec[2];
	  temppoint.z = -valuevec[1];
	  temppoint.intensity = 0;
	  float azimuth = atan2(value[1],value[0])*180/M_PI;
	  if(pointnum==0)
	    {
	      azimuth_init=0;
	      LOG(INFO)<<"firstazimuth="<<azimuth;
	    }
	  if(azimuth<azimuth_init)
	    azimuth+=360;
	  float azimuth_diff = azimuth - azimuth_init;

//	  LOG(INFO)<<"pointnum="<<pointnum<<"\tlayer="<<layer<<"\tverticalangle="<<verticalangle;
	  if(pointnum>0&&azimuth_diff<last_azimuth_diff-180)
	    {
	      azimuth_init = 0;
	      layer++;

	      //LOG(INFO)<<"azimuth_diff="<<azimuth_diff<<"\tlast_azimuth_diff="<<last_azimuth_diff;
	      if(layer>=64)
	      {
	    	  layer--;
	    	  break;
	  		LOG(ERROR)<<"pointnum="<<pointnum<<"\tlayer="<<layer;
	      }


	    }


	  last_azimuth_diff = azimuth_diff;


	    velocloud[layer].push_back(temppoint);
	  pointnum++;

	}
      if(layer<63)
      LOG(WARNING)<<"pointnum="<<pointnum<<"\tlayer="<<layer;
      velofile.close();
      for(int j=0;j<64;j++)
	{
	  firsthalfcloud.clear();
	  secondhalfcloud.clear();
	  float last_azimuth = 0;
	  flag_firsthalf = true;
	  float move_y = 1;
	  float last_distance = 0;
	  int pointnum=0;
	  for(pcl::PointCloud<pcl::PointXYZI>::iterator it_point=velocloud[j].begin();it_point!=velocloud[j].end();it_point++)
	    {
	      pcl::PointXYZI temppoint = *it_point;

	      float azimuth = atan2(temppoint.y,temppoint.x);

	      temppoint.azimuth = azimuth*180/M_PI+180;
	      if(temppoint.azimuth>360)
		temppoint.azimuth-=360;
	      azimuth*=180/M_PI;
	      temppoint.range = sqrt(temppoint.x*temppoint.x+temppoint.y*temppoint.y);
	      if(azimuth<0)
		azimuth+=360;
	      if(temppoint.y>-2&&temppoint.y<2&&temppoint.x>-3&&temppoint.x<3)
		temppoint.range = -0.01;

	      if((azimuth>179&&azimuth<181)||(temppoint.x<0 && temppoint.y>-0.5&&temppoint.y<0.5))
		continue;
	      if(azimuth>180)
	      {
		flag_firsthalf=false;

	      }


	      temppoint.intensity = 63-j;

	      if(flag_firsthalf)
		firsthalfcloud.push_back(temppoint);
	      else
		secondhalfcloud.push_back(temppoint);
	      //pointcloud.push_back(temppoint);
	      pointnum++;
	    }
//	  LOG(INFO)<<secondhalfcloud[0].azimuth;
	  pointcloud += secondhalfcloud;
	  pointcloud += firsthalfcloud;
	}
      pcl::PointCloud<pcl::PointXYZI> tempcloud;

      tempcloud.swap(pointcloud);
      pointcloud.reserve(tempcloud.size());
      for(int i=tempcloud.size()-1;i>=0;i-=2)
	{
	  pointcloud.push_back(tempcloud.at(i));
	}
      static double laststamp = -1;
      double nowstamp = ros::Time::now().toSec();
      if(laststamp>0 && nowstamp - laststamp<0.1)
	{
	ros::Duration(0.1+laststamp-nowstamp).sleep();
	LOG(INFO)<<"sleep:"<<0.1+laststamp-nowstamp;
	}

      std_msgs::Header tempheader;
      stamp+=0.1;
      tempheader.stamp.fromSec(stamp);
      LOG(INFO)<<std::fixed<<std::setprecision(2)<<count<<"stamp="<<stamp;
      sensor_msgs::PointCloud2 velocloudMsg;
      pcl::toROSMsg(pointcloud, velocloudMsg);
      velocloudMsg.header.stamp = tempheader.stamp;
      velocloudMsg.header.frame_id = "vehicle_frame";
      //velocloudMsg.header.seq = 0;

      laststamp = stamp;

      pubLaserCloud.publish(velocloudMsg);
      if(firstnum>0)
	firstnum--;
      else
	count++;
      sprintf(filename,"%06d.bin",count);
      filetotalpath = velodir + filename;

      status = ros::ok();
    }

}

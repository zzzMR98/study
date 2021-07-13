#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sensor_msgs/NavSatFix.h"
#include"geometry_msgs/Vector3Stamped.h"
#include"sensor_msgs/NavSatStatus.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include <nav_msgs/Odometry.h>

#include "gps/AnalysisGPS.h"
#include "util/ToXml.hh"
#include <ros/package.h>
#include <common/common.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <util/playback/iv_data_playback.h>
#include <util/ToXml.hh>
#include <util/utm/utm.h>
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/VehicleState.h"

struct struct_GPS_Mess
{
	double dLongitude;
	double dLatitude;
	double Altitude;
	double HDOP;
	double VTG_V;
	double VTG_heading;
        double  PHDT_heading = 0;
	int DGPSState;
	double PTRA_heading;
	char DGPSState_VTG;
	double PASHR_heading;
    uint32_t gps_week;
    uint64_t gps_ms;  				//时间戳 ms

	double	lTimeStamp;			// 时间戳(Unit:ms)
};


class run_model {

public:
  run_model(std::string configstr):configstr_(configstr) {
    init();
  }
 // ~run_model();
  bool init()
  {
    framenum = 0;
     Module_On=false;
    autoMapCounter=0;
    autoMapIndex=0;
    starttime_counter = 0;
    CreatListenToGPS = false;
    start_point.x = 0;
    start_point.y = 0;
    if(!Module_On)//qjy,20170630。只进行一次INIT
  {
	  if(LoadConfigFile(configstr_.c_str())==true)
	  {
//			if(Module_On)
		  {
			  std::cout<<"Get  GPS port sucess  ,the number   is "<<port<<std::endl;
			  CreatListenToGPS=m_AnalysisGPS.Init(port);
			  if(CreatListenToGPS)
			  std::cout<<" 监听GPS端口成功 "<<std::endl;
			  else
			  std::cout<<"监听GPS端口失败 "<<std::endl;
		  }

		    return true;
	  }
	  else
	  std::cout<<"Get  GPS port failed  "<<std::endl;

  }
    return false;
  }
  struct_GPS_Mess * GetGPSData(){
    return & m_GPS_Data;
  }

  inline const state_struct&  gpsmetricdata() const{
    return gps_metric_data_;
  }

  bool process()

  {

	if(Module_On)
	{
		if(playback.PlaybackIsOn())
		{
			//std::cout<<"回放模式11"<<std::endl;
			bool succeed = my_playback();
			if(!succeed)
			  return false;
			//std::cout<<"回放模式"<<std::endl;
			/*if(autoMapCounter++==20)
			{

				FILE *fp2 = fopen("roadmap_generate_online_gps.txt", "a");
				fprintf(fp2, "%d\t%.6f\t%.6f\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\n",++autoMapIndex,ms_data_gps_data.GPSData_struct.dLongitude,ms_data_gps_data.GPSData_struct.dLatitude,5,10,3.5
					,2,3,1,20);
				fclose(fp2);
				autoMapCounter=0;
			}*/

		}
		else
		{
			m_AnalysisGPS.Update();//数据更新
			m_GPS_Data.dLatitude=	m_AnalysisGPS.GPSData_struct.dLatitude;
			m_GPS_Data.dLongitude= m_AnalysisGPS.GPSData_struct.dLongitude;
			m_GPS_Data.Altitude = m_AnalysisGPS.GPSData_struct.Altitude;
			m_GPS_Data.HDOP=m_AnalysisGPS.GPSData_struct.HDOP;
			m_GPS_Data.PHDT_heading=m_AnalysisGPS.GPSData_struct.PHDT_heading;
			m_GPS_Data.VTG_V=m_AnalysisGPS.GPSData_struct.VTG_V;
			m_GPS_Data.VTG_heading=m_AnalysisGPS.GPSData_struct.VTG_heading;
			m_GPS_Data.DGPSState=m_AnalysisGPS.GPSData_struct.DGPSState;
			m_GPS_Data.PTRA_heading=m_AnalysisGPS.GPSData_struct.PTRA_heading;
			m_GPS_Data.DGPSState_VTG=m_AnalysisGPS.GPSData_struct.DGPSState_VTG;
			m_GPS_Data.PASHR_heading=m_AnalysisGPS.GPSData_struct.PASHR_heading;//海梁项目.qjy 20170928
            m_AnalysisGPS.UTC2GPS(m_AnalysisGPS.GPSData_struct.year, m_AnalysisGPS.GPSData_struct.month, m_AnalysisGPS.GPSData_struct.day,
                                  m_AnalysisGPS.GPSData_struct.hour, m_AnalysisGPS.GPSData_struct.min, m_AnalysisGPS.GPSData_struct.second,
                                  &m_GPS_Data.gps_week, &m_GPS_Data.gps_ms);
			//			temptime=etime();
			m_GPS_Data.lTimeStamp=playback.SysTime();
			/*if(autoMapCounter++==GENERATROADMAPCOUNTER)
			{

				FILE *fp2 = fopen("roadmap_generate_online_gps.txt", "a");
				fprintf(fp2, "%d\t%.6f\t%.6f\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\n",++autoMapIndex,ms_data_gps_data.GPSData_struct.dLongitude,ms_data_gps_data.GPSData_struct.dLatitude,5,10,3.5
					,2,3,1,20);
				fclose(fp2);
				autoMapCounter=0;
			}*/

		}
		/*
		ms_data_gps_data.MessageValid_pos=true;
		ms_data_gps_data.MessageValid_head=false;
		#ifdef BYDRAYYUANZHENG
		ms_data_gps_data.MessageValid_head=true;
		#endif
		#ifdef MIDBUS_HAILIANG
		ms_data_gps_data.MessageValid_head=true;
		#endif


	//#define INTEGRATENAV//开启组合导航宏定义,输入：GPS，前轮偏角，车速。
	#ifdef INTEGRATENAV
		MS_DATA_ECU_CHANNEL->read();


	#endif

*/



		//MS_DATA_GPS_CHANNEL->write(ms_data_gps_data);//数据输出

	}
    	if(Module_On)
	{
    	    static const double error = 0.000001;
    	    if(fabs(m_GPS_Data.dLatitude)<error&&fabs(m_GPS_Data.dLongitude)<error)
    	      {
    		std::cerr<<"gps datas are zero totally";
    		return false;
    	      }


//	    double tem_x,tem_y;
//
//	    double N,E;
//	    GridZone zone =GRID_AUTO;
//	    Hemisphere hemi = HEMI_NORTH;
//	    geographic_to_grid(a, e2, m_GPS_Data.dLatitude*M_PI/180, m_GPS_Data.dLongitude*M_PI/180, &zone, &hemi, &N, &E);
//
//	    gps_metric_data_.position.x = E - 500000;
//	    gps_metric_data_.position.y = N ;

//    	   if(log_on_&&!log_path_.empty())
//    	     saveMetricData();
	  if (playback.RecordIsOn())
	  {
		  my_record();
		  return true;
	  }
	}
	return true;
  }


private:
  	bool LoadConfigFile(const char* configstr){

	if(!config.Parse(configstr, "GetGPSData"))
	{
		std::cout<<"GetGPSData  is not exist in config xml  file"<<std::endl;
		return false;
	}
	//	ConfigParam();

	playback.Setup(config);
	ConfigParam();

	if(playback.RecordIsOn())
	{
		InitRecordFileTitle();
	}
	return true;
	}
	void ConfigParam(){

	if(!config.GetModuleParam("port",port))
	{
		std::cout<<"端口设置错误"<<std::endl;
	}

	if(!config.GetSystemParam("GetGPSData_on",Module_On))
	{
		std::cout<<"GetGPSData模块的开关不存在"<<std::endl;
	}
	else
	{
		if(Module_On)
		std::cout<<"GetGPSData 模块开启"<<std::endl;
		else
		std::cout<<"GetGPSData 模块关闭"<<std::endl;
	}

	log_on_ = false;

	if(!config.GetSystemParam("log_on",log_on_))
	{
		std::cout<<"log_on 不存在"<<std::endl;
	}

	if(!config.GetSystemParam("log_path",log_path_))
	{
		std::cout<<"log_path 不存在"<<std::endl;
	}

	}

	bool my_playback(){

	if(playback.BeginLoadLine()==true)
	{
		playback>>m_GPS_Data.lTimeStamp;

		playback>>m_GPS_Data.dLatitude;
		playback>>m_GPS_Data.dLongitude;
		m_GPS_Data.dLatitude=m_GPS_Data.dLatitude*0.01;
		m_GPS_Data.dLongitude=m_GPS_Data.dLongitude*0.01;

		playback>>m_GPS_Data.PHDT_heading;
		playback>>m_GPS_Data.HDOP;//暂缓
		playback>>m_GPS_Data.VTG_heading;
		playback>>m_GPS_Data.VTG_V;
		playback>>m_GPS_Data.DGPSState;
		playback>>m_GPS_Data.PASHR_heading;

		playback.EndLoadLine();
		return true;
	}
	else
	  return false;
	}
	//void Realtime();
	void my_record()
	{
	  playback.BeginSaveLine();
	playback<<m_GPS_Data.lTimeStamp;
{


	playback<<(m_GPS_Data.dLatitude*100.0);//为解决默认存储的小数点位数是5，不能满足精度要求的问题。qjy，20170508
	playback<<(m_GPS_Data.dLongitude*100.0);
	playback<<m_GPS_Data.PHDT_heading;
	playback<<m_GPS_Data.HDOP;//20170904,qjy。
	playback<<m_GPS_Data.VTG_heading;
	playback<<m_GPS_Data.VTG_V;
	playback<<m_GPS_Data.DGPSState;

	playback<<m_GPS_Data.PASHR_heading;//qjy 20170928
    }

	playback.EndSaveLine();
	}

	void saveMetricData()
	{
	    static std::string filename = log_path_ + "/gpsmetricdata.txt";
	    static std::ofstream file(filename.c_str());

	    framenum++;
	    current_vehicle_positon.x = gps_metric_data_.position.x;
	    current_vehicle_positon.y = gps_metric_data_.position.y;
	    file<<std::fixed<<std::setprecision(3)<<m_GPS_Data.lTimeStamp<<" "
	    <<current_vehicle_positon.x<<" "
	    <<current_vehicle_positon.y<<std::endl;

	}

	void InitRecordFileTitle(){
	  playback.BeginSaveTitle();
	playback<<"time_stampe";
	//playback<<"dLatitude"<<"dLongitude"<<"PHDT_heading"<<"VTG_heading"<<"VTG_V"<<"DGPSState";
	playback<<"relative_x"<<"relative_y";
	playback.EndSaveTitle();
	}
	//	bool LoadConfigFile();
	//	void ConfigParam();

	int starttime_counter;
	int autoMapCounter;
	int autoMapIndex;
	std::string configstr_;
	XmlConf config;
	int port;
	ANALYSIS_GPS::CAnalysisGPS m_AnalysisGPS;
	bool CreatListenToGPS;
	bool Module_On;

	IvDataPlayback playback;
	struct_GPS_Mess m_GPS_Data;
	int framenum;
	point start_point;
	point current_vehicle_positon;
	bool log_on_;
	std::string log_path_;
	state_struct gps_metric_data_;
	const  double a=6378137;
	const double e2= 0.0818192*0.0818192;//e的平方


};


int main(int argc,char ** argv)
{
  ros::init(argc,argv,"getgpsdata");
  ros::NodeHandle nh;

  ros::Publisher GPS_Pub = nh.advertise<sensor_driver_msgs::GpswithHeading> ("GPSmsg", 50);
  ros::Publisher VehicleState_pub = nh.advertise<sensor_driver_msgs::VehicleState>("vehiclestate_GPS", 50);

//  ros::Publisher GPS_Pub = nh.advertise<nav_msgs::Odometry> ("/gpsdata", 50);
//  ros::Publisher GPS_Pub = nh.advertise<sensor_msgs::NavSatFix>("gpsdata",50);
//   ros::Publisher GPS_Pub = nh.advertise<geometry_msgs::Vector3Stamped>("gpsdata",50);
//  sensor_msgs::NavSatFix gps;
//   geometry_msgs::Vector3Stamped gps;
  nav_msgs::Odometry gps;
//   geometry_msgs::Vector3Stamped pos0;
   //int framenum = 0;

   ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
   sensor_driver_msgs::startconfig configsrv;

   while(!configclient.call(configsrv))
    {
      ros::Duration(0.01).sleep();
    }

   std::string startconfig = configsrv.response.configstr;
   run_model m_run_model(startconfig);


  bool state = ros::ok();
  struct_GPS_Mess * tem_gps_data;
  const  double a=6378137;
  const double e2= 0.0818192*0.0818192;//e的平方
  GridZone zone =UTM_ZONE_49;
  Hemisphere hemi = HEMI_NORTH;
  state_struct tem_point;

    while(state)
  {

    bool success = m_run_model.process();

    if (!success){
      ROS_ERROR_STREAM("gpsmodule process error");
      usleep(100000);
      state = ros::ok();
      continue;
    }
    tem_gps_data = m_run_model.GetGPSData();
//    double tem_x,tem_y;
//    double N,E;
//    geographic_to_grid(a, e2, tem_gps_data->dLatitude*M_PI/180, tem_gps_data->dLongitude*M_PI/180, &zone, &hemi, &N, &E);
//    tem_point = Position_Trans_From_ECEF_To_UTM (tem_gps_data->dLatitude,tem_gps_data->dLongitude,tem_x,tem_y);
//    if(framenum==0)
//	{
//	  pos0.vector.x = tem_point.position.x;
//	  pos0.vector.y = tem_point.position.y;
//
//	}
//      framenum++;
      //geometry_msgs::Vector3Stamped pos;
     //pos.header.stamp = imuout.header.stamp;
     // pos.vector.x = E - pos0.vector.x;
      //pos.vector.z = - N - pos0.vector.z;

//    std::cout<<std::fixed<<std::setprecision(7)<<tem_gps_data->dLatitude<<" "<<tem_gps_data->dLongitude<<std::endl;
//    std::cout<<lat*180/M_PI<<" "<<lon*180/M_PI<<std::endl;

  sensor_driver_msgs::GpswithHeading gps;
  gps.header.frame_id = "gps_frame";
  gps.header.stamp = ros::Time::now();
  gps.gps.header = gps.header;

  gps.gps.latitude = tem_gps_data->dLatitude;
  gps.gps.longitude = tem_gps_data->dLongitude;
   double heading_angle= tem_gps_data->PHDT_heading;
	if(heading_angle<=0)
	  heading_angle = -heading_angle;
	else
	  heading_angle = 360 -heading_angle;
	if(heading_angle>180)
		heading_angle -= 360;

  gps.heading = heading_angle;
  gps.gps.altitude = tem_gps_data->Altitude;
  gps.roll = tem_gps_data->HDOP;
  gps.pitch = tem_gps_data->DGPSState;
  GPS_Pub.publish(gps);

  sensor_driver_msgs::VehicleState vehicleState;
  vehicleState.header = gps.header;
  vehicleState.gps_week = tem_gps_data->gps_week;
  vehicleState.gps_ms = tem_gps_data->gps_ms;
  vehicleState.gps = gps.gps;
  VehicleState_pub.publish(vehicleState);

  state = ros::ok();
  }
}




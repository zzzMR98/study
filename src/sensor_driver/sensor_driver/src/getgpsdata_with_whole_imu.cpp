#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatStatus.h"
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

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>
#include <glog/logging.h>
#include <util/playback/iv_data_playback.h>
#include <util/utm/datum.h>
#include <util/utm/utm.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <serial/serial.h>
#include "ivcommon/transform/rigid_transform.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/InsVelocity.h"
//#include "ivcommon/transform/rigid_transform.h"
#ifdef TANGPLATFORM_76GF6//如果是无人唐
#include "ins/AnalysisINS_Octans.h"
#endif
#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
#include "ins/imu531.h"
#endif
#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
#include "ins/AnalysisINS_OxT.h"
#endif

struct struct_INS_Mess
{

    double lTimeStamp;			// 时间戳(Unit:ms)
    double dLongitude;
    double dLatitude;
    double dAltitude;
    double dHeading;
    double dPitch;
    double dRoll;

    double dAccx;
    double dAccy;
    double dAccz;

    double dArx;	//Pear
    double dAry;
    double dArz;

	double dVelE;  //velocity
	double dVelN;
	double dVelSky;

    int dState;
};

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

class InsModule{
public:
    serial::Serial ser;


  InsModule(std::string configstr):configstr_(configstr)
  {
    init();
  }
  bool init()
  {
    Module_On=false;
    autoMapCounter=0;
    autoMapIndex=0;
    serial_on = false;
    if(!Module_On)//qjy,20170630。只进行一次INIT
    {
	if(LoadConfigFile(configstr_.c_str())==true)
	{
	    //			imu.ImuInit(port);
	    //			halt=false;
	    std::cout<<"Get  INS port sucess  ,the number   is "<<port<<std::endl;

	    #ifdef TANGPLATFORM_76GF6//如果是无人唐

	    CreatListenToINS=m_AnalysisINS.Init(port);

	    #endif

	    #if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
	    if(serial_on)
            {
                CreatListenToINS = m_INSFons.SerialInit(comport_, baudrate_, timeout_ms_, port);
            }
//                CreatListenToINS = m_INSFons.SerialInit(1, port);
	    else
	    	CreatListenToINS=m_INSFons.ImuInit(port);
	    #endif

	    #if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
	    CreatListenToINS=m_AnalysisINS_OxT.Init(port);
	    #endif

	    if(CreatListenToINS)
	    std::cout<<" Creat ListenTo INS  Sucess "<<std::endl;
	    else
	    std::cout<<"Creat ListenTo INS   failure "<<std::endl;
	}
	else
	std::cout<<"Get  INS port failed  "<<std::endl;
    }
    return Module_On;
  }

  bool process()
  {
    if(Module_On)
    {

	#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
	if(playback.PlaybackIsOn())
	{
		my_playback();
		if(autoMapCounter++==100)//每5s存储1行路网文件
		{

			FILE *fp2 = fopen("roadmap_offline_atuo.txt", "a");
			fprintf(fp2, "%d\t%.6f\t%.6f\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\n",++autoMapIndex,ins_data_.dLongitude,ins_data_.dLatitude,5,10,3.5
				,2,3,1,10);
			fclose(fp2);
			autoMapCounter=0;
		}

	}
	else
	{
		m_AnalysisINS_OxT.Update();
		ins_data_.dHeading=360-m_AnalysisINS_OxT.INSData_struct.dHeading;
		ins_data_.dPitch=m_AnalysisINS_OxT.INSData_struct.dPitch;
		ins_data_.dRoll=m_AnalysisINS_OxT.INSData_struct.dRoll;

		ins_data_.dLatitude=m_AnalysisINS_OxT.INSData_struct.dLat;
		ins_data_.dLongitude=m_AnalysisINS_OxT.INSData_struct.dLng;
		ins_data_.dAltitude=m_AnalysisINS_OxT.INSData_struct.dAltitude;
		ins_data_.dVelE=m_AnalysisINS_OxT.INSData_struct.dEastV;
		ins_data_.dVelN=m_AnalysisINS_OxT.INSData_struct.dNorthV;
		ins_data_.dVelSky=m_AnalysisINS_OxT.INSData_struct.dAltitudeV;
		ins_data_.dAccx=m_AnalysisINS_OxT.INSData_struct.dAccy;
		ins_data_.dAccy=m_AnalysisINS_OxT.INSData_struct.dAccx;
		ins_data_.dAccz=-m_AnalysisINS_OxT.INSData_struct.dAccz;
		ins_data_.dArx=m_AnalysisINS_OxT.INSData_struct.dGyroY;
		ins_data_.dAry=m_AnalysisINS_OxT.INSData_struct.dGyroX;
		ins_data_.dArz=-m_AnalysisINS_OxT.INSData_struct.dGyroZ;
		ins_data_.lTimeStamp=playback.SysTime();
//		if(autoMapCounter++==100)////每5s存储1行路网文件
//		{
//
//			FILE *fp2 = fopen("roadmap_online_atuo.txt", "a");
//			fprintf(fp2, "%d\t%.6f\t%.6f\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\n",++autoMapIndex,ins_data_.dLongitude,ins_data_.dLatitude,5,10,3.5
//				,2,3,1,10);
//			fclose(fp2);
//			autoMapCounter=0;
//		}
	}

	if (playback.RecordIsOn())
	{
		my_record();
	}

	#endif

	#ifdef TANGPLATFORM_76GF6//如果是无人唐
	if(playback.PlaybackIsOn())
	{
		my_playback();

	}
	else
	{
		m_AnalysisINS.Update();
		ins_data_.dHeading=m_AnalysisINS.INSData_struct.dHeading;// Octans
		insdata_.dPitch = m_AnalysisINS.INSData_struct.dPitch;
		insdata_.dRoll = m_AnalysisINS.INSData_struct.dRoll;
		ins_data_.lTimeStamp=playback.SysTime();
	}
	if (playback.RecordIsOn())
	{
		my_record();
	}

	#endif

	#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐

	if(playback.PlaybackIsOn())
	{
		//std::cout<<"回放模式11"<<std::endl;
		my_playback();
		//std::cout<<"回放模式"<<std::endl;

	}
	else
	{
		if(serial_on)
			m_INSFons.UpdateBySerial();
		else
			m_INSFons.Update();
		unsigned int utc_time = m_INSFons.m_sFONSData.utc_time;
		//LOG(WARNING)<< " UTC time from INS : " << m_INSFons.m_sFONSData.utc_time;
		int ms = utc_time%1000;
		utc_time /=1000;
		int s = utc_time%100;
		utc_time /=100;
		int min = utc_time%100;
		utc_time /=100;
		int h = utc_time%100 + 8;

		LOG(INFO)<<std::setfill('0')<<std::setw(2)<<h<<"-"
				<<std::setw(2)<<min<<"-"
				<<std::setw(2)<<s<<":"
				<<std::setw(3)<<ms;

		struct timeval t1;
		gettimeofday(&t1,NULL);

		tm* pTmp=localtime(&t1.tv_sec);
		char readable_start_time[100];
		memset(readable_start_time,0,21);
		sprintf(readable_start_time, "%02d-%02d-%02d:%06d",
		  pTmp->tm_hour,
		  pTmp->tm_min,
		  pTmp->tm_sec
		  ,t1.tv_usec);
		LOG(INFO)<<readable_start_time;
		double delaytime = (pTmp->tm_sec - s)+(t1.tv_usec/1000-ms)/1000.;
//		LOG(WARNING)<<"delay:"<<delaytime<<"s";
//		if(delaytime>0.08)
//			LOG(ERROR)<<"delay:"<<delaytime<<"s";
		ins_data_.dHeading=m_INSFons.m_sFONSData.dHeading;

//		if(ins_data_.dHeading<0.0)
//		ins_data_.dHeading=-ins_data_.dHeading;
//		else
//		ins_data_.dHeading=360-ins_data_.dHeading;

		ins_data_.dRoll=m_INSFons.m_sFONSData.dRoll;
		ins_data_.dPitch=m_INSFons.m_sFONSData.dPitch;


		ins_data_.dLatitude=m_INSFons.m_sFONSData.dFOSNLat;
		ins_data_.dLongitude=m_INSFons.m_sFONSData.dFOSNLng;
		ins_data_.dAltitude=m_INSFons.m_sFONSData.dFOSNAltitude;

		ins_data_.dAccx=m_INSFons.m_sFONSData.dAccz;
		ins_data_.dAccy=m_INSFons.m_sFONSData.dAccx;
		ins_data_.dAccz=m_INSFons.m_sFONSData.dAccy;
		ins_data_.dArx=m_INSFons.m_sFONSData.dArz;
		ins_data_.dAry=m_INSFons.m_sFONSData.dArx;
		ins_data_.dArz=m_INSFons.m_sFONSData.dAry;

		ins_data_.dVelE=m_INSFons.m_sFONSData.dVelE;
		ins_data_.dVelN=m_INSFons.m_sFONSData.dVelN;
		ins_data_.dVelSky=m_INSFons.m_sFONSData.dVelSky;
        ins_data_.dState = m_INSFons.m_sFONSData.dFOSNState;

		ins_data_.lTimeStamp=playback.SysTime();
		if(autoMapCounter++==100)////每5s存储1行路网文件
		{

			FILE *fp2 = fopen("roadmap_online_atuo.txt", "a");
			fprintf(fp2, "%d\t%.6f\t%.6f\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\n",++autoMapIndex,ins_data_.dLongitude,ins_data_.dLatitude,5,10,3.5
				,2,3,1,10);
			fclose(fp2);
			autoMapCounter=0;
		}

	}
	if (playback.RecordIsOn())
	{
		my_record();
	}
	#endif

	return true;
    }
    return false;
  }

    bool LoadConfigFile(const char* configstr)
    {
    	//memset(m_sys_start_data->xmlconf,0,10000);
    	if(!config.Parse(configstr, "GetINSData"))
    	{
    		std::cout<<"GetINSData  is not exist in config xml  file"<<std::endl;
    		return false;
    	}


    	playback.Setup(config);
    	ConfigParam();

    	if(playback.RecordIsOn())
    	{
    		InitRecordFileTitle();
    	}
    	return true;

  }

  void ConfigParam()
  {
      //float parm;
      if(!config.GetModuleParam("port",port))
      {
	      std::cout<<"port num is incorrect"<<std::endl;
      }
      if(!config.GetModuleParam("serial_on",serial_on))
      {
	      std::cout<<"serial_on is incorrect"<<std::endl;
      }
      if(!config.GetModuleParam("serial_on",serial_on))
      {
          std::cout<<"serial_on is incorrect"<<std::endl;
      }
      if(!config.GetModuleParam("comport",comport_))
      {
          std::cout<<"comport is incorrect"<<std::endl;
      }
      if(!config.GetModuleParam("baudrate",baudrate_))
      {
          std::cout<<"baudrate is incorrect"<<std::endl;
      }
      if(!config.GetModuleParam("timeout_ms",timeout_ms_))
      {
	      std::cout<<"timeout_ms  is  not exist"<<std::endl;
      }
      if(!config.GetSystemParam("GetINSData_on",Module_On))
      {
              std::cout<<"GetINSData_on  is  not exist"<<std::endl;
      }
      else
      {
	      if(Module_On)
	      std::cout<<"GetINSData_on  On"<<std::endl;
	      else
	      std::cout<<"GetINSData_on Off"<<std::endl;
      }


      /*	double timeout;
      if(!config.GetModuleParam("timeout",timeout))
      {
	      std::cout<<"port num is incorrect"std::endl;
      }*/
      /*	std::cout<<timeout<<" "<<timer->timeout<<std::endl;
      timer->timeout=timeout;
      std::cout<<timeout<<" "<<timer->timeout<<std::endl;*/
  }
  void InitRecordFileTitle()
  {
  	playback.BeginSaveTitle();
  	playback<<"time_stampe";
  	#ifdef TANGPLATFORM_76GF6//如果是无人唐
  	playback<<"dHeading";
  	#endif
  	#ifdef TANGPLATFORM_YOUREN//如果是有人唐
  	playback<<"dLatitude"<<"dLongitude"<<"dHeading";
  	#endif
  	#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
  	playback<<"dLatitude"<<"dLongitude"<<"dHeading";
  	#endif
	#if(defined TOYOTAPLATFORM)|| (defined TANGPLATFORM_KI0E5)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
	playback<<"dLatitude"<<"dLongitude"
			<<"dHeading"<<"dPitch"<<"dRoll"
			<<"dAccx"<<"dAccy"<<"dAccz"
			<<"dArx"<<"dAry"<<"dArz"
			<<"dVelE"<<"dVelN"<<"dVelSky";
	#endif
  	playback.EndSaveTitle();
  }

  const struct_INS_Mess& getInsData()
  {
    return ins_data_;
  }

private:
  bool Module_On;
  int autoMapCounter;
  int autoMapIndex;
  XmlConf config;
  IvDataPlayback playback;
  int port;
  bool serial_on;
  std::string configstr_;
  bool CreatListenToINS;
  struct_INS_Mess ins_data_;

  std::string comport_;
  int baudrate_;
  int timeout_ms_;
#ifdef TANGPLATFORM_76GF6//如果是无人唐
  CAnalysisINS m_AnalysisINS;
#endif
#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
  Imu_531 m_INSFons;
#endif
#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
  CAnalysisINS_OxT m_AnalysisINS_OxT;
#endif

  void my_record()
  {
  	playback.BeginSaveLine();
  	#ifdef TANGPLATFORM_76GF6//如果是无人唐
  	playback<<ins_data_.lTimeStamp;
  	playback<<ins_data_.dHeading;
  	#endif

  	#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
  	playback<<ins_data_.lTimeStamp;
  	playback<<ins_data_.dLatitude*100.0;
  	playback<<ins_data_.dLongitude*100.0;
  	playback<<ins_data_.dHeading;
  	playback<<ins_data_.dPitch;
  	playback<<ins_data_.dRoll;
  	playback<<ins_data_.dAccx;
  	playback<<ins_data_.dAccy;
  	playback<<ins_data_.dAccz;

  	playback<<ins_data_.dArx;
  	playback<<ins_data_.dAry;
  	playback<<ins_data_.dArz;
  	playback<<ins_data_.dVelE;
  	playback<<ins_data_.dVelN;
  	playback<<ins_data_.dVelSky;

  	#endif

  	#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
  	playback<<ins_data_.lTimeStamp;
  	playback<<ins_data_.dLatitude*100.0;
  	playback<<ins_data_.dLongitude*100.0;
  	playback<<ins_data_.dHeading;
  	#endif


  	playback.EndSaveLine();
  }

void my_playback()
{
  if(playback.BeginLoadLine()==true)
  {
	  #ifdef TANGPLATFORM_76GF6//如果是无人唐
	  playback>>ins_data_.lTimeStamp;
	  playback>>ins_data_.dHeading;
	  #endif

	  #if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
	  playback>>ins_data_.lTimeStamp;
	  playback>>ins_data_.dLatitude;
	  ins_data_.dLatitude *= 0.01;
	  playback>>ins_data_.dLongitude;
	  ins_data_.dLongitude *= 0.01;
	  playback>>ins_data_.dHeading;
	  playback>>ins_data_.dPitch;
	  playback>>ins_data_.dRoll;
	  playback>>ins_data_.dAccx;
	  playback>>ins_data_.dAccy;
	  playback>>ins_data_.dAccz;

	  playback>>ins_data_.dArx;
	  playback>>ins_data_.dAry;
	  playback>>ins_data_.dArz;

	  playback>>ins_data_.dVelE;
	  playback>>ins_data_.dVelN;
	  playback>>ins_data_.dVelSky;

	  LOG(INFO)<<ins_data_.dLatitude<<" "<<ins_data_.dLongitude;
	  #endif

	  #if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
	  playback>>ins_data_.lTimeStamp;
	  playback>>ins_data_.dLatitude;
	  playback>>ins_data_.dLongitude;
	  ins_data_.dLatitude=ins_data_.dLatitude*0.01;
	  ins_data_.dLongitude=ins_data_.dLongitude*0.01;

	  playback>>ins_data_.dHeading;



	  #endif
	  playback.EndLoadLine();
  }
}
};



int main(int argc,char ** argv)
{
  ros::init(argc,argv,"getgpsdata");
  ros::NodeHandle nh;
google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
//  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

  ros::Publisher GPS_Pub = nh.advertise<sensor_driver_msgs::GpswithHeading> ("GPSmsg", 50);
  ros::Publisher VehicleState_pub = nh.advertise<sensor_driver_msgs::VehicleState>("vehiclestate_GPS", 50);
//IMU原来的
 ros::Publisher pubImudata;
  pubImudata = nh.advertise<sensor_msgs::Imu> ("imudata", 50);
  ros::Publisher pubPosition;
  pubPosition = nh.advertise<geometry_msgs::Vector3Stamped> ("imuposition", 50);
  ros::Publisher gps_Pub = nh.advertise<sensor_driver_msgs::GpswithHeading> ("gpsdata", 50);
  ros::Publisher pubInsVelocity = nh.advertise<sensor_driver_msgs::InsVelocity> ("insvelocity", 50);
  ros::Publisher pubGlobalVelocity = nh.advertise<sensor_driver_msgs::InsVelocity> ("globalvelocity", 50);

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

std::string filename;
  InsModule insmodule(startconfig);

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

	bool succeed = insmodule.process();

      //if(succeed == false)
	//{
	//LOG(ERROR)<<"insmodule process error";
	//break;
	//}

    if (!success || !succeed){
      ROS_ERROR_STREAM("gpsmodule process error");
      usleep(100000);
      state = ros::ok();
      continue;
    }
    tem_gps_data = m_run_model.GetGPSData();
 const struct_INS_Mess& ins_data = insmodule.getInsData();

////pub imudata
	  float roll = ins_data.dRoll*M_PI/180;
      float pitch = ins_data.dPitch*M_PI/180;
      float heading = ins_data.dHeading*M_PI/180;
      tf::Quaternion orientation=tf::createQuaternionFromRPY(roll, pitch, heading);

      sensor_msgs::Imu imuout;
      tf::quaternionTFToMsg(orientation,imuout.orientation);
      imuout.header.stamp = ros::Time::now();
      imuout.header.frame_id="imu_frame";
      imuout.linear_acceleration.x = ins_data.dAccx;
      imuout.linear_acceleration.y = ins_data.dAccy;
      imuout.linear_acceleration.z = ins_data.dAccz;

      imuout.angular_velocity.x = ins_data.dArx*M_PI/180;
      imuout.angular_velocity.y = ins_data.dAry*M_PI/180;
      imuout.angular_velocity.z = ins_data.dArz*M_PI/180;
	  pubImudata.publish(imuout);

// pub imuposition
      double N,E;
      geographic_to_grid(a, e2, ins_data.dLatitude*M_PI/180, ins_data.dLongitude*M_PI/180, &zone, &hemi, &N, &E);
      static geometry_msgs::Vector3Stamped pos0;
      static int framenum=0;
      if(framenum==0)
	{
	  pos0.vector.x = E;
	  pos0.vector.z = - N;

	}
      framenum++;
      geometry_msgs::Vector3Stamped pos;
      pos.header.stamp = imuout.header.stamp;

      pos.vector.x = E - pos0.vector.x;
      pos.vector.z = - N - pos0.vector.z;
      pubPosition.publish(pos);

//pub insvelocity
		sensor_driver_msgs::InsVelocity insvelocity;
		insvelocity.header.frame_id = "ins_frame";
		insvelocity.header.stamp = imuout.header.stamp;
		insvelocity.angular_velocity = imuout.angular_velocity;
		Eigen::Vector3d globalvelocity;
		globalvelocity.x() = ins_data.dVelE;
		globalvelocity.y() = ins_data.dVelN;
		globalvelocity.z() = ins_data.dVelSky;

		auto inspose = ::ivcommon::transform::RollPitchYaw(roll, pitch, heading);
		Eigen::Vector3d velocity = inspose.inverse()*globalvelocity;
		insvelocity.linear_velocity.x = velocity.x();
		insvelocity.linear_velocity.y = velocity.y();
		insvelocity.linear_velocity.z = velocity.z();
		if(velocity.norm()<100)
			pubInsVelocity.publish(insvelocity);
///pub globalvelocity
		sensor_driver_msgs::InsVelocity globalvel;
		globalvel.header.frame_id = "gps_frame";
		globalvel.header.stamp = imuout.header.stamp;
		globalvel.linear_velocity.x = ins_data.dVelE;
		globalvel.linear_velocity.y = ins_data.dVelN;
		globalvel.linear_velocity.z = ins_data.dVelSky;
		if(globalvelocity.norm()<100)
			pubGlobalVelocity.publish(globalvel);

///pub GPSmsg
  sensor_driver_msgs::GpswithHeading GPS_msg;
  GPS_msg.header.frame_id = "gps_frame";
  GPS_msg.header.stamp = ros::Time::now();
  GPS_msg.gps.header = GPS_msg.header;

  GPS_msg.gps.latitude = tem_gps_data->dLatitude;
  GPS_msg.gps.longitude = tem_gps_data->dLongitude;
  GPS_msg.gps.altitude = tem_gps_data->Altitude;
   double heading_angle= tem_gps_data->PHDT_heading;
	if(heading_angle<=0)
	  heading_angle = -heading_angle;
	else
	  heading_angle = 360 -heading_angle;
	if(heading_angle>180)
		heading_angle -= 360;

  GPS_msg.heading = heading_angle;
  GPS_msg.roll = tem_gps_data->HDOP;
  GPS_msg.pitch = tem_gps_data->DGPSState;
  GPS_Pub.publish(GPS_msg);

//pub gpsdata
  sensor_driver_msgs::GpswithHeading gps;
  gps.header.frame_id = "gps_frame";
  gps.header.stamp = ros::Time::now();
  gps.gps.header = gps.header;

  gps.gps.latitude = tem_gps_data->dLatitude;
  gps.gps.longitude = tem_gps_data->dLongitude;
  gps.gps.altitude = tem_gps_data->Altitude;
  gps.heading = ins_data.dHeading;
  gps.pitch = ins_data.dPitch;
  gps.roll = ins_data.dRoll;
  gps.mode = ins_data.dState;
      
   if(gps.heading<-180)
		  gps.heading +=360;
      if(gps.heading>180)
		  gps.heading -=360;

  gps_Pub.publish(gps);

//pub vehiclestate_GPS
  sensor_driver_msgs::VehicleState vehicleState;
  vehicleState.header = gps.header;
  vehicleState.gps_week = tem_gps_data->gps_week;
  vehicleState.gps_ms = tem_gps_data->gps_ms;
  vehicleState.gps = gps.gps;
  VehicleState_pub.publish(vehicleState);

  state = ros::ok();
  }
}




#include <common/common.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/console.h>
#include <glog/logging.h>
#include <util/playback/iv_data_playback.h>
#include <util/ToXml.hh>
#include <util/utm/datum.h>
#include <util/utm/utm.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "ecu/AnalysisECU.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/ECUData.h"

class ecuModule{
public:
  ecuModule(std::string configstr):configstr_(configstr)
  {
    init();
  }
  bool init()
  {
    Module_On=false;
    autoMapCounter=0;
    autoMapIndex=0;
    if(!Module_On)//qjy,20170630。只进行一次INIT
    {
	if(LoadConfigFile(configstr_.c_str())==true)
	{
	#if((defined BYDTANG)||(defined BYDRAY)||(defined TOYOTAPLATFORM))//如果是比亚迪；如果是丰田——霍钊2018010
		std::cout<<"Get  ECU port sucess  ,the number   is "<<port<<std::endl;
		ListenSucess_=m_AnalysisECU_.Init(port);
		if(!ListenSucess_)
			std::cout<<"Creat ListenTo ECU   failure "<<std::endl;
	#endif
	}
	else
	std::cout<<"Get  ECU port failed  "<<std::endl;
    }
    return Module_On;
  }

  bool process()
  {
		if(Module_On)
		{
			if(play_back_.PlaybackIsOn())
			{
				my_playback();
				//		cout<<1234;
			}
			else
			{
								//如果是唐6或丰田或6吨//20180102
	#if(defined TANGPLATFORM_76GF6)||(defined TOYOTAPLATFORM)||(defined CATERPILLARCAR6)||(defined HUACHEN)||(defined HUALONG)//20180306 add qjy udp

				m_AnalysisECU_.Update();//
				ecu_data_.lTimeStamp=play_back_.SysTime();
				ecu_data_.EnginRate=m_AnalysisECU_.ECUData_struct.EnginRate;
				ecu_data_.Estop_enabled=m_AnalysisECU_.ECUData_struct.Estop_enabled;
				ecu_data_.PC_Tx_err=m_AnalysisECU_.ECUData_struct.PC_Tx_err;
				ecu_data_.autodrive_status=m_AnalysisECU_.ECUData_struct.autodrive_status;
				ecu_data_.brakeRx_err=m_AnalysisECU_.ECUData_struct.brakeRx_err;
				ecu_data_.brakeTx_err=m_AnalysisECU_.ECUData_struct.brakeTx_err;
				ecu_data_.brake_enabled=m_AnalysisECU_.ECUData_struct.brake_enabled;
				ecu_data_.brake_pedal_signal=m_AnalysisECU_.ECUData_struct.brake_pedal_signal;
				ecu_data_.bugle_status=m_AnalysisECU_.ECUData_struct.bugle_status;
				ecu_data_.fDeForwardVel=m_AnalysisECU_.ECUData_struct.fDeForwardVel;
				ecu_data_.fFLRWheelAverAngle=m_AnalysisECU_.ECUData_struct.fFLRWheelAverAngle;//linshi

				ecu_data_.fForwardVel=m_AnalysisECU_.ECUData_struct.fForwardVel;//linshi
				//	ecu_data_.fHeading=m_AnalysisECU_.ECUData_struct.fHeading;

				ecu_data_.fOdometer=m_AnalysisECU_.ECUData_struct.fOdometer;
				ecu_data_.fRadius=m_AnalysisECU_.ECUData_struct.fRadius;
				//	ecu_data_.fTheta=m_AnalysisECU_.ECUData_struct.fTheta;
				ecu_data_.f_estop=m_AnalysisECU_.ECUData_struct.f_estop;
				ecu_data_.f_leftlamp=m_AnalysisECU_.ECUData_struct.f_leftlamp;
				ecu_data_.f_rightlamp=m_AnalysisECU_.ECUData_struct.f_rightlamp;

				ecu_data_.f_shift=m_AnalysisECU_.ECUData_struct.f_shift;

				ecu_data_.f_shift1=m_AnalysisECU_.ECUData_struct.f_shift1;

				//	ecu_data_.lTimeStamp=m_AnalysisECU_.ECUData_struct.lTimeStamp;

				ecu_data_.lateralctrl_enabled=m_AnalysisECU_.ECUData_struct.lateralctrl_enabled;
				ecu_data_.light_far=m_AnalysisECU_.ECUData_struct.light_far;
				ecu_data_.light_near=m_AnalysisECU_.ECUData_struct.light_near;
				ecu_data_.longitutdectrl_enabled=m_AnalysisECU_.ECUData_struct.longitutdectrl_enabled;

				ecu_data_.petral_pressure=m_AnalysisECU_.ECUData_struct.petral_pressure;
				ecu_data_.PC_Tx_err=m_AnalysisECU_.ECUData_struct.PC_Tx_err;
				ecu_data_.poweron_status=m_AnalysisECU_.ECUData_struct.poweron_status;
				ecu_data_.pressure_back=m_AnalysisECU_.ECUData_struct.pressure_back;
				ecu_data_.start_status=m_AnalysisECU_.ECUData_struct.start_status;
				ecu_data_.steerRx_err=m_AnalysisECU_.ECUData_struct.steerRx_err;

				ecu_data_.switch_signal=m_AnalysisECU_.ECUData_struct.switch_signal;
				ecu_data_.steerTx_err=m_AnalysisECU_.ECUData_struct.steerTx_err;
				ecu_data_.throtle_feedback=m_AnalysisECU_.ECUData_struct.throtle_feedback;
				ecu_data_.warning_status=m_AnalysisECU_.ECUData_struct.warning_status;
				ecu_data_.FrontLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontLeftWheelSpeed;
				ecu_data_.FrontLeftWheelSpeedStatus=m_AnalysisECU_.ECUData_struct.FrontLeftWheelSpeedStatus;
				ecu_data_.FrontRightWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontRightWheelSpeed;
				ecu_data_.FrontRightWheelSpeedStatus=m_AnalysisECU_.ECUData_struct.FrontRightWheelSpeedStatus;
				ecu_data_.BackLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.BackLeftWheelSpeed;
				ecu_data_.BackLeftWheelSpeedStatus=m_AnalysisECU_.ECUData_struct.BackLeftWheelSpeedStatus;
				ecu_data_.BackRightWheelSpeed=m_AnalysisECU_.ECUData_struct.BackRightWheelSpeed;
				ecu_data_.BackRightWheelSpeedStatus=m_AnalysisECU_.ECUData_struct.BackRightWheelSpeedStatus;

				ecu_data_.Yaw_Rate=m_AnalysisECU_.ECUData_struct.Yaw_Rate;
				ecu_data_.Yaw_Rate_Offset=m_AnalysisECU_.ECUData_struct.Yaw_Rate_Offset;
				ecu_data_.Yaw_Rate_Status=m_AnalysisECU_.ECUData_struct.Yaw_Rate_Status;

				ecu_data_.AX=m_AnalysisECU_.ECUData_struct.AX;
				ecu_data_.AX=m_AnalysisECU_.ECUData_struct.AX_Offset;
				ecu_data_.AX_Status=m_AnalysisECU_.ECUData_struct.AX_Status;

				ecu_data_.AX=m_AnalysisECU_.ECUData_struct.AY;
				ecu_data_.AX=m_AnalysisECU_.ECUData_struct.AY_Offset;
				ecu_data_.AX_Status=m_AnalysisECU_.ECUData_struct.AY_Status;

	            /***************************20180130huozhao**********************/
	            ecu_data_.T_lFBrakePressure = m_AnalysisECU_.ECUData_struct.T_lFBrakePressure;
	            ecu_data_.T_lRBrakePressure = m_AnalysisECU_.ECUData_struct.T_lRBrakePressure;
	            ecu_data_.T_rFBrakePressure = m_AnalysisECU_.ECUData_struct.T_rFBrakePressure;
	            ecu_data_.T_rRBrakePressure = m_AnalysisECU_.ECUData_struct.T_rRBrakePressure;
	            ecu_data_.Engine_load = m_AnalysisECU_.ECUData_struct.Engine_load;
	            /**************************20180130huozhao*********************/

				#endif

				#ifdef BYDRAY					//如果是速锐

				m_AnalysisECU_.Update();//
				ecu_data_.lTimeStamp=play_back_.SysTime();
				ecu_data_.EnginRate=m_AnalysisECU_.ECUData_struct.EnginRate;
				ecu_data_.Estop_enabled=m_AnalysisECU_.ECUData_struct.Estop_enabled;
				ecu_data_.PC_Tx_err=m_AnalysisECU_.ECUData_struct.PC_Tx_err;
				ecu_data_.autodrive_status=m_AnalysisECU_.ECUData_struct.autodrive_status;
				ecu_data_.brakeRx_err=m_AnalysisECU_.ECUData_struct.brakeRx_err;
				ecu_data_.brakeTx_err=m_AnalysisECU_.ECUData_struct.brakeTx_err;
				ecu_data_.brake_enabled=m_AnalysisECU_.ECUData_struct.brake_enabled;
				ecu_data_.brake_pedal_signal=m_AnalysisECU_.ECUData_struct.brake_pedal_signal;
				ecu_data_.bugle_status=m_AnalysisECU_.ECUData_struct.bugle_status;
				ecu_data_.fDeForwardVel=m_AnalysisECU_.ECUData_struct.fDeForwardVel;
				ecu_data_.fFLRWheelAverAngle=m_AnalysisECU_.ECUData_struct.fFLRWheelAverAngle;//linshi

				ecu_data_.fForwardVel=m_AnalysisECU_.ECUData_struct.fForwardVel;//linshi
				//	ecu_data_.fHeading=m_AnalysisECU_.ECUData_struct.fHeading;

				ecu_data_.fOdometer=m_AnalysisECU_.ECUData_struct.fOdometer;
				ecu_data_.fRadius=m_AnalysisECU_.ECUData_struct.fRadius;
				//	ecu_data_.fTheta=m_AnalysisECU_.ECUData_struct.fTheta;
				ecu_data_.f_estop=m_AnalysisECU_.ECUData_struct.f_estop;
				ecu_data_.f_leftlamp=m_AnalysisECU_.ECUData_struct.f_leftlamp;
				ecu_data_.f_rightlamp=m_AnalysisECU_.ECUData_struct.f_rightlamp;

				ecu_data_.f_shift=m_AnalysisECU_.ECUData_struct.f_shift;

				ecu_data_.f_shift1=m_AnalysisECU_.ECUData_struct.f_shift1;

				//	ecu_data_.lTimeStamp=m_AnalysisECU_.ECUData_struct.lTimeStamp;

				ecu_data_.lateralctrl_enabled=m_AnalysisECU_.ECUData_struct.lateralctrl_enabled;
				ecu_data_.light_far=m_AnalysisECU_.ECUData_struct.light_far;
				ecu_data_.light_near=m_AnalysisECU_.ECUData_struct.light_near;
				ecu_data_.longitutdectrl_enabled=m_AnalysisECU_.ECUData_struct.longitutdectrl_enabled;

				ecu_data_.petral_pressure=m_AnalysisECU_.ECUData_struct.petral_pressure;
				ecu_data_.PC_Tx_err=m_AnalysisECU_.ECUData_struct.PC_Tx_err;
				ecu_data_.poweron_status=m_AnalysisECU_.ECUData_struct.poweron_status;
				ecu_data_.pressure_back=m_AnalysisECU_.ECUData_struct.pressure_back;
				ecu_data_.start_status=m_AnalysisECU_.ECUData_struct.start_status;
				ecu_data_.steerRx_err=m_AnalysisECU_.ECUData_struct.steerRx_err;

				ecu_data_.switch_signal=m_AnalysisECU_.ECUData_struct.switch_signal;
				ecu_data_.steerTx_err=m_AnalysisECU_.ECUData_struct.steerTx_err;
				ecu_data_.throtle_feedback=m_AnalysisECU_.ECUData_struct.throtle_feedback;
				ecu_data_.warning_status=m_AnalysisECU_.ECUData_struct.warning_status;

				ecu_data_.FrontLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontLeftWheelSpeed;
				ecu_data_.FrontRightWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontRightWheelSpeed;
				ecu_data_.BackLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.BackLeftWheelSpeed;
				ecu_data_.BackRightWheelSpeed=m_AnalysisECU_.ECUData_struct.BackRightWheelSpeed;


				#endif

		//		#ifdef FOTONBUS					//如果是福田的冬奥会大巴,直接从Can接收模块读取ECU数据
			#if(defined FOTONBUS)||(defined MIDBUS_HAILIANG)||(defined TANGPLATFORM_KI0E5)//如果是福田大巴或海梁中巴或唐5
				MS_CANDATAOUT_CHANNEL->read();

				ecu_data_.lTimeStamp=play_back_.SysTime();
				ecu_data_.Estop_enabled=ms_CanDataOut_data->ECUData_struct.Estop_enabled;
				ecu_data_.PC_Tx_err=ms_CanDataOut_data->ECUData_struct.PC_Tx_err;

				ecu_data_.brakeRx_err=ms_CanDataOut_data->ECUData_struct.brakeRx_err;
				ecu_data_.brakeTx_err=ms_CanDataOut_data->ECUData_struct.brakeTx_err;

				ecu_data_.brake_pedal_signal=ms_CanDataOut_data->ECUData_struct.brake_pedal_signal;
				ecu_data_.bugle_status=ms_CanDataOut_data->ECUData_struct.bugle_status;
				ecu_data_.fDeForwardVel=ms_CanDataOut_data->ECUData_struct.fDeForwardVel;

				ecu_data_.fOdometer=ms_CanDataOut_data->ECUData_struct.fOdometer;
				ecu_data_.fRadius=ms_CanDataOut_data->ECUData_struct.fRadius;
				ecu_data_.f_estop=ms_CanDataOut_data->ECUData_struct.f_estop;

				ecu_data_.PC_Tx_err=ms_CanDataOut_data->ECUData_struct.PC_Tx_err;
				ecu_data_.poweron_status=ms_CanDataOut_data->ECUData_struct.poweron_status;

				ecu_data_.start_status=ms_CanDataOut_data->ECUData_struct.start_status;
				ecu_data_.steerRx_err=ms_CanDataOut_data->ECUData_struct.steerRx_err;

				ecu_data_.switch_signal=ms_CanDataOut_data->ECUData_struct.switch_signal;
				ecu_data_.steerTx_err=ms_CanDataOut_data->ECUData_struct.steerTx_err;

				ecu_data_.warning_status=ms_CanDataOut_data->ECUData_struct.warning_status;

				ecu_data_.autodrive_status=ms_CanDataOut_data->ECUData_struct.autodrive_status;//自动驾驶状态
				ecu_data_.brake_enabled=ms_CanDataOut_data->ECUData_struct.brake_enabled;//是否处于刹车状态
				ecu_data_.fForwardVel=ms_CanDataOut_data->ECUData_struct.fForwardVel;//车辆速度



				ecu_data_.fFLRWheelAverAngle = (double)(-ms_CanDataOut_data->ECUData_struct.Streeing_Angle) ;
				if(ecu_data_.fFLRWheelAverAngle > 0 )
					ecu_data_.fFLRWheelAverAngle = 10*ecu_data_.fFLRWheelAverAngle / steeringratio_l_;
				else
					ecu_data_.fFLRWheelAverAngle =10* ecu_data_.fFLRWheelAverAngle / steeringratio_r_;



				ecu_data_.longitutdectrl_enabled=ms_CanDataOut_data->ECUData_struct.longitutdectrl_enabled;//车辆当前是否处于纵向使能状态？
				ecu_data_.lateralctrl_enabled=ms_CanDataOut_data->ECUData_struct.lateralctrl_enabled;//车辆横向使能反馈
				ecu_data_.f_shift=ms_CanDataOut_data->ECUData_struct.f_shift;//当前档位
				ecu_data_.f_shift1=ms_CanDataOut_data->ECUData_struct.f_shift1;//当前小档位
				ecu_data_.light_far=ms_CanDataOut_data->ECUData_struct.light_far;//远光灯状态
				ecu_data_.light_near=ms_CanDataOut_data->ECUData_struct.light_near;//近光灯状态
				ecu_data_.f_leftlamp=ms_CanDataOut_data->ECUData_struct.f_leftlamp;//左转向灯状态
				ecu_data_.f_rightlamp=ms_CanDataOut_data->ECUData_struct.f_rightlamp;//右转向灯状态
				ecu_data_.petral_pressure=ms_CanDataOut_data->ECUData_struct.petral_pressure;//前制动压力
				ecu_data_.pressure_back=ms_CanDataOut_data->ECUData_struct.pressure_back;//后制动压力
				ecu_data_.throtle_feedback=ms_CanDataOut_data->ECUData_struct.throtle_feedback;//油门反馈量
				ecu_data_.speaker=ms_CanDataOut_data->ECUData_struct.speaker;//福田项目中喇叭反馈 Active1 off 0
				ecu_data_.Brakelight=ms_CanDataOut_data->ECUData_struct.Brakelight;//福田项目中制动灯反馈 Active1 off 0
				ecu_data_.Positionlight=ms_CanDataOut_data->ECUData_struct.Positionlight;//福田项目中位置灯反馈 Active1 off 0



				ecu_data_.StrAngleValid=ms_CanDataOut_data->ECUData_struct.StrAngleValid;//福田项目中转角是否有效 Valid 0 Not Valid1
				ecu_data_.fFLRWheelAverAngleSpd=ms_CanDataOut_data->ECUData_struct.fFLRWheelAverAngleSpd;//福田项目中方向盘角速度速度
				ecu_data_.StrAngleSpdValid=ms_CanDataOut_data->ECUData_struct.StrAngleSpdValid;///福田项目中转角速度是否有效 Valid 0 Not Valid1


				ecu_data_.transmissioutputshafttorque=ms_CanDataOut_data->ECUData_struct.transmissioutputshafttorque;//电机输出扭矩反馈
				ecu_data_.GasPedalPosition=ms_CanDataOut_data->ECUData_struct.GasPedalPosition;


				ecu_data_.FrontLeftWheelSpeed=ms_CanDataOut_data->ECUData_struct.Flwheelspeed;//新增轮速信息，	左前轮速,qjy,20171001
				ecu_data_.FrontRightWheelSpeed=ms_CanDataOut_data->ECUData_struct.Frwheelspeed;//新增轮速信息，
				ecu_data_.BackLeftWheelSpeed=ms_CanDataOut_data->ECUData_struct.Blwheelspeed;//新增轮速信息，
				ecu_data_.BackRightWheelSpeed=ms_CanDataOut_data->ECUData_struct.Brwheelspeed;//新增轮速信息，

				#endif
    	if(play_back_.RecordIsOn())
			my_record();
			}
			return true;
		}
		return false;
  }

    bool LoadConfigFile(const char* configstr)
    {
    	//memset(m_sys_start_data->xmlconf,0,10000);
    	if(!config.Parse(configstr, "GetECUData"))
    	{
    		std::cout<<"GetECUData  is not exist in config xml  file"<<std::endl;
    		return false;
    	}


    	play_back_.Setup(config);
    	ConfigParam();

    	if(play_back_.RecordIsOn())
    	{
    		InitRecordFileTitle();
    	}
    	return true;

  }

  void ConfigParam()
  {

		if(!config.GetModuleParam("port",port))
		{
			std::cout<<"port num is incorrect"<<std::endl;
		}
		if(!config.GetSystemParam("GetECUData_on",Module_On))
		{
			std::cout<<"GetECUData_on  is  not exist"<<std::endl;
		}
		else
		{
			if(Module_On)
			std::cout<<"GetECUData_on  On"<<std::endl;
			else
			std::cout<<"GetECUData_on Off"<<std::endl;
		}


		if(!config.Parse(configstr_.c_str(), "VehicleParam"))//获得参数0829，qjy
		{
			std::cout<<"VehicleParam  is not exist in config xml  file"<<std::endl;
		}
		if(!config.GetModuleParam("steeringratio_l",steeringratio_l_))
		{
			std::cout<<"steeringratio_l num is incorrect"<<std::endl;
		}
		else
		std::cout<<"steeringratio_l is "<<steeringratio_l_<<std::endl;

		if(!config.GetModuleParam("steeringratio_r",steeringratio_r_))
		{
			std::cout<<"steeringratio_r num is incorrect"<<std::endl;
		}
		else
		std::cout<<"steeringratio_r is "<<steeringratio_r_<<std::endl;

			#ifdef BYDTANG					//如果是唐
				m_AnalysisECU.steeringratio_l_=steeringratio_l_;
				m_AnalysisECU.steeringratio_r_=steeringratio_r_;
			#endif

			#ifdef BYDRAY					//如果是速锐
				m_AnalysisECU.steeringratio_l_=steeringratio_l_;
				m_AnalysisECU.steeringratio_r_=steeringratio_r_;
			#endif

  }
  void InitRecordFileTitle()
  {
		play_back_.BeginSaveTitle();
		play_back_<<"time_stampe"<<"fDeForwardVel"<<"fFLRWheelAverAngle"<<"fForwardVel"<<"f_shift"<<"petral_pressure"<<"pressure_back";

		play_back_.EndSaveTitle();
  }

  const struct_ECU& getEcuData()
  {
    return ecu_data_;
  }

private:
  void my_record()
    {
    	play_back_.BeginSaveLine();
    /*	#if(defined BYDTANG)||(defined BYDRAY)//如果是比亚迪
    	play_back_<<ecu_data_.lTimeStamp;
    	play_back_<<ecu_data_.fDeForwardVel;
    	play_back_<<ecu_data_.fFLRWheelAverAngle;//1
    	play_back_<<ecu_data_.fForwardVel;
    	play_back_<<(int)(ecu_data_.f_shift);
    	play_back_<<ecu_data_.petral_pressure;//2
    	play_back_<<ecu_data_.pressure_back;

    	play_back_<<ecu_data_.FrontLeftWheelSpeed;
    	play_back_<<ecu_data_.FrontRightWheelSpeed;
    	play_back_<<ecu_data_.BackLeftWheelSpeed;//1
    	play_back_<<ecu_data_.BackRightWheelSpeed;

    	#endif

    #if(defined FOTONBUS)||(defined MIDBUS_HAILIANG)//如果是福田大巴或海梁中巴
    */
    	play_back_<<ecu_data_.lTimeStamp;
    	play_back_<<ecu_data_.fDeForwardVel;
    	play_back_<<ecu_data_.fFLRWheelAverAngle;
    	play_back_<<ecu_data_.fForwardVel;
    	play_back_<<(int)ecu_data_.f_shift;
    	play_back_<<ecu_data_.petral_pressure;
    	play_back_<<ecu_data_.pressure_back;
    	play_back_<<ecu_data_.FrontLeftWheelSpeed;
    	play_back_<<ecu_data_.FrontRightWheelSpeed;
    	play_back_<<ecu_data_.BackLeftWheelSpeed;//1
    	play_back_<<ecu_data_.BackRightWheelSpeed;

    	//#endif



    	play_back_.EndSaveLine();
    }

  void my_playback()
  {
  	int shift;//qjy,20170812，解决档位回放错误的问题
  	if(play_back_.BeginLoadLine()==true)
  	{

  		play_back_>>ecu_data_.lTimeStamp;
  		play_back_>>ecu_data_.fDeForwardVel;
  		play_back_>>ecu_data_.fFLRWheelAverAngle;
  		play_back_>>ecu_data_.fForwardVel;
  		play_back_>>shift;
  		ecu_data_.f_shift=shift;
  		play_back_>>ecu_data_.petral_pressure;
  		play_back_>>ecu_data_.pressure_back;
  		play_back_>>ecu_data_.FrontLeftWheelSpeed;
  		play_back_>>ecu_data_.FrontRightWheelSpeed;
  		play_back_>>ecu_data_.BackLeftWheelSpeed;//1
  		play_back_>>ecu_data_.BackRightWheelSpeed;

  		play_back_.EndLoadLine();
  	}
  }

  bool Module_On;
  int autoMapCounter;
  int autoMapIndex;
  XmlConf config;
  IvDataPlayback play_back_;
  int port;
  std::string configstr_;

  CAnalysisECU m_AnalysisECU_;
  struct_ECU ecu_data_;
  double steeringratio_l_;
  double steeringratio_r_;
  double petral_pressure;
  bool ListenSucess_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "getecudata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
//  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

  ros::Publisher pubECUdata;
  pubECUdata = nh.advertise<sensor_driver_msgs::ECUData> ("ecudata", 50);



  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
   {
     ros::Duration(0.01).sleep();
   }

  std::string startconfig = configsrv.response.configstr;


  std::string filename;
  ecuModule ecumodule(startconfig);
  ros::Rate rate(50);
  bool status = ros::ok();
  while(status)
  {

    bool succeed = ecumodule.process();
    if(succeed == false)
	{
	LOG(ERROR)<<"ecu module process error";
	break;
	}
    const struct_ECU& ecu_data = ecumodule.getEcuData();
    sensor_driver_msgs::ECUData ecumsg;
    ecumsg.header.stamp=ros::Time::now();
    ecumsg.fDeForwardVel=ecu_data.fDeForwardVel;
    ecumsg.fFLRWheelAverAngle=ecu_data.fFLRWheelAverAngle;
    ecumsg.fForwardVel=ecu_data.fForwardVel;

    ecumsg.petral_pressure=ecu_data.petral_pressure;
    ecumsg.pressure_back=ecu_data.pressure_back;
    ecumsg.FrontLeftWheelSpeed=ecu_data.FrontLeftWheelSpeed;
    ecumsg.FrontRightWheelSpeed=ecu_data.FrontRightWheelSpeed;
    ecumsg.BackLeftWheelSpeed=ecu_data.BackLeftWheelSpeed;//1
    ecumsg.BackRightWheelSpeed=ecu_data.BackRightWheelSpeed;

	LOG(INFO)<<"f_shift:"<<(int)ecu_data.f_shift1;
    LOG(INFO)<<"fDeForwardVel:"<<ecu_data.fDeForwardVel;
    LOG(INFO)<<"fFLRWheelAverAngle:"<<ecu_data.fFLRWheelAverAngle;
    LOG(INFO)<<"fForwardVel:"<<ecu_data.fForwardVel;
    LOG(INFO)<<"petral_pressure:"<<ecu_data.petral_pressure;
    LOG(INFO)<<"pressure_back:"<<ecu_data.pressure_back;
    LOG(INFO)<<"FrontLeftWheelSpeed:"<<ecu_data.FrontLeftWheelSpeed;
    LOG(INFO)<<"FrontRightWheelSpeed:"<<ecu_data.FrontRightWheelSpeed;
    LOG(INFO)<<"BackLeftWheelSpeed:"<<ecu_data.BackLeftWheelSpeed;//1
    LOG(INFO)<<"BackRightWheelSpeed:"<<ecu_data.BackRightWheelSpeed;

    pubECUdata.publish(ecumsg);
    rate.sleep();
    status = ros::ok();
  }
}

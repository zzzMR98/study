//======================================================================
// Author   : zhm
// Email    : zhanghm_1995@qq.com
// Version  :
// Copyright    :
// Descriptoin  : get vehicle and radar data, and publish them in a ROS message
//======================================================================
#include <iostream>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include "sensor_driver_msgs/ECUData.h"//ECUData
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
//Project headers

#include "frontal_delphi_radar.h"

#include "del_object_detection_radar.h"

#include "frontal_mmw_radar_msgs/RadarPoint.h"
#include "frontal_mmw_radar_msgs/RadarData.h"
// #include "frontal_mmw_radar_msgs/startconfig.h"

using namespace std;

//subscribe ROS messages and implement the detph image conversion
class PostProcess
{
public:
  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
      //      image_transport_nh_(nodehandle), //for transport image
      processthread_(NULL),
      processthreadfinished_ (false),
      pub_radar_data_thread_(NULL)
  {
//#ifdef DELPHI
    memset(&vehicle_info_received_,0,sizeof(vehicle_info_received_));
//#endif
    init();
  }
  ~PostProcess()
  {
    processthreadfinished_ = true;
    processthread_->join();
    pub_radar_data_thread_->join();
  }

  void init()
  {
    subECUData_ = nodehandle_.subscribe<sensor_driver_msgs::ECUData>("ecudata", 1, boost::bind(&PostProcess::ECUDataHandler,this,_1));//
    subImuData_ = nodehandle_.subscribe<sensor_msgs::Imu>("imudata", 1, boost::bind(&PostProcess::ImuDataHandler,this,_1));//
    pubRadarData_ = nodehandle_.advertise<frontal_mmw_radar_msgs::RadarData>("radardata",1);
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
    pub_radar_data_thread_ = new boost::thread(boost::bind(&PostProcess::PublishData,this));
  }




  void ECUDataHandler(const sensor_driver_msgs::ECUDataConstPtr& ecu_data_msg) //ECU数据
  {
    ROS_INFO("<get_radar_data> ECUData callback...");//indicate receive ECU data
//#ifdef DELPHI
    vehicle_info_received_.vehicle_speed = ecu_data_msg->fForwardVel;//车速, m/s
//#endif
  }

  void ImuDataHandler(const sensor_msgs::ImuConstPtr& imu_data_msg)
  {
    ROS_INFO("<get_radar_data> ImuData callback...");//indicate receive Imu data
    //惯导横摆角速度逆时针为正，毫米波雷达要求顺时针为正
    vehicle_info_received_.yaw_rate = (-imu_data_msg->angular_velocity.z)*180.0/3.1415926; //横摆角速度, degree/s
  }

	void process() {

		bool flag = frontal_delphi_receiver_.Init(); //build connection to MMW(毫米波的简称) radar
		if(flag == true) {
			cout<<"[INFO] <get_radar_data> MMW Radar UDP socket has been built!"<<endl;
		}
		else {
			cout<<"error"<<endl;
			return;
		}
		while(!processthreadfinished_)
		{
			frontal_delphi_receiver_.set_self_vehicle_info(vehicle_info_received_);
			frontal_delphi_receiver_.Update();

			usleep(100);
		}

	}
	void PublishData() {
		ros::Rate rate(25);
		while (!processthreadfinished_) {
			//data publish

			delphi_radar_target radar_data=frontal_delphi_receiver_.radar_target_data();
			frontal_mmw_radar_msgs::RadarData radar_data_msg; //final send message
			frontal_mmw_radar_msgs::RadarPoint radar_point_msg;
			for(int i=0;i<64;++i)
			{
				radar_point_msg.target_ID = radar_data.delphi_detection_array[i].target_ID;
				radar_point_msg.range = radar_data.delphi_detection_array[i].range;
				radar_point_msg.v = radar_data.delphi_detection_array[i].v;
				radar_point_msg.angle = radar_data.delphi_detection_array[i].angle;
				radar_point_msg.x = radar_data.delphi_detection_array[i].x;
				radar_point_msg.y = radar_data.delphi_detection_array[i].y;
				radar_point_msg.valid = radar_data.delphi_detection_array[i].valid;
				radar_point_msg.status = radar_data.delphi_detection_array[i].status;
				radar_point_msg.moving = radar_data.delphi_detection_array[i].moving;
				radar_point_msg.moving_fast = radar_data.delphi_detection_array[i].moving_fast;
				radar_point_msg.moving_slow = radar_data.delphi_detection_array[i].moving_slow;
				radar_data_msg.delphi_detection_array[i]=radar_point_msg;
			}
			radar_data_msg.ESR_vehicle_speed = radar_data.ESR_vehicle_speed;
			radar_data_msg.ESR_yaw_rate = radar_data.ESR_yaw_rate;
			radar_data_msg.vehicle_speed_origin = radar_data.vehicle_speed_origin;
			radar_data_msg.yaw_rate_origin = radar_data.yaw_rate_origin;
			radar_data_msg.header.stamp = ros::Time::now();
			radar_data_msg.ACC_Target_ID = radar_data.ACC_Target_ID;
			pubRadarData_.publish(radar_data_msg);
			rate.sleep();

		}
	}



private:
  ros::NodeHandle& nodehandle_;
  ros::Subscriber subImuData_ ;//sub yaw rate
  ros::Subscriber subECUData_ ;//sub vehicle speed
  ros::Publisher pubRadarData_;
  //multi thread
  boost::thread* processthread_;
  boost::thread* pub_radar_data_thread_;
  bool processthreadfinished_;
  //zhanghm add: 20180129
  FrontalDelphiRadar frontal_delphi_receiver_;
  Vehicle_Info vehicle_info_received_; //收到的车辆状态信息


};

int main(int argc, char** argv){
  ros::init(argc, argv, "get_delphi_radar_data");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);

  ros::spin();
}

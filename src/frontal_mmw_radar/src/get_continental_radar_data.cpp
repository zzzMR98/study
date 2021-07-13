//======================================================================
// Author   : mdj
// Version  :
// Copyright    :
//======================================================================
#include <iostream>
//ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
//Project headers

#include "frontal_continental_radar.h"

#include "con_object_detection_radar.h"

#include "frontal_mmw_radar_msgs/Continental_RadarPoint.h"
#include "frontal_mmw_radar_msgs/Continental_RadarData.h"
// #include "frontal_mmw_radar_msgs/startconfig.h"
using namespace std;

//subscribe ROS messages and implement the detph image conversion
class PostProcess
{
public://
  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
      //      image_transport_nh_(nodehandle), //for transport image
      processthread_(NULL),
      processthreadfinished_ (false),
      pub_radar_data_thread_(NULL)
  {
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
    pubRadarData_ = nodehandle_.advertise<frontal_mmw_radar_msgs::Continental_RadarData>("Continental_radardata",1);
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
    pub_radar_data_thread_ = new boost::thread(boost::bind(&PostProcess::PublishData,this));
  }


	void process() {

		bool flag = frontal_continental_receiver_.Init(); //build connection to MMW(毫米波的简称) radar
		if (flag == true) {
			cout << "[INFO] <get_radar_data> MMW Radar UDP socket has been built!" << endl;
		} else {
			cout << "error" << endl;
			return;
		}
		while (!processthreadfinished_) {
			frontal_continental_receiver_.Update();

			usleep(100);
		}
	}
	void PublishData() {
		ros::Rate rate(25);
		while (!processthreadfinished_) {
			//data publish

			continental_radar_target radar_data =
					frontal_continental_receiver_.radar_target_data();
			frontal_mmw_radar_msgs::Continental_RadarData radar_data_msg; //final send message
			frontal_mmw_radar_msgs::Continental_RadarPoint radar_point_msg;
			for (int i = 0 ; i < 64 ; i++)
			{
			  memset(&radar_data_msg.continental_detection_array[i], 0, sizeof(frontal_mmw_radar_msgs::Continental_RadarPoint));
			}
			for (int i = 0; i < 64; ++i) {
				radar_point_msg.target_ID =
						radar_data.continental_detection_array[i].target_ID;

				radar_point_msg.Relative_xv = radar_data.continental_detection_array[i].Relative_xv;
				radar_point_msg.Relative_yv = radar_data.continental_detection_array[i].Relative_yv;
				radar_point_msg.angle =
						radar_data.continental_detection_array[i].angle;
				radar_point_msg.x = radar_data.continental_detection_array[i].x;
				radar_point_msg.y = radar_data.continental_detection_array[i].y;
				radar_point_msg.Relative_acc_x =
						radar_data.continental_detection_array[i].Relative_acc_x;
				radar_point_msg.Relative_acc_y =
						radar_data.continental_detection_array[i].Relative_acc_y;
				radar_point_msg.Obiect_class =
						radar_data.continental_detection_array[i].Obiect_class;
				radar_point_msg.Object_Length =
						radar_data.continental_detection_array[i].Object_Length;
				radar_point_msg.Object_Width =
						radar_data.continental_detection_array[i].Object_Width;
				radar_data_msg.continental_detection_array[i] = radar_point_msg;
//				std::cout<<"ggggggggggggggggggg"<<radar_point_msg.target_ID<<"     "<<radar_point_msg.x<<std::endl;
			}
			radar_data_msg.header.stamp = ros::Time::now();
			pubRadarData_.publish(radar_data_msg);
			rate.sleep();
		}
	}



private:
  ros::NodeHandle& nodehandle_;
  ros::Publisher pubRadarData_;
  //multi thread
  boost::thread* processthread_;
  boost::thread* pub_radar_data_thread_;
  bool processthreadfinished_;
  //zhanghm add: 20180129

  FrontalContinentalRadar frontal_continental_receiver_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "get_continental_radar_data");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);

  ros::spin();
}

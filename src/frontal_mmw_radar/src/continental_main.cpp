#include <iostream>
//ROS
#include <ros/ros.h>
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

//Udp
#include "UdpServer.h"
using namespace std;

#define FRONTAL_IP  "192.168.1.111"
#define FRONTAL_PORT  9913

class PostProcess
{
	public:
	PostProcess(ros::NodeHandle& nodehandle) : nodehandle_(nodehandle),processthread_(NULL), processthreadfinished_(false)
	{
		memset(&continental_radar_data_, 0, sizeof(continental_radar_data_));
		init();
	}
	~PostProcess()
	{
		processthreadfinished_ = true;
		processthread_->join();
	}

	void init()
	{

		subcontinental_RadarData_ = nodehandle_.subscribe<frontal_mmw_radar_msgs::Continental_RadarData>(
				"Continental_radardata", 1, boost::bind(&PostProcess::Continental_RadarDataHandler, this, _1));

		processthread_ = new boost::thread(boost::bind(&PostProcess::process, this));
	}

	void Continental_RadarDataHandler(const frontal_mmw_radar_msgs::Continental_RadarDataConstPtr& radar_msg)
	{
		for (int i = 0 ; i < 64 ; i++)
		{
		  memset(&continental_radar_data_.continental_detection_array[i], 0, sizeof(moving_object_millimeter));
		}
		for (int i = 0; i < 64; ++i)
		{
			continental_radar_data_.continental_detection_array[i].target_ID =
					radar_msg->continental_detection_array[i].target_ID;
			continental_radar_data_.continental_detection_array[i].Relative_xv =
					radar_msg->continental_detection_array[i].Relative_xv;
			continental_radar_data_.continental_detection_array[i].Relative_yv =
					radar_msg->continental_detection_array[i].Relative_yv;
			continental_radar_data_.continental_detection_array[i].x =
					radar_msg->continental_detection_array[i].x;
			continental_radar_data_.continental_detection_array[i].y =
					radar_msg->continental_detection_array[i].y;
			continental_radar_data_.continental_detection_array[i].angle =
					radar_msg->continental_detection_array[i].angle;
			continental_radar_data_.continental_detection_array[i].Relative_acc_x =
					radar_msg->continental_detection_array[i].Relative_acc_x;
			continental_radar_data_.continental_detection_array[i].Obiect_class =
					radar_msg->continental_detection_array[i].Obiect_class;
			continental_radar_data_.continental_detection_array[i].Object_Length =
					radar_msg->continental_detection_array[i].Object_Length;
			continental_radar_data_.continental_detection_array[i].Object_Width =
					radar_msg->continental_detection_array[i].Object_Width;
		}
	}
	void process()
	{
		object_detection_.draw_basic_info();   //绘制网格线
	    udpSend m_udpsend(FRONTAL_IP , FRONTAL_PORT);
		while (!processthreadfinished_)
		{
			object_detection_.set_radar_data(continental_radar_data_);
			object_detection_.main_function2(Obj_pro_Send);
			m_udpsend.sendMsgs(Obj_pro_Send);
			object_detection_.DisplayAll();   //显示图像
		}

	}

	private:
	ros::NodeHandle& nodehandle_;
	ros::Subscriber subcontinental_RadarData_;
	boost::thread* processthread_;
	bool processthreadfinished_;
	continental_radar_target continental_radar_data_;
	ObjectDetection object_detection_;
	vector<moving_object_millimeter> Obj_pro_Send;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "frontal_continental_radar");
	ros::NodeHandle nh;

	PostProcess postprocess(nh);

	ros::spin();
}

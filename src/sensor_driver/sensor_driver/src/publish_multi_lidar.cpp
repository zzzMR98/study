/*!
* \file getmultivelodynedata.cpp
* \brief 雷达数据获取程序
*
*该文件是多个velodyne雷达数据获取程序
*
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
#include "velodyne/myhdl_grabber.h"
#include "rslidar_new/myhdl_grabber.h"
#include "common/lua_parameter_dictionary.h"
#include "common/configuration_file_resolver.h"
#include "common/make_unique.h"
#include "transform/rigid_transform.h"
#include "src/velodyne/proto/lidar_options.pb.h"
#include "common/blocking_queue.h"
#include <opencv2/opencv.hpp>
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <util/playback/iv_data_playback.h>
#include <util/ToXml.hh>
#include <util/xmlconf/xmlconf.h>
#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
#include "ins/imu531.h"
#endif
#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
#include "ins/AnalysisINS_OxT.h"
#endif
#include "ecu/AnalysisECU.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/ECUData.h"
#include "sensor_driver_msgs/InsVelocity.h"
//#include "sensor_driver_msgs/PointCloudMultiLidar.h"
#ifdef HAVE_PCAP
#include <pcap.h>
#include <pcap/sll.h>
#endif // #ifdef HAVE_PCAP
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


/// \brief 传感器配置
///
/// 传感器配置的基类
struct SensorOption
{
	virtual ~SensorOption(){}
	enum class Type { kGps, kLidar, kIns, kEcu };
	std::string ip;
	uint16_t port;
	Type type;
};

/// \brief 雷达传感器配置
///
struct LidarOption:public SensorOption
{
	LidarOption(){
		type = SensorOption::Type::kLidar;
	}
	  pcl::CalibrationValue calibrationvalue;
	  int id = 0;
	  double lasernum = 32;
	  std::string calibration_file;
	  std::string model = "velodyne";
};

/// \brief 惯导传感器配置
///
struct InsOption:public SensorOption
{
	InsOption(){
		type = SensorOption::Type::kIns;
	}
	  pcl::CalibrationValue calibrationvalue;
	  int id = 0;
};
/// \brief ｅｃｕ传感器配置
///
struct EcuOption:public SensorOption
{
	EcuOption(){
		type = SensorOption::Type::kEcu;
	}
	  double steeringratio_l = 0;
	  double steeringratio_r = 0;
	  int id = 0;
};

struct struct_INS_Mess
{

    double lTimeStamp;			// 时间戳(Unit:ms)
    double dLongitude;
    double dLatitude;
    double dHeading;
    double dPitch;
    double dRoll;

    double dAccx;
    double dAccy;
    double dAccz;

    double dArx;	//Pear
    double dAry;
    double dArz;


};

LidarOption createlidaroptions(    const string& configuration_directory,
	    const string& configuration_basename)
{
	  auto file_resolver = ::ivcommon::make_unique<
	      ::ivcommon::ConfigurationFileResolver>(
	      std::vector<string>{configuration_directory});
	  const string code =
	      file_resolver->GetFileContentOrDie(configuration_basename);

	  ::ivcommon::LuaParameterDictionary lua_parameter_dictionary(
	        code, std::move(file_resolver));

	  LidarOption options;
	  options.ip = lua_parameter_dictionary.GetString("ip");
	  options.port = lua_parameter_dictionary.GetInt("port");
	  options.lasernum = lua_parameter_dictionary.GetInt("lasernum");
	  options.calibrationvalue.x_offset = lua_parameter_dictionary.GetDouble("x_offset");
	  options.calibrationvalue.y_offset = lua_parameter_dictionary.GetDouble("y_offset");
	  options.calibrationvalue.z_offset = lua_parameter_dictionary.GetDouble("z_offset");
	  options.calibrationvalue.alfa = lua_parameter_dictionary.GetDouble("x_angle");
	  options.calibrationvalue.beta = lua_parameter_dictionary.GetDouble("y_angle");
	  options.calibrationvalue.gama = lua_parameter_dictionary.GetDouble("z_angle");
	  options.calibration_file = lua_parameter_dictionary.GetString("calibration_file");
	  options.model = lua_parameter_dictionary.GetString("model");
	  return options;
}



class Lidardata{
public:
	/// \brief 点云及其相关信息
	///
	 struct CloudwithID{
		  int id;
		  Cloud cloud;
		  double stamp;
		  bool calibrated;
	 };

  Lidardata(const std::string& configstr,ros::NodeHandle& nodehandle):
    configstr_(configstr)
    , hdl_read_data_thread_ (NULL)
	, ins_read_data_thread_ (NULL)
	, ecu_read_data_thread_ (NULL)
	,synchornizationcontainor_(new vector<int>)
	,nodehandle_(nodehandle)
  {
    init();
  };
  ~Lidardata()
  {
    close();
  }

  /// \brief 结束相关子线程，雷达grabber程序等
  ///
  void close ()
  {
	  for(int i = 0;i < grabberunitmaps_.size();i++)
	  {
		if(replay_ != 2)
		  {
			grabberunitmaps_[i].cloud_connection.disconnect();
			grabberunitmaps_[i].grabber->stop();
		  }
	  }
	  finish_read_data_thread_ = true;
	  if(hdl_read_data_thread_)
		  hdl_read_data_thread_->join();
	  if(ins_read_data_thread_)
	  {
		  insdatas_.stopQueue();
		  ins_read_data_thread_->join();
	  }

  }

  int get_lidar_num() {
	  return lidarnum_;
  }
  /// \brief 点云获取回调函数
  ///
  /// 获取点云数据
  /// \param cloud　会调的点云数据
  /// \param id 点云id 奇数代表被矫正过得点云，偶数为原始点云，id/2代表雷达编号
  void cloud_callback (const CloudConstPtr& cloud,int id)
  {
	  CHECK_GT(cloud->size(),0);
	  std::shared_ptr<CloudwithID> cloudwithid(new CloudwithID);

	  cloudwithid->id = id;
	  cloudwithid->stamp = cloud->header.stamp/1000000.0;
	  LOG(INFO)<<"id:"<<id;
	  CHECK_GT(cloud->size(),0);
	  cloudwithid->cloud = *cloud;
	  if(id%2==0)
	  {
		cloudwithid->calibrated = false;
	  }
	  else if(id%2==1)
		cloudwithid->calibrated = true;

	  cloudwithid_.Push(cloudwithid);
  }

  /// \brief 点云存储
  ///
  void Record()
  {
	int cloudnum = 0;
	for(auto it=finalcloudmap_origin_.begin();it!=finalcloudmap_origin_.end();it++)
	{
			cloudnum++;
	}
	if(cloudnum<=0)
		return;
	static int index=0;
	static std::string filename;
	playback_.BeginSaveLine();
	playback_<<cloudnum;
	for(auto it=finalcloudmap_origin_.begin();it!=finalcloudmap_origin_.end();it++)
	{
		filename.clear();
		filename=playback_.MakeRecordFileName(index,".pcd");
		playback_<<it->first<<playback_.SysTime(it->second->stamp)<<filename;

		filename=playback_.GetRecordPath()+filename;
		pcl::io::savePCDFile(filename, it->second->cloud,true);
		index++;
	}
	playback_.EndSaveLine();
  }

  /// \brief ｐｃｄ数据回放
  ///
	void getdatafrompcd()
	{
	  std::string filename;
	  finish_read_data_thread_ =false;
	  LOG(INFO)<<"getdatafrompcd";
	  while(!finish_read_data_thread_&&playback_.BeginLoadLine()==true)
		{
		    std::map<int,std::shared_ptr<CloudwithID>> tempcloudmap_origin;
		    std::map<int,std::shared_ptr<CloudwithID>> tempcloudmap;

			int count=0;
			playback_>>count;
//			LOG(INFO)<<"count:"<<count;
			for(int i=0;i<count;i++)
			{
				CloudwithID tempcloudwithid;
		        int id;
		        double stamp;
		        tempcloudmap_origin[id] = std::make_shared<CloudwithID>();
		        tempcloudmap[id] = std::make_shared<CloudwithID>();
				playback_>>id;
				playback_>>stamp>>filename;
				std::cout<<filename<<std::endl;
				filename=playback_.GetPlaybackPath()+filename;
				tempcloudmap_origin[id]->stamp = playback_.NowTime(stamp);
				pcl::io::loadPCDFile(filename,tempcloudmap_origin[id]->cloud);
				tempcloudmap_origin[id]->cloud.header.stamp = tempcloudmap_origin[id]->stamp * 1000000;
				*tempcloudmap[id] = *tempcloudmap_origin[id];

				precomputecloud(tempcloudmap[id]->cloud,grabberunitmaps_[id].calibvalue);
				pcl::transformPointCloud (tempcloudmap[id]->cloud, tempcloudmap[id]->cloud,
									   grabberunitmaps_[id].transform_matrix_Lidar2Vehicle_);
				filtercloudonvehicle(tempcloudmap[id]->cloud,grabberunitmaps_[id].calibvalue);
			}
			playback_.EndLoadLine();

			for(auto it=tempcloudmap_origin.begin();it!=tempcloudmap_origin.end();it++)
			{
				int id = it->first;
				CHECK_LT(id,lidarnum_)<<"id is bigger than lidarnum";

				cloud_callback(CloudConstPtr(new Cloud(it->second->cloud)),id*2);
				cloud_callback(CloudConstPtr(new Cloud(tempcloudmap[id]->cloud)),id*2+1);
			}
		}
	}
	  /// \brief ｐｃａｐ文件回放
	  ///
	  /// \param pcap_file_name_ 文件名
	void getdatafrompcap (std::string pcap_file_name_)
	{
		finish_read_data_thread_ =false;
		struct pcap_pkthdr *header;
		const unsigned char *data;
		char errbuff[PCAP_ERRBUF_SIZE];

		//std::cout<<"readPacketsFromPcap\t"<<pcap_file_name_.c_str ()<<std::endl;
		pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

		struct bpf_program filter;
		std::ostringstream stringStream;

		stringStream << "udp ";
//		if (!isAddressUnspecified(source_address_filter_))
//		{
//			stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
//		}
		stringStream << " and (dst port ";

		int i = 0;
		for(auto sensor:sensormaps_)
		{
			if(i==0)
			{
				stringStream <<sensor.first;
			}
			else
				stringStream<<" or dst port "<<sensor.first;
			i++;
		}
		stringStream<<")";
		LOG(WARNING)<<"filter:"<<stringStream.str();
		// PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
		if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
		{
			PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
		}
		else if (pcap_setfilter(pcap, &filter) == -1)
		{
			PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
		}

		MyTime totaltime;

		long long uSecDelay;

		long long autualtime=0;

		int returnValue=1;
		std::cout<<"pcap"<<std::endl;
		int count = 0;
		totaltime.start();

		while (!finish_read_data_thread_)
		{
//			std::cout<<"start"<<std::endl;
			int offset = 0;
			returnValue = pcap_next_ex(pcap, &header, &data);

			static long long start_tv_sec =header->ts.tv_sec;
			static long long start_tv_usec =header->ts.tv_usec;
//			std::cout<<"header->len:"<<header->len<<"\theader->caplen:"<<header->caplen<<std::endl;
			if(returnValue<0)
				break;
		    if (header->len != header->caplen) {
		      continue;
		    }

		    auto eth = reinterpret_cast<const ether_header *>(data);
		    auto ether_type = eth->ether_type;
//		    std::cout<<"ether_type:"<<ntohs(ether_type)<<std::endl;
		    if (ntohs(ether_type) != ETHERTYPE_IP) {
		    	auto sll = reinterpret_cast<const sll_header *>(data);
		    	ether_type = sll->sll_protocol;
		    	offset += sizeof(sll_header);
		     }
		    else
		    	offset += sizeof(ether_header);
//		    std::cout<<"ether_type:"<<ntohs(ether_type)<<std::endl;
		    if (ntohs(ether_type) != ETHERTYPE_IP) {
		       continue;
		     }


		    auto ip = reinterpret_cast<const iphdr *>(data + offset);
//		    std::cout<<"ip->version:"<<(int)ip->version<<std::endl;
		    if (ip->version != 4) {
		      continue;
		    }
		    offset += ip->ihl * 4;
		    auto udp = reinterpret_cast<const udphdr *>(data + offset);
//		    std::cout<<"ip->protocol:"<<(int)ip->protocol<<std::endl;
		    if (ip->protocol != IPPROTO_UDP) {
		      continue;
		    }
		    offset += sizeof(udphdr);
//		    std::cout<<"offset"<<offset<<std::endl;
		    std::string srcip_str = inet_ntoa({ip->saddr});
		    uint16_t port = ntohs(udp->dest);


			totaltime.stop();
			//    timecounter.stop();

			autualtime=static_cast<long long>(totaltime.gettime_s()*1000000);
			long long pcaptime = (header->ts.tv_sec-start_tv_sec)*1000000+header->ts.tv_usec-start_tv_usec;

			uSecDelay=pcaptime-autualtime;
			if(uSecDelay>1000000)
				LOG(ERROR)<<"delay too much time:"<<uSecDelay/1000000<<" "<<pcaptime;

			if(uSecDelay>0)
				boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));

//			if(count%100==0)
//			LOG(INFO)<<"port:"<<port;
//			count++;
//			LOG(INFO)<<"pcaptime:"<<pcaptime;
			if (sensormaps_.find(port)!=sensormaps_.end())
			{

				dispatchData(sensormaps_[port].get(),data + offset , header->len - offset);
			}


		}
//		std::cout<<"pcap end"<<std::endl;
		pcap_close(pcap);
//		LOG(INFO)<<"pcap end";
	}

  /// \brief 数据分发
  ///
  /// 进行不同传感器的数据分发，主要是ｐｃａｐ回放时使用
  /// \param sensor　传感器配置
  /// \param data　数据
  /// \param len　数据长度
	void dispatchData(SensorOption* sensor,const unsigned char * data ,int len){

		switch(sensor->type)
		{
		case SensorOption::Type::kLidar:
		{
			static int i = 0;
			i++;
			if(i>1000)
			{
				LOG(INFO)<<"get lidar data";
				i = 0;
			}

			grabberunitmaps_[dynamic_cast<LidarOption*>(sensor)->id].grabber->externenqueueHDLPacket(data,len);
			break;
		}
		case SensorOption::Type::kIns:
		{
			char* dstdata = new char[len];
			memcpy(dstdata,data,len);
			insdatas_.Push(std::make_shared<std::pair<int,char*>>(std::make_pair(len,dstdata)));
			break;
		}
		case SensorOption::Type::kEcu:
		{
			char* dstdata = new char[len];
			memcpy(dstdata,data,len);
			ecudatas_.Push(std::make_shared<std::pair<int,char*>>(std::make_pair(len,dstdata)));
			break;
		}
		default :
			break;
		}
	}

#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
  /// \brief 惯导数据解析和发布
  ///
	void insprocess()
	{
		finish_read_data_thread_ = false;
		while(!finish_read_data_thread_)
		{

			auto data = insdatas_.Pop();
			if(data == nullptr)
				continue;
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
//			LOG(INFO)<<readable_start_time;
//			LOG(INFO)<<"got ins data";

			m_INSFons_.ParseImu(data->second,data->first);

			delete [] data->second;

			float roll = m_INSFons_.m_sFONSData.dRoll*M_PI/180;
			float pitch = m_INSFons_.m_sFONSData.dPitch*M_PI/180;
			float heading = (m_INSFons_.m_sFONSData.dHeading)*M_PI/180;
			tf::Quaternion orientation=tf::createQuaternionFromRPY(roll, pitch, heading);

			sensor_msgs::Imu imuout;
			tf::quaternionTFToMsg(orientation,imuout.orientation);
			imuout.header.stamp = ros::Time::now();
			imuout.header.frame_id="imu_frame";
			imuout.linear_acceleration.x = m_INSFons_.m_sFONSData.dAccz;
			imuout.linear_acceleration.y = m_INSFons_.m_sFONSData.dAccx;
			imuout.linear_acceleration.z = m_INSFons_.m_sFONSData.dAccy;

			imuout.angular_velocity.x = m_INSFons_.m_sFONSData.dArz*M_PI/180;
			imuout.angular_velocity.y = m_INSFons_.m_sFONSData.dArx*M_PI/180;
			imuout.angular_velocity.z = m_INSFons_.m_sFONSData.dAry*M_PI/180;
			sensor_driver_msgs::GpswithHeading gps;

			gps.gps.header.frame_id = "gps_frame";
			gps.gps.header.stamp = imuout.header.stamp;
			gps.gps.latitude = m_INSFons_.m_sFONSData.dFOSNLat;
			gps.gps.longitude = m_INSFons_.m_sFONSData.dFOSNLng;
			gps.heading = m_INSFons_.m_sFONSData.dHeading;
			gps.gps.altitude = m_INSFons_.m_sFONSData.dFOSNAltitude;
			if(gps.heading<-180)
				gps.heading +=360;
//			if(m_INSFons_.m_sFONSData.dHeading < 0)
//				gps.heading = - m_INSFons_.m_sFONSData.dHeading;
//			else
//				gps.heading = 360 - m_INSFons_.m_sFONSData.dHeading;
			gps.pitch = m_INSFons_.m_sFONSData.dPitch;
			gps.roll = m_INSFons_.m_sFONSData.dRoll;
			unsigned int utc_time = m_INSFons_.m_sFONSData.utc_time;
			static auto last_utc_time = utc_time;
			static auto last_time = imuout.header.stamp;
			double diff_utctime = (utc_time - last_utc_time)%10000/1000.;
			double diff_time = (imuout.header.stamp - last_time).toSec();
//			if(diff_utctime>0.06||diff_utctime<-0.01||utc_time<last_utc_time)
//				LOG(ERROR)<<"too much utc time:"<<diff_utctime;
//
//			if(diff_time>0.06)
//				LOG(ERROR)<<"too much diff time:"<<diff_time;
			last_utc_time = utc_time;
			last_time = imuout.header.stamp;
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

			LOG(INFO)<<readable_start_time;
//			LOG(INFO)<<"gps.heading="<<gps.heading;
			pubImudata_.publish(imuout);
			pubGps_.publish(gps);
			sensor_driver_msgs::InsVelocity insvelocity;
			insvelocity.header.frame_id = "ins_frame";
			insvelocity.header.stamp = imuout.header.stamp;
			insvelocity.angular_velocity = imuout.angular_velocity;
			Eigen::Vector3d globalvelocity;
			globalvelocity.x() = m_INSFons_.m_sFONSData.dVelE;
			globalvelocity.y() = m_INSFons_.m_sFONSData.dVelN;
			globalvelocity.z() = m_INSFons_.m_sFONSData.dVelSky;

			auto inspose = ::ivcommon::transform::RollPitchYaw(roll, pitch, heading);
			Eigen::Vector3d velocity = inspose.inverse()*globalvelocity;
			insvelocity.linear_velocity.x = velocity.x();
			insvelocity.linear_velocity.y = velocity.y();
			insvelocity.linear_velocity.z = velocity.z();
			if(velocity.norm()<100)
				pubInsvelocity_.publish(insvelocity);
//			LOG(INFO)<<"publish ins data";
		}
	}
#endif
#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
	  /// \brief 惯导数据解析和发送　（牛津惯导）
	  ///
	void insprocess()
	{
		finish_read_data_thread_ = false;
		while(!finish_read_data_thread_)
		{
			auto data = insdatas_.Pop();
			if(data == nullptr)
				continue;
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
//			LOG(INFO)<<readable_start_time;
			LOG(INFO)<<"got ins data";

			m_AnalysisINS_OxT.ParseImu(data->second,data->first);

			delete [] data->second;

			float roll = m_AnalysisINS_OxT.INSData_struct.dRoll*M_PI/180;
			float pitch = m_AnalysisINS_OxT.INSData_struct.dPitch*M_PI/180;
			float heading = (360-m_AnalysisINS_OxT.INSData_struct.dHeading)*M_PI/180;
			tf::Quaternion orientation=tf::createQuaternionFromRPY(roll, pitch, heading);

			sensor_msgs::Imu imuout;
			tf::quaternionTFToMsg(orientation,imuout.orientation);
			imuout.header.stamp = ros::Time::now();
			imuout.header.frame_id="imu_frame";
			imuout.linear_acceleration.x = m_AnalysisINS_OxT.INSData_struct.dAccy;
			imuout.linear_acceleration.y = m_AnalysisINS_OxT.INSData_struct.dAccx;
			imuout.linear_acceleration.z = -m_AnalysisINS_OxT.INSData_struct.dAccz;

			imuout.angular_velocity.x = m_AnalysisINS_OxT.INSData_struct.dGyroY*M_PI/180;
			imuout.angular_velocity.y = m_AnalysisINS_OxT.INSData_struct.dGyroX*M_PI/180;
			imuout.angular_velocity.z = -m_AnalysisINS_OxT.INSData_struct.dGyroZ*M_PI/180;
			sensor_driver_msgs::GpswithHeading gps;

			gps.gps.header.frame_id = "gps_frame";
			gps.gps.header.stamp = imuout.header.stamp;
			gps.gps.latitude = m_AnalysisINS_OxT.INSData_struct.dLat;
			gps.gps.longitude = m_AnalysisINS_OxT.INSData_struct.dLng;
			gps.heading = 360-m_AnalysisINS_OxT.INSData_struct.dHeading;
			gps.gps.altitude = m_AnalysisINS_OxT.INSData_struct.dAltitude;
			if(gps.heading<-180)
				gps.heading +=360;
			if(gps.heading>180)
				gps.heading -=360;
//			if(m_AnalysisINS_OxT.INSData_struct.dHeading < 0)
//				gps.heading = - m_AnalysisINS_OxT.INSData_struct.dHeading;
//			else
//				gps.heading = 360 - m_AnalysisINS_OxT.INSData_struct.dHeading;
			gps.pitch = m_AnalysisINS_OxT.INSData_struct.dPitch;
			gps.roll = m_AnalysisINS_OxT.INSData_struct.dRoll;


//			LOG(INFO)<<h<<"-"
//					<<min<<"-"
//					<<s<<":"
//					<<ms;
//			LOG(INFO)<<"gps.heading="<<gps.heading;
			pubImudata_.publish(imuout);
			pubGps_.publish(gps);
			sensor_driver_msgs::InsVelocity insvelocity;
			insvelocity.header.frame_id = "ins_frame";
			insvelocity.header.stamp = imuout.header.stamp;
			insvelocity.angular_velocity = imuout.angular_velocity;
			Eigen::Vector3d globalvelocity;
			globalvelocity.x() = m_AnalysisINS_OxT.INSData_struct.dEastV;
			globalvelocity.y() = m_AnalysisINS_OxT.INSData_struct.dNorthV;
			globalvelocity.z() = m_AnalysisINS_OxT.INSData_struct.dAltitudeV;

			auto inspose = ::ivcommon::transform::RollPitchYaw(roll, pitch, heading);
			Eigen::Vector3d velocity = inspose.inverse()*globalvelocity;
			insvelocity.linear_velocity.x = velocity.x();
			insvelocity.linear_velocity.y = velocity.y();
			insvelocity.linear_velocity.z = velocity.z();
			if(velocity.norm()<100)
				pubInsvelocity_.publish(insvelocity);
//			LOG(INFO)<<"publish ins data";
		}
	}
#endif
	  /// \brief ＥＣＵ数据解析和发布
	  ///
	void ecuprocess()
	{
		finish_read_data_thread_ = false;
		ros::Rate rate(50);
		while(!finish_read_data_thread_)
		{
			int datanum = ecudatas_.Size();
			if(datanum>0)
			{
				for(int i=0;i<datanum;i++)
				{
					auto data = ecudatas_.Pop();
					m_AnalysisECU_.ParseEcu(data->second,data->first);
					delete [] data->second;
				}

//				LOG(INFO)<<"f_shift:"<<(int)m_AnalysisECU_.ECUData_struct.f_shift1;
//				LOG(INFO)<<"fDeForwardVel:"<<m_AnalysisECU_.ECUData_struct.fDeForwardVel;
//				LOG(INFO)<<"fFLRWheelAverAngle:"<<m_AnalysisECU_.ECUData_struct.fFLRWheelAverAngle;
				LOG(INFO)<<"fForwardVel:"<<m_AnalysisECU_.ECUData_struct.fForwardVel;
//				LOG(INFO)<<"petral_pressure:"<<m_AnalysisECU_.ECUData_struct.petral_pressure;
//				LOG(INFO)<<"pressure_back:"<<m_AnalysisECU_.ECUData_struct.pressure_back;
//				LOG(INFO)<<"FrontLeftWheelSpeed:"<<m_AnalysisECU_.ECUData_struct.FrontLeftWheelSpeed;
//				LOG(INFO)<<"FrontRightWheelSpeed:"<<m_AnalysisECU_.ECUData_struct.FrontRightWheelSpeed;
//				LOG(INFO)<<"BackLeftWheelSpeed:"<<m_AnalysisECU_.ECUData_struct.BackLeftWheelSpeed;//1
//				LOG(INFO)<<"BackRightWheelSpeed:"<<m_AnalysisECU_.ECUData_struct.BackRightWheelSpeed;

				sensor_driver_msgs::ECUData ecumsg;
				ecumsg.header.stamp=ros::Time::now();
				ecumsg.f_shift = m_AnalysisECU_.ECUData_struct.f_shift;
				ecumsg.f_shift1 = m_AnalysisECU_.ECUData_struct.f_shift1;
				ecumsg.fDeForwardVel=m_AnalysisECU_.ECUData_struct.fDeForwardVel;
				ecumsg.fFLRWheelAverAngle=m_AnalysisECU_.ECUData_struct.fFLRWheelAverAngle;
				ecumsg.fForwardVel=m_AnalysisECU_.ECUData_struct.fForwardVel;

				ecumsg.petral_pressure=m_AnalysisECU_.ECUData_struct.petral_pressure;
				ecumsg.pressure_back=m_AnalysisECU_.ECUData_struct.pressure_back;
				ecumsg.FrontLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontLeftWheelSpeed;
				ecumsg.FrontRightWheelSpeed=m_AnalysisECU_.ECUData_struct.FrontRightWheelSpeed;
				ecumsg.BackLeftWheelSpeed=m_AnalysisECU_.ECUData_struct.BackLeftWheelSpeed;//1
				ecumsg.BackRightWheelSpeed=m_AnalysisECU_.ECUData_struct.BackRightWheelSpeed;
				pubEcudata_.publish(ecumsg);
			}
//			LOG(INFO)<<"publish ins data";
			rate.sleep();
		}
	}

  /// \brief 保存一帧雷达数据
  ///
  /// \param index　雷达点云索引号，用于文件名
  /// \param cloudwithid　点云
	void SaveOneFrame(int index,std::shared_ptr<CloudwithID> cloudwithid)
	{
		if(cloudwithid->cloud.size()>0)
		{
			std::stringstream sstr;
			sstr<<index<<"_"<<cloudwithid->id/2<<".pcd";
			std::string path = calibrationpcddir_ + sstr.str();
			pcl::io::savePCDFile(path, cloudwithid->cloud,false);
		}
	}

	void initins()
		{
			XmlConf gpsconfig;
		    if(!gpsconfig.Parse(configstr_.c_str(), "GetINSData"))
		    {
		  	  std::cout<<"GetINSData  is not exist in config xml  file"<<std::endl;
		    }
			bool withins = true;
			if(!gpsconfig.GetSystemParam("GetINSData_on",withins))
			{
				std::cout<<"GetINSData模块的开关不存在"<<std::endl;
			}
			else
			{
				if(withins)
				std::cout<<"GetINSData 模块开启"<<std::endl;
				else
				std::cout<<"GetINSData 模块关闭"<<std::endl;
			}
			if(withins)
			{
				int port;
				if(!gpsconfig.GetModuleParam("port",port))
				{
				    std::cout<<"port num is incorrect"<<std::endl;
				}
				else
					std::cout<<"port ="<<port<<std::endl;
				InsOption gpsoption;
				gpsoption.port = port;
				sensormaps_[port] = std::make_shared<InsOption>(gpsoption);
				ins_read_data_thread_ = new boost::thread (boost::bind (&Lidardata::insprocess, this));
				pubImudata_ = nodehandle_.advertise<sensor_msgs::Imu> ("imudata", 50);

				pubInsvelocity_ = nodehandle_.advertise<sensor_driver_msgs::InsVelocity> ("insvelocity", 50);
				pubGps_ = nodehandle_.advertise<sensor_driver_msgs::GpswithHeading> ("gpsdata", 50);
			}

		}

	void initecu()
	{
		XmlConf ecuconfig;
	    if(!ecuconfig.Parse(configstr_.c_str(), "GetECUData"))
	    {
	  	  std::cout<<"GetECUData  is not exist in config xml  file"<<std::endl;
	    }
		bool withecu = true;
		if(!ecuconfig.GetSystemParam("GetECUData_on",withecu))
		{
			std::cout<<"GetECUData模块的开关不存在"<<std::endl;
			withecu = false;
		}
		else
		{
			if(withecu)
			std::cout<<"GetECUData 模块开启"<<std::endl;
			else
			std::cout<<"GetECUData 模块关闭"<<std::endl;
		}
		if(withecu)
		{
			int port;
			double steeringratio_l=0;
			double steeringratio_r=0;
			if(!ecuconfig.GetModuleParam("port",port))
			{
			    std::cout<<"port num is incorrect"<<std::endl;
			}
			else
				std::cout<<"port ="<<port<<std::endl;


			if(!ecuconfig.Parse(configstr_.c_str(), "VehicleParam"))//获得参数0829，qjy
			{
				std::cout<<"VehicleParam  is not exist in config xml  file"<<std::endl;
			}
			if(!ecuconfig.GetModuleParam("steeringratio_l",steeringratio_l))
			{
				std::cout<<"steeringratio_l num is incorrect"<<std::endl;
			}
			else
			std::cout<<"steeringratio_l is "<<steeringratio_l<<std::endl;

			if(!ecuconfig.GetModuleParam("steeringratio_r",steeringratio_r))
			{
				std::cout<<"steeringratio_r num is incorrect"<<std::endl;
			}
			else
			std::cout<<"steeringratio_r is "<<steeringratio_r<<std::endl;

			EcuOption ecuoption;
			ecuoption.port = port;
			ecuoption.steeringratio_l = steeringratio_l;
			ecuoption.steeringratio_r = steeringratio_r;

			sensormaps_[port] = std::make_shared<EcuOption>(ecuoption);
			ecu_read_data_thread_ = new boost::thread (boost::bind (&Lidardata::ecuprocess, this));

			pubEcudata_ = nodehandle_.advertise<sensor_driver_msgs::ECUData> ("ecudata", 50);
		}

	}

  /// \brief 程序初始化
  ///
  /// 程序初始化主要是各个传感器，线程、ｒｏｓ话题的初始化
  bool init()
  {
	initins();
	initecu();
    if(!xmlconfig_.Parse(configstr_.c_str(), "iv_lidar"))
    {
  	  std::cout<<"iv_lidar  is not exist in config xml  file"<<std::endl;
    }
    else
      {
	ConfigParam();
	if(lidarnum_==0)
	{
		std::cout<<"lidar num cannot be zero"<<std::endl;
	}

	float minDistanceThreshold = 1;
	float maxDistanceThreshold = 120;
	float velodyneheight = 1.2;

	playback_.Setup(xmlconfig_);
	if(playback_.PlaybackIsOn())
	{
		replay_=playback_mode;
	}
	else
	replay_=0;
	//replay_=1;

	if(calibrationmode_)
		calibrationpcddir_ = ::ivcommon::createStampedDir("~/calibrationlidar") + "/";
	std::string pkgPath = ros::package::getPath("sensor_driver");
	std::string calibdirpath = pkgPath + "/config/extrinsicparameter/" + calibdirname_;
	synchornizationcontainor_->resize(lidarnum_,0);
	if(lidarnum_>0)
		synchornizationcontainor_->front() = 1;
	for(int i=0;i<lidarnum_;i++)
	{
		std::stringstream sstr;
		sstr<<i<<".lua";
		LidarOption option = createlidaroptions(calibdirpath,sstr.str());
		option.id = i;
		sensormaps_[option.port] = std::make_shared<LidarOption>(option);
		std::string intriparacalibration;

		intriparacalibration =pkgPath+ "/config/";//塔河比赛丰田
		intriparacalibration = intriparacalibration+option.calibration_file;
		if(option.calibration_file == "")
		{
			LOG(WARNING)<<"intriparacalibration file is invalid";
			intriparacalibration = "";
		}
		if(replay_==1 || replay_==2)
		{
			boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(option.ip);
			unsigned short  listen_port = option.port;

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
			if(option.model == "velodyne")
				grabberunitmaps_[i].grabber=new pcl::VelodyneGrabber(intriparacalibration,pcapFile_,option.lasernum
																,i,synchornizationcontainor_);
			else if(option.model == "rslidar")
				grabberunitmaps_[i].grabber=new pcl::RslidarGrabber(intriparacalibration,pcapFile_,option.lasernum
																	,i,synchornizationcontainor_);
			grabberunitmaps_[i].grabber->setMaximumDistanceThreshold(maxDistanceThreshold);
			grabberunitmaps_[i].grabber->setMinimumDistanceThreshold(minDistanceThreshold);
			//grabberunitmaps_[i].grabber->filterPackets(listen_ip,listen_port);


		}
		else
		{
			std::cout<<"realtime start"<<std::endl;
			std::cout<<option.ip.c_str()<<std::endl;
			boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(option.ip);
			unsigned short  listen_port = option.port;
			std::cout<<listen_port<<std::endl;
			if(option.model == "velodyne")
				grabberunitmaps_[i].grabber=new pcl::VelodyneGrabber(listen_ip, listen_port, intriparacalibration,option.lasernum
																,i,synchornizationcontainor_);
			else if(option.model == "rslidar")
				grabberunitmaps_[i].grabber=new pcl::RslidarGrabber(listen_ip, listen_port, intriparacalibration,option.lasernum
																,i,synchornizationcontainor_);
			else
				LOG(FATAL)<<"lidar model "<<option.model<<" is invalid";
			grabberunitmaps_[i].grabber->setMaximumDistanceThreshold(maxDistanceThreshold);
			grabberunitmaps_[i].grabber->setMinimumDistanceThreshold(minDistanceThreshold);
		}
		grabberunitmaps_[i].calibvalue = option.calibrationvalue;

		grabberunitmaps_[i].lasernum = option.lasernum;

		pcl::get_transform_matrix(grabberunitmaps_[i].calibvalue,grabberunitmaps_[i].transform_matrix_Lidar2Vehicle_);

		boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,int)> cloud_cb
			= boost::bind(&Lidardata::cloud_callback,this,_1,_2);
		grabberunitmaps_[i].cloud_connection = grabberunitmaps_[i].grabber->registerCallback(cloud_cb);

		if(replay_ != 2)
		  {
			if(replay_==1)
				grabberunitmaps_[i].grabber->setDataFromExtern();
			grabberunitmaps_[i].grabber->setCalibration(grabberunitmaps_[i].calibvalue);
			grabberunitmaps_[i].grabber->start ();
		  }

	}

	if(replay_ == 2)
	  {
		hdl_read_data_thread_ = new boost::thread (boost::bind (&Lidardata::getdatafrompcd, this));
	  }
	else if(replay_ == 1)
	{
		if(!xmlconfig_.GetModuleParam("pcapfile",pcapFile_))
		{
			std::cout<<"pcapfile is incorrect"<<std::endl;
		}
		else
		std::cout<<"pcapfile is "<<pcapFile_<<std::endl;
		hdl_read_data_thread_ = new boost::thread (boost::bind (&Lidardata::getdatafrompcap, this,pcapFile_));
	}

    updated_ = false;
	return true;
      }
    return false;
  }

  /// \brief ｘｍｌ文件雷达部分配置解析
  ///
  void ConfigParam()
  {
  	//对于config.xml里面的参数的赋值


	if(!xmlconfig_.GetModuleParam("lidarnum",lidarnum_))
	{
		std::cout<<"lidarnum is incorrect"<<std::endl;
	}
	else
	std::cout<<"lidarnum is "<<lidarnum_<<std::endl;

	if(!xmlconfig_.GetModuleParam("calibdir",calibdirname_))
	{
		std::cout<<"calibdirname is incorrect"<<std::endl;
	}
	else
	std::cout<<"calibdirname is "<<calibdirname_<<std::endl;

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

	calibrationmode_ = false;
	if(!xmlconfig_.GetModuleParam("calibrationmode",calibrationmode_))
	{
		std::cout<<"calibrationmode_ is incorrect"<<std::endl;
	}
	else
	std::cout<<"calibrationmode_ is "<<calibrationmode_<<std::endl;

  }
  /// \brief 点云信息增加到待发点云中，便于多个雷达时的点云的打包发送与后续解析
  ///
  /// \param pointcloud　点云
  /// \param indexmaptable　每条线编号的映射
  /// \param lasernum　雷达数目
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
  /// \brief 点云后处理
  ///
  /// 进行点云的保存、多雷达数据的打包
  void  postprocess()
  {
	  if(playback_.RecordIsOn())
		  Record();
	//   if(calibrationmode_)
	// 	  savecalibrationpcd();//多雷达的原始点云
//	  convertcloudmsg();
	  mixcloud(finalcloudmap_origin_,totalorigincloud_);
	  mixcloud(finalcloudmap_calibrated_,totalcloud_);
	//   if(calibrationmode_)
	// 	  savetotalpcd();//经过坐标转换的总的点云
	  resetcloudstate();
  }

  /// \brief 分别保存多雷达的原始点云，用于标定雷达之间的参数
  ///
  /// 分别保存多雷达的原始点云，用于标定雷达之间的参数
  void savecalibrationpcd()
  {
	 int key=0;
	 static int index = 0;
	 cv::Mat img=cv::Mat::zeros(100,100,CV_8U);
	 cv::imshow("waitkey",img);
	 key = cv::waitKey(1);
	 if(key=='s')
	 {
		 index++;
		for(auto it=finalcloudmap_origin_.begin();it!=finalcloudmap_origin_.end();it++)
		{
			SaveOneFrame(index,it->second);
		}
	 }
  }

  /// \brief 保存多雷达数据组成的总的点云数据，用于标定地面
  ///
  /// 保存多雷达数据组成的总的点云数据，成为一个ｐｃｄ文件，用于多雷达之间标定后总的地面标定或者单个雷达的地面标定矫正俯仰侧倾和高度值
  void savetotalpcd()
  {
	 int key=0;
	 static int index = 0;
	 cv::Mat img=cv::Mat::zeros(100,100,CV_8U);
	 cv::imshow("waitkey",img);
	 key = cv::waitKey(1);

	 if(key=='s')
	 {
		 Cloud filtercloud_;
		 for(int i=0;i<totalcloud_.size();i++)
		 {
			 if(totalcloud_[i].range>0.5)
				 filtercloud_.push_back(totalcloud_[i]);
		 }
		index++;
		std::stringstream sstr;
		sstr<<index<<"_"<<"total"<<".pcd";
		std::string path = calibrationpcddir_ + sstr.str();
		pcl::io::savePCDFile(path, filtercloud_,false);
	 }
  }

  /// \brief 多雷达点云打包为一个，便于发送
  ///
  /// \param cloudmap　多雷达点云
  /// \param totalcloud　打包后的大点云
  void mixcloud(std::map<int,std::shared_ptr<CloudwithID>>& cloudmap,Cloud& totalcloud)
  {
	std::map<int,int> cloudstartnum;
	totalcloud.clear();
	pcl::uint64_t maxstamp;
	int i=0;
	LOG_IF(ERROR,cloudmap.size()<lidarnum_)<<"cloudmap.size()<lidarnum_";
	for(auto it=cloudmap.begin();it!=cloudmap.end();it++)
	{
		int lidarid = it->first;
		cloudstartnum[lidarid]=(totalcloud.size());
		if(i==0)
		{
			CHECK_GT(it->second->cloud.size(),0);
			maxstamp = it->second->cloud.header.stamp;
			totalcloud .swap(it->second->cloud);
		}
		else
		{
			CHECK_GT(it->second->cloud.size(),0);

			if(maxstamp<it->second->cloud.header.stamp)
				maxstamp = it->second->cloud.header.stamp;
			totalcloud += it->second->cloud;
		}
		i++;
	}
	for(auto it=cloudmap.begin();it!=cloudmap.end();it++)
	{
		int lidarid = it->first;
		LOG(INFO)<<"cloudstartnum[lidarid]="<<cloudstartnum[lidarid];
		pcl::PointXYZI temppoint;
		temppoint.x = grabberunitmaps_[lidarid].calibvalue.x_offset;
		temppoint.y = grabberunitmaps_[lidarid].calibvalue.y_offset;
		temppoint.z = grabberunitmaps_[lidarid].calibvalue.z_offset;
		temppoint.azimuth=grabberunitmaps_[lidarid].lasernum+0.5; //线数
		temppoint.range = cloudstartnum[lidarid] + 0.5; //第二个点云的起点
		temppoint.passibility = 1;
		totalcloud.push_back(temppoint);
	}
	totalcloud.header.stamp = maxstamp;
	cloudmap.clear();
  }

bool isvalid()
{
  return  updated_;
}

/// \brief 重置点云的更新状态
///
void resetcloudstate()
{
  updated_ = false;
}
/// \brief 更新点云的range azimuth passibility 变量
///
/// 更新点云的range azimuth passibility 变量，range　代表点云的距离，azimuth代表点云的角度，passibility代表点云的可通行概率
///（这三个变量的意思是人为定的，后面也会用来存储其他意思的值，比如雷达线的编号以及打包时便于解析的值（如点云数目等））
/// range<0 一般代表无效点云，使用时注意过滤，这里仍然保留range<0的点云是为了保持点云的排布顺序不变，便于一些需要原始数据、线编号的程序的使用
/// \param pointcloud 输入点云
/// \param calibvalue　雷达标定参数，这里用于矫正ａｚｉｍｕｔｈ
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
 /// \brief 滤出车辆周围的点云
 ///
 /// 将车辆周围的点云置为ｒａｎｇｅ<０
 /// \param pointcloud 输入点云
 /// \param calibvalue　标定参数
 static void filtercloudonvehicle(Cloud& pointcloud,pcl::CalibrationValue& calibvalue)
 {
  for (int i = 0; i < pointcloud.points.size(); i++)
  {
      //range < 0.5 ��Ч
	  pcl::PointXYZI& temppoint = pointcloud.points[i];
      if(temppoint.range>0&&((temppoint.x > -fabs(1.5) && temppoint.x < fabs(1.5) &&
	  temppoint.y < 4 && temppoint.y > -2)))
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
 /// \brief 点云预处理根据雷达ｉｄ暂存到ｍａｐ中
 ///
 void preprocess()
 {
	 finalcloudmap_origin_.clear();
	 finalcloudmap_calibrated_.clear();
	 std::shared_ptr<CloudwithID> cloudwithid;

	 double interval = 0.02;
	 LOG(INFO)<<"start";

	 while((cloudwithid = cloudwithid_.PopWithTimeout(::ivcommon::FromSeconds(interval)))!=nullptr)
   {
//		if(playback_.RecordIsOn()&&cloudwithid->id%2==0)
//			Record(cloudwithid);
	   int lidarid = cloudwithid->id/2;
	   LOG(INFO)<<"lidarid:"<<lidarid;
	   if(!cloudwithid->calibrated)
	   {
		   LOG_IF(ERROR,finalcloudmap_origin_.find(lidarid)!=finalcloudmap_origin_.end())
				   <<"finalcloudmap_origin_.find(lidarid)!=finalcloudmap_origin_.end()";
		   addcloudinfo(cloudwithid->cloud,
				   grabberunitmaps_[lidarid].grabber->indexmaptable,grabberunitmaps_[lidarid].lasernum);
		   finalcloudmap_origin_[lidarid] = cloudwithid;
		   LOG(INFO)<<"lidarid:"<<lidarid;
		   continue;
	   }
	   updated_=true;

	   LOG_IF(ERROR,finalcloudmap_calibrated_.find(lidarid)!=finalcloudmap_calibrated_.end())
	   	   	   	   <<"finalcloudmap_calibrated_.find(lidarid)!=finalcloudmap_calibrated_.end()";
	   addcloudinfo(cloudwithid->cloud,
				   grabberunitmaps_[lidarid].grabber->indexmaptable,grabberunitmaps_[lidarid].lasernum);
	   finalcloudmap_calibrated_[lidarid] = cloudwithid;
	   LOG(INFO)<<"lidarid:"<<lidarid;
	   if((lidarid == lidarnum_-1)
			   &&finalcloudmap_calibrated_.size()==lidarnum_)
		   break;
   }
 }
 const Cloud& gettotalcloud()
 {
//	 auto fieldmap = pcl::detail::getMapping(totalcloud_);
//	 LOG(INFO)<<"size:"<<fieldmap->size();
   return totalcloud_;
 }

 const Cloud& gettotalorigincloud()
 {
//	 auto fieldmap = pcl::detail::getMapping(totalcloud_);
//	 LOG(INFO)<<"size:"<<fieldmap->size();
   return totalorigincloud_;
 }

 /// \brief 雷达ｇｒａｂｂｅｒ信息组
 ///
 /// 主要包括雷达线数、标定参数、标定矩阵、ｇｒａｂｂｅｒ指针、信号
 struct GrabberUnit{
	  int lasernum;
	  pcl::CalibrationValue calibvalue;
	  Eigen::Matrix4f transform_matrix_Lidar2Vehicle_;
	  pcl::IVGrabber* grabber;
	  boost::signals2::connection cloud_connection;
 };
  std::map<int,std::shared_ptr<CloudwithID>> finalcloudmap_origin_;//!<最后的原始点云映射
private:
  IvDataPlayback playback_;//!<　数据回放
  XmlConf xmlconfig_;//!<　ｘｍｌ解析

  int replay_;//!<回放标志
  bool calibrationmode_;//!<标定模式
  bool playback_on_;//!<回放打开
  int playback_mode;//!<回放模式
  bool record_on;//!<数据保存开
  bool finish_read_data_thread_;//!<读数据线程完成标志
  std::string pcapFile_;//!<ｐｃａｐ文件名

  std::string configstr_;//!<ｘｍｌ配置字符串
  bool updated_;//!<点云更新标志

  Cloud totalcloud_;//!<打包后的点云
  Cloud totalorigincloud_;//!<打包后的原始点云
  boost::thread *hdl_read_data_thread_;//!<雷达数据解析线程
  boost::thread *ins_read_data_thread_;//!<惯导数据解析线程
  boost::thread *ecu_read_data_thread_;//!<ｅｃｕ数据解析线程
  int lidarnum_;//!<雷达数目
  boost::shared_ptr<std::vector<int>> synchornizationcontainor_;//!<多雷达信号同步器
  ::ivcommon::BlockingQueue<std::shared_ptr<CloudwithID>> cloudwithid_;//!<回调获得的点云
  ::ivcommon::BlockingQueue<std::shared_ptr<std::pair<int,char*>>> insdatas_;//!<惯导数据
  ::ivcommon::BlockingQueue<std::shared_ptr<std::pair<int,char*>>> ecudatas_;//!<ｅｃｕ数据
//   std::map<int,std::shared_ptr<CloudwithID>> finalcloudmap_origin_;//!<最后的原始点云映射
  std::map<int,std::shared_ptr<CloudwithID>> finalcloudmap_calibrated_;//!<标定的点云映射
  std::map<int,GrabberUnit> grabberunitmaps_;//!<各个雷达ｇｒａｂｂｅｒ单元
  std::string calibdirname_;//!<存储标定数据
  std::string calibrationpcddir_;//!<存储标定数据
  std::map<uint16_t,std::shared_ptr<SensorOption>> sensormaps_;//!<传感器组
#if (defined TANGPLATFORM_KI0E5)||(defined TOYOTAPLATFORM)//如果是有人唐
  Imu_531 m_INSFons_;//!<惯导解析类
#endif
#if(defined BYDRAYYUANZHENG)||(defined FOTONBUS)||(defined HUACHEN)//如果是元征的速锐或福田客车（即装了牛津惯导的设备）
  CAnalysisINS_OxT m_AnalysisINS_OxT;//!<惯导解析类
#endif
  CAnalysisECU m_AnalysisECU_;//!<ｅｃｕ解析
  ros::NodeHandle nodehandle_;//!<公有ｈａｎｄｌｅ
  ros::Publisher pubImudata_;//!<发布ｉｍｕ数据
  ros::Publisher pubInsvelocity_;//!<发布惯导速度信息
  ros::Publisher pubGps_;//!<发布ＧＰＳ信息
  ros::Publisher pubEcudata_;//!<发布ｅｃｕ信息
//  sensor_driver_msgs::PointCloudMultiLidarPtr pointcloudmultilidarptr;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_multi_lidar");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
//  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

//   ros::Publisher pubLaserCloud;
//   pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
//                                  ("lidar_cloud_calibrated", 2);

//   ros::Publisher pubOriginCloud;
//   pubOriginCloud = nh.advertise<sensor_msgs::PointCloud2>
//                                  ("lidar_cloud_origin", 2);
//  ros::Publisher pubPointCloudMultilidar;
//  pubPointCloudMultilidar = nh.advertise<sensor_driver_msgs::PointCloudMultiLidar>
//                                 ("/pointcloudmultilidar", 2);

  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
   {
     ros::Duration(0.01).sleep();
   }

  std::string startconfig = configsrv.response.configstr;

  bool fixframe = false;
  ros::param::get("~fix_frame",fixframe);
  Lidardata lidar(startconfig,nh);
  int lidarnum = lidar.get_lidar_num();
  std::map<int,ros::Publisher> lidar_publisher;
  for (int pub_idx=0; pub_idx<lidarnum; pub_idx++) {
	  std::stringstream pub_name_ss;
	  pub_name_ss << "lidar_raw_" << pub_idx;
		lidar_publisher[pub_idx] = nh.advertise<sensor_msgs::PointCloud2>
                                 (pub_name_ss.str(), 2);
  }
  ros::Rate rate(1000);
  bool status = ros::ok();
  while(ros::ok())
    {
	  lidar.preprocess();
	  if(!lidar.isvalid())
		  continue;

	for(auto it=lidar.finalcloudmap_origin_.begin();it!=lidar.finalcloudmap_origin_.end();it++)
	{
		CHECK_GT(it->second->cloud.size(),0);
		sensor_msgs::PointCloud2 cloudmsg;
		pcl::toROSMsg(it->second->cloud, cloudmsg);
		if(it->first > 0)
			cloudmsg.header.frame_id = "child_frame";
		else
			cloudmsg.header.frame_id = "parent_frame";
		lidar_publisher[it->first].publish(cloudmsg);
	}


// 	  lidar.postprocess();
// 	  const Cloud& totalcloud= lidar.gettotalcloud();

// //	  LOG(INFO)<<"totalcloud.size="<<totalcloud.size();
// 	  sensor_msgs::PointCloud2 cloudmsg;
// 	  pcl::toROSMsg(totalcloud, cloudmsg);
// 	  LOG(INFO)<<" time:"<<(ros::Time::now() - ros::Time(cloudmsg.header.stamp)).toSec();
// 	  if(fixframe)
// 		  cloudmsg.header.frame_id = "global_init_frame";
// 	  else
// 		  cloudmsg.header.frame_id = "vehicle_frame";
// 	  pubLaserCloud.publish(cloudmsg);

// 	  if(pubOriginCloud.getNumSubscribers()>0)
// 	  {
// 		  pcl::toROSMsg(lidar.gettotalorigincloud(), cloudmsg);
// 		  LOG(INFO)<<" time:"<<(ros::Time::now() - ros::Time(cloudmsg.header.stamp)).toSec();
// 		  if(fixframe)
// 			  cloudmsg.header.frame_id = "global_init_frame";
// 		  else
// 			  cloudmsg.header.frame_id = "vehicle_frame";
// 		  pubOriginCloud.publish(cloudmsg);
// 	  }
// //	  auto pointcloudmsgptr = lidar.getcloudmsg();
// //	  pubPointCloudMultilidar.publish(pointcloudmsgptr);
// //	  pointcloudmsgptr->
// 	  LOG(INFO)<<"pub pointcloud:"<<totalcloud.size();
    }

  return 1;

}

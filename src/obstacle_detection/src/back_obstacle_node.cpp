
#include <cmath>
#include "transform/rigid_transform.h"
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
#include <ros/package.h>
#include <fstream>
#include <list>
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "obstacle_detection.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "obstacle_msgs/ObstacleOGM.h"
#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906
/*!
 * \brief 后置雷达目标检测
 */
class BackLidarProcess
{
public:
	typedef std::pair<double,ivcommon::transform::Rigid3d> TimePosePair;
	/*!
	 *\brief 构造函数，初始化一部分参数
	 */
	BackLidarProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle)
	,processthread_(NULL)
	,negativethread_(NULL)
	,processthreadfinished_ (false)
	{
		init();
	}
	/*!
	 * \brief 析构函数，关掉线程
	 */
	~BackLidarProcess()
	{
	  lidarOdoms_.stopQueue();
	  processthreadfinished_ = true;
	  processthread_->join();
	  negativethread_->join();
	}
	/*!
	 *\brief 变量及对象初始化
	 */
	void init()
	{
		display = false;
		ros::param::get("~N_SCANS",N_SCANS_);
		ros::param::get("~heightdiff_threshold_higher",heightdiffthreshold_higher);
		if(N_SCANS_ == 64)
			hdlCalibration = ros::package::getPath("sensor_driver") + "/config/64S3db.xml";//这里我设置成了绝对
		else if (N_SCANS_ == 32)
			hdlCalibration = "";

		hdlgrabber=new pcl::VelodyneGrabber(hdlCalibration, pcapFile, N_SCANS_);
		obstacle_detection = new Obstacle_Detection(*hdlgrabber);
//		lidarprocess=new LidarProcess(*hdlgrabber,replay,display,velodyneheight,rigid_heightdiffthreshold,xmlconfig,calibvalue);
		ros::param::get("~heightdiff_threshold",heightdiffthreshold_);
		pubBackOGM_ = nodehandle_.advertise<obstacle_msgs::ObstacleOGM> ("backOgm", 5);

//		subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
//										 ("lidar_odometry_to_init", 5, boost::bind(&BackLidarProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉

		subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
										 ("lidar_cloud_back_calibrated", 1, boost::bind(&BackLidarProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云
//		subCheck = nodehandle_.subscribe<sensor_driver_msgs::NegativeOGM>("negativeOgm", 5 ,boost::bind(&BackLidarProcess::ogmChecker,this,_1));

		processthread_ = new boost::thread(boost::bind(&BackLidarProcess::process,this));
		//negativethread_ = new boost::thread(boost::bind(&BackLidarProcess::pointCloudDisplay,this));
		//file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);

	}
	/*!
	 * \brief 利用udp发送数据（在RCS时代使用）
	 * \param ogmdata 需要发送的栅格地图
	 */
	void SendData(OGMData<unsigned char>& ogmdata)  //udp通信 发送端例程
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
	/*!
	 * \brief 处理雷达里程计发来的消息
	 * \param laserOdometry 里程计消息
	 */
	void laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)  //雷达里程计
	{
	  double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
	  static double last_stamp = -1;
	//  static geometry_msgs::Quaternion last_geoQuat;
	  static ::ivcommon::transform::Rigid3d lasttransformodometry;
	//  static float last_trans[6];
	//  double roll, pitch, yaw;
	  geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

	  Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
	  Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
			  laserOdometry->odometry.pose.pose.position.y,
			  laserOdometry->odometry.pose.pose.position.z);

	  ::ivcommon::transform::Rigid3d transformodometry(translation,roatation);

	  lidarOdoms_.Push(::ivcommon::make_unique<TimePosePair>(timeOdometry,transformodometry));

	}
	/*!
	 * \brief 点云显示函数（可根据点云中点的passibility设置颜色）
	 */
	void pointCloudDisplay()
	{
			boost::shared_ptr<PCLVisualizer> cloud_viewer_negative(new PCLVisualizer ("Obstacle Cloud"));
			cloud_viewer_negative->addCoordinateSystem (3.0);
			cloud_viewer_negative->setBackgroundColor (0, 0, 0);
			cloud_viewer_negative->initCameraParameters ();
			cloud_viewer_negative->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
			cloud_viewer_negative->setCameraClipDistances (0.0, 100.0);
			Cloud::Ptr passableCloud(new Cloud);//
			Cloud::Ptr negativeCloud(new Cloud);//
			passableCloud->clear();
			negativeCloud->clear();
			char linename[20];
			float x1,x2,y1,y2,z;
		    pcl::PointXYZI pt1, pt2, pt3, pt4;
			for(int i = 0 ; i < 10 ; i++)
			{
				x1 = -20 ;
				x2 = 20 ;
				y1 = (i - 4) * 5 ;
				y2 = (i - 4) * 5;
				z = 0;
				pt1.x = min(x1 , x2) ;
				pt1.y = min(y1 , y2) ;
				pt1.z = z;
				pt2.x = max(x1 , x2) ;
				pt2.y = max(y1 , y2) ;
				pt2.z = z;
				memset(linename, 0 , 20);
				sprintf(linename , "lat%02d" , i);
				cloud_viewer_negative->addLine(pt1, pt2, linename);
			}

			for(int i = 0 ; i < 5 ; i++)
			{
				x1 = i * 10 - 20;
				x2 = i * 10 - 20;
				y1 = -20 ;
				y2 = 70 ;
				z = 0;
			    pt1.x = min(x1 , x2) ;
			    pt1.y = min(y1 , y2) ;
			    pt1.z = z;
			    pt2.x = max(x1 , x2) ;
			    pt2.y = max(y1 , y2) ;
			    pt2.z = z;
			    memset(linename, 0 , 20);
			    sprintf(linename , "lng%02d" , i);
			    cloud_viewer_negative->addLine(pt1, pt2, linename);
			}

			while(!cloud_viewer_negative->wasStopped())
		{
			cloud_mutex_.lock();
			for(int i = 0;i < totalCloud.points.size();i++)
			{
				if(totalCloud.points[i].passibility < 0.1 || totalCloud.points[i].passibility >3.1)
					negativeCloud->points.push_back(totalCloud.points[i]);
				else
					passableCloud->points.push_back(totalCloud.points[i]);
			}
			cloud_mutex_.unlock();
				cloud_viewer_negative->removeAllPointClouds();

				if(passableCloud->points.size()>0)
				{
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> passableCloudHandler (passableCloud, 0, 255, 0);
					if (!cloud_viewer_negative->updatePointCloud (passableCloud, passableCloudHandler, "passableCloud"))
					{
						cloud_viewer_negative->addPointCloud (passableCloud, passableCloudHandler, "passableCloud");
					}
				}
				if(negativeCloud->points.size()>0)
				{
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> negativeCloudHandler (negativeCloud, 255, 0, 255);
					if (!cloud_viewer_negative->updatePointCloud (negativeCloud, negativeCloudHandler, "negativeCloud"))
					{
						cloud_viewer_negative->addPointCloud (negativeCloud, negativeCloudHandler, "negativeCloud");
					}
				}

				cloud_viewer_negative->spinOnce();
				boost::this_thread::sleep (boost::posix_time::microseconds(100));
				passableCloud->clear();
				negativeCloud->clear();
			}
			cloud_viewer_negative->close();
	}
	/*!
	 * \brief 接收点云数据的程序
	 * \param laserCloudFullRes2 点云消息
	 */
	void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) //点云数据
	{
		double timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
		LOG(INFO)<<std::fixed<<std::setprecision(3)<<"cloudtime:"<<timeLaserCloudFullRes;
		LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
	  lidarCloudMsgs_.Push(laserCloudFullRes2);
	  if(lidarCloudMsgs_.Size()>1)
		  lidarCloudMsgs_.Pop();

	}
	/*!
	 * \brief 对打包好的点云进行拆分（可能一个包里有多帧点云，需要将其拆分）
	 * \param inputcloud 输入点云
	 * \param outputclouds 输出点云
	 * \param lidarpropertys 点云的特性以及雷达点的位置
	 */
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
	/*!
	 * \brief 处理函数
	 */
	void process()
	{

		while(!processthreadfinished_)
		{
			const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.1));
			if(cloudmsg == nullptr)
				continue;
			pcl::PointCloud<pcl::PointXYZI> pointcloud_back;

			pcl::PointCloud<pcl::PointXYZI> pointcloud_back_out;

			pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）

			pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;

			OGMData<unsigned char> ogm_data_height_back(40,40,0.2,40,20);


			std::vector<pcl::PointXYZI> lidarpropertys;
			pcl::PointXYZI point;
			analysisCloud(tempcloud,outputclouds,lidarpropertys);
			LOG(INFO)<<"cloud num:"<<lidarpropertys.size();
			point = lidarpropertys.at(0);

			pointcloud_back = *outputclouds[0];


			obstacle_detection->heightdiffOgmDetection(pointcloud_back,pointcloud_back_out,ogm_data_height_back,0.2,heightdiffthreshold_,1,REFINERIGIDNOPASSABLE);
			obstacle_detection->heightdiffOgmDetection(pointcloud_back,pointcloud_back_out,ogm_data_height_back,0.2,heightdiffthreshold_higher,1,RIGIDNOPASSABLE);


			cloud_mutex_.lock();
			totalCloud = pointcloud_back_out;
			cloud_mutex_.unlock();

//			Obstacle_Detection::cloud2OGM(pointcloud_back_out,ogm_data_height_back,1);
			obstacle_msgs::ObstacleOGM obstacleOGM;
			obstacleOGM.header.stamp = ros::Time(cloudmsg->header.stamp);
			obstacleOGM.ogmheight = ogm_data_height_back.ogmheight_cell;
			obstacleOGM.ogmwidth = ogm_data_height_back.ogmwidth_cell;
			obstacleOGM.ogmresolution = ogm_data_height_back.ogmresolution;
			obstacleOGM.vehicle_x = ogm_data_height_back.vehicle_x / ogm_data_height_back.ogmresolution;
			obstacleOGM.vehicle_y = ogm_data_height_back.vehicle_y / ogm_data_height_back.ogmresolution;

			for(int i = 0 ;i < ogm_data_height_back.ogmcell_size;i++)
			{
				obstacleOGM.data.push_back(ogm_data_height_back.ogm[i]);
			}
			pubBackOGM_.publish(obstacleOGM);
//			Obstacle_Detection::showOGM("total_obstacle_frame",ogm_data_height_back);
	/*******************************************display*************************************************/
			cv::Mat Display_Image = cv::Mat::zeros(
					cv::Size(ogm_data_height_back.ogmheight_cell, ogm_data_height_back.ogmwidth_cell), CV_8UC3);

			//  cv::imshow("out",out);
			int data_index = 0;
			for (int j = 0; j < Display_Image.rows; j++) {

				unsigned char* display_data = (unsigned char*) Display_Image.ptr<
						uchar>(j);
				for (int i = 0; i < Display_Image.cols; i++) {
					uchar pixel;

					if (ogm_data_height_back.ogm[data_index] == RIGIDNOPASSABLE)
					{

						display_data[3 * i] = 255;
						display_data[3 * i + 1] = 255;
						display_data[3 * i + 2] = 255;
					}
					else if (ogm_data_height_back.ogm[data_index] == REFINERIGIDNOPASSABLE)
					{
						display_data[3 * i] = 0;
						display_data[3 * i + 1] = 0;
						display_data[3 * i + 2] = 255;
					}
					else if (ogm_data_height_back.ogm[data_index] == UNKNOWN)
					{

						display_data[3 * i] = 0;
						display_data[3 * i + 1] = 0;
						display_data[3 * i + 2] = 0;
					}
					else if (ogm_data_height_back.ogm[data_index] == PASSABLE)
					{

						display_data[3 * i] = 0;
						display_data[3 * i + 1] = 255;
						display_data[3 * i + 2] = 0;
					}
					data_index++;
					//if(*pdata == 254)

				}
			}
			int linestep = 10 / ogm_data_height_back.ogmresolution;
			int heightnum = Display_Image.rows / (linestep);
			int widthnum = Display_Image.cols / (linestep);
			for (int i = 0; i < heightnum; i++) {
				cv::line(Display_Image, cv::Point(0, linestep * i),
						cv::Point(Display_Image.cols - 1, linestep * i),
						cv::Scalar(255, 0, 0));
			}

			for (int i = 1; i < widthnum; i++) {
				cv::line(Display_Image, cv::Point(linestep * i, 0),
						cv::Point(linestep * i, Display_Image.rows - 1),
						cv::Scalar(255, 0, 0));
			}
			cv::Point vehicle_pos(ogm_data_height_back.vehicle_x/ogm_data_height_back.ogmresolution
					,ogm_data_height_back.vehicle_y/ogm_data_height_back.ogmresolution);
			cv::circle(Display_Image,vehicle_pos,
					5, cv::Scalar(255, 0, 0), -1);
			cv::flip(Display_Image, Display_Image,0);
			if(1)
			{
				cv::imshow("back_ogm", Display_Image);
				cvWaitKey(2);
			}

//			lidarOdoms_.Pop(); //需要雷达里程计信息时需要，否则可以注释掉

		}

	}


	struct ObjectState
        {
			

        };


	
	ros::Subscriber subLaserOdometry_ ;/*< 里程计接收*/
	ros::Publisher pubBackOGM_;/*< 后置雷达正障碍栅格发布*/


	ros::Subscriber subLaserCloudFullRes_ ;/*< 经过筛选且转换之后的点云*/
	ros::Subscriber subCheck;
	double heightdiffthreshold_,heightdiffthreshold_higher;/*< 两个不同的高度差阈值*/
	::ivcommon::BlockingQueue<std::unique_ptr<TimePosePair>> lidarOdoms_;/*< 里程计位姿*/
	::ivcommon::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;/*< 点云消息*/
//	std::fstream file_;
	boost::mutex cloud_mutex_;/*< 点云赋值时需要用到的互斥锁*/
	boost::thread* processthread_;/*< 处理的总进程*/
	boost::thread* negativethread_;/*< 负障碍显示的进程*/
	ros::NodeHandle& nodehandle_;/*< ros节点*/
	bool processthreadfinished_;/*< 处理结束的标志位*/
	//negative detection
	Obstacle_Detection* obstacle_detection = NULL;/*< 障碍物检测的对象*/
	pcl::VelodyneGrabber* hdlgrabber;/*< 内部含有点云排布的类的对象*/
	bool display;/*< 显示开关*/
	int N_SCANS_;/*< 雷达线束*/
	std::string hdlCalibration, pcapFile, srcip_H;/*< 初始化hdlgrabber的参数*/
	pcl::PointCloud<pcl::PointXYZI> totalCloud;/*< 所有点云*/
};

/*!
 * \brief main函数
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "back_obstacle_node");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  BackLidarProcess backlidarprocess(nh);
  ros::spin();


  return 0;
}

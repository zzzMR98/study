#include <cmath>
#include <memory>
#include "transform/rigid_transform.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
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

#include "obstacle_detection.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "util/xmlconf/xmlconf.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "common/lua_parameter_dictionary.h"
#include "common/configuration_file_resolver.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "obstacle_msgs/ObstacleOGM.h"
#include "lanelet_map_msgs/Way.h"
#include "sensor_driver_msgs/startconfig.h"
#include "object_msgs/ObjectStates.h"
#include "signal_msgs/SmogEnvironment.h"
#include <time.h>

#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906
/*!
 * \brief 前置雷达正负障碍检测
 */
class ObstacleDetectionProcess
{
public:
	typedef std::pair<double,ivcommon::transform::Rigid3d> TimePosePair;
	/*!
	 *\brief 构造函数，初始化一部分参数
	 */
	ObstacleDetectionProcess(const std::string &configstr, ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
	configstr_(configstr),
	// ,processthread_(NULL)
	negativethread_(NULL),
	processthreadfinished_ (false),
	smog_on(false)
	{
		init();
	}
	/*!
	 * \brief 析构函数，关掉线程
	 */
	~ObstacleDetectionProcess()
	{
		subLaserCloudFullRes_.shutdown();
		processthreadfinished_ = true;
		keep_visualize_thread_running = false;
		lidarOdoms_.stopQueue();
		lidarCloudMsgs_.stopQueue();
        radarObjectsMsgs_.stopQueue();
		if (processthread_.joinable())
			processthread_.join();
		if (display_on_ && visualize_thread!=nullptr && visualize_thread->joinable()) {
			visualize_thread->join();
		}
		// if(display_on_)
		// 	negativethread_->join();
	}
	/*!
	*\brief 变量及对象初始化
	*/
	void init()
	{
		ros::NodeHandle pnh("~");
		pnh.param<int>("N_SCANS", N_SCANS_, 32);
		pnh.param<bool>("display_on", display_on_, false);
		pnh.param<bool>("enable_unknown_area", enable_unknown_area, false);
		pnh.param<double>("heightdiff_threshold_higher", heightdiffthreshold_higher, 0.5); //较大的高度差阈值
		pnh.param<double>("heightdiff_threshold", heightdiffthreshold_, 0.3); //较小的高度差阈值
		if(N_SCANS_ == 64)
			hdlCalibration = ros::package::getPath("sensor_driver") + "/config/64S3db.xml";//这里我设置成了绝对  激光雷达配置文件路径zmr
		else if (N_SCANS_ == 32)
			hdlCalibration = "";
		hdlgrabber=new pcl::VelodyneGrabber(hdlCalibration, pcapFile, N_SCANS_); //这个pcapFile是怎么传过来的
		obstacle_detection = new Obstacle_Detection(*hdlgrabber);
		//		lidarprocess=new LidarProcess(*hdlgrabber,replay,display_on_,velodyneheight,rigid_heightdiffthreshold,xmlconfig,calibvalue);

		pnh.param<bool>("is_floor_flat", is_floor_flat, false);
		pnh.param<float>("lower_height_bound",lower_height_bound,0.1);
		pnh.param<float>("hyper_height_bound",hyper_height_bound,2.5);
		pnh.param<double>("outer_angle_start", outer_angle_start, 0);
		pnh.param<double>("outer_angle_end", outer_angle_end, 360);
		pnh.param<double>("outer_unknown_radius", outer_unknown_radius, 4.);
		pnh.param<double>("inner_angle_start", inner_angle_start, 0);
		pnh.param<double>("inner_angle_end", inner_angle_end, 0);
		pnh.param<double>("inner_unknown_radius", inner_unknown_radius, 4.);
		pnh.param<double>("grid_resolution", grid_resolution, 0.2);
		pnh.param<int>("grid_map_rows", grid_map_rows, 70);
		pnh.param<int>("grid_map_cols", grid_map_cols, 40);
		pnh.param<int>("vehicle_grid_x", vehicle_grid_x, 20);
		pnh.param<int>("vehicle_grid_y", vehicle_grid_y, 20);
		// 获取车身点云坐标范围
		pnh.getParam("body_x_range", body_x_range);
		pnh.getParam("body_y_range", body_y_range);
		pnh.getParam("body_z_range", body_z_range);
		if (!xmlconfig_.Parse(configstr_.c_str(), "iv_lidar")) //c_str()函数返回一个指向正规C字符串的指针常量, 内容与本string串相同。 zmr
		{
			std::cout << "iv_lidar  is not exist in config xml  file" << std::endl;
		}
		else
		{
			//加载雷达数量和标定文件 zmr
			int lidarnum_;
			string calibdirname_;
			if (!xmlconfig_.GetModuleParam("lidarnum", lidarnum_))
			{
				std::cout << "lidarnum is incorrect" << std::endl;
			}
			// else
			// 	std::cout << "lidarnum is " << lidarnum_ << std::endl;

			if (!xmlconfig_.GetModuleParam("calibdir", calibdirname_))
			{
				std::cout << "calibdirname is incorrect" << std::endl;
			}
			// else
			// 	std::cout << "calibdirname is " << calibdirname_ << std::endl;
			std::string pkgPath = ros::package::getPath("sensor_driver");
			std::string calibdirpath = pkgPath + "/config/extrinsicparameter/" + calibdirname_;
			std::stringstream sstr;
			sstr << 0 << ".lua";
			auto file_resolver = ::ivcommon::make_unique<::ivcommon::ConfigurationFileResolver>(
				std::vector<string>{calibdirpath});
			const string lua_code = file_resolver->GetFileContentOrDie(sstr.str());
			::ivcommon::LuaParameterDictionary lua_parameter_dictionary(
				lua_code, std::move(file_resolver));
			x_offset = lua_parameter_dictionary.GetDouble("x_offset");
			// std::cout << "x_offset: " << x_offset << std::endl;
			y_offset = lua_parameter_dictionary.GetDouble("y_offset");
			// std::cout << "y_offset: " << y_offset << std::endl;

		}
		pubNegativeOGM_ = nodehandle_.advertise<obstacle_msgs::ObstacleOGM> ("negativeOgm", 5); //创建发布者对象 zmr
		// filtered_pointcloud_pub = nodehandle_.advertise<sensor_msgs::PointCloud2> ("filtered_pointcloud", 1);

		//		subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
		//										 ("lidar_odometry_to_init", 5, boost::bind(&ObstacleDetectionProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉

		subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2> //创建订阅者对象 zmr
		("lidar_cloud_calibrated", 1, boost::bind(&ObstacleDetectionProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云

		subObstacleCheck = nodehandle_.subscribe<lanelet_map_msgs::Way>("topology_global_path",1,
				boost::bind(&ObstacleDetectionProcess::negativeDetectionCheck,this,_1));

		subRadarObjects = nodehandle_.subscribe<object_msgs::ObjectStates>("smog_radar_objects",1,
				boost::bind(&ObstacleDetectionProcess::RadarObjectHandler,this,_1)); //zmr增加

        subSmogState = nodehandle_.subscribe<signal_msgs::SmogEnvironment>("smog_state_for_mapping",1,
                boost::bind(&ObstacleDetectionProcess::SmogStateHandler,this,_1)); //zmr增加
                
		processthread_ = boost::thread(boost::bind(&ObstacleDetectionProcess::process,this));
		if (display_on_) {
			keep_visualize_thread_running = true;
			visualize_thread = std::make_shared<boost::thread>(
				boost::thread(boost::bind(&ObstacleDetectionProcess::visualizeObstacleGridmap,this)));
		}
		else {
			keep_visualize_thread_running = false;
			visualize_thread = nullptr;
		}
		// if(display_on_)
		// 	negativethread_ = new boost::thread(boost::bind(&ObstacleDetectionProcess::pointCloudDisplay,this));
		//file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);

	}

	void visualizeObstacleGridmap() {
		while (keep_visualize_thread_running) {
			if (!display_image.empty()) {
				cv::namedWindow("obstacle_img", 0);
				cv::imshow("obstacle_img", display_image);
				cv::waitKey(1);
			}
			usleep(80000);
		}
	}


    /*!
    * zmr增加
    * \brief 处理规划发来的是否为烟雾环境的消息
    * \param smogSignal 是否为烟雾环境的消息
    */
    void SmogStateHandler(const signal_msgs::SmogEnvironment::ConstPtr& smogSignal)
    {
		smogSignalMsgs_.Push(smogSignal);//队列末端添加，但队列满时会阻塞
        if(smogSignalMsgs_.Size()>1)
            smogSignalMsgs_.Pop();// \brief 弹出最前端数据，会阻塞等待队列非空或请求结束的信号
	    
    }


    /*!
    * zmr增加
    * \brief 处理毫米波雷达发来的障碍物消息
    * \param radarObjects 毫米波雷达发来的消息
    */
    void RadarObjectHandler(const object_msgs::ObjectStates::ConstPtr& radarObjects)
    {

        double timeRadar = radarObjects->header.stamp.toSec();
        LOG(INFO)<<std::fixed<<std::setprecision(3)<<"radartime:"<<timeRadar;
        LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeRadar;
        bool radar_detection_status = radarObjects->detection_status;

        radarObjectsMsgs_.Push(radarObjects);//队列末端添加，但队列满时会阻塞
        if(radarObjectsMsgs_.Size()>1)
            radarObjectsMsgs_.Pop();// \brief 弹出最前端数据，会阻塞等待队列非空或请求结束的信号


    }

	/*!
	* \brief 根据全局规划判断是否需要打开负障碍检测
	* \param totalCheck 全局规划发来的消息
	*/
	void negativeDetectionCheck(const lanelet_map_msgs::Way::ConstPtr& totalCheck)
	{
		negative_on_ = totalCheck->open_concave_obs_det;
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
			pt1.x = min(x1 , x2) ; //-20
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
			for(int i = 0;i < negativeDetectionCloud.points.size();i++)
			{
				if(negativeDetectionCloud.points[i].passibility >3.1)
					negativeCloud->points.push_back(negativeDetectionCloud.points[i]);
				else
					passableCloud->points.push_back(negativeDetectionCloud.points[i]);
			}
			cloud_mutex_.unlock();
			cloud_viewer_negative->removeAllPointClouds();

			//点云可视化流 zmr
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
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> negativeCloudHandler (negativeCloud, 255, 0, 0);
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
		lidarCloudMsgs_.Push(laserCloudFullRes2);//队列末端添加，但队列满时会阻塞
		if(lidarCloudMsgs_.Size()>1)
			lidarCloudMsgs_.Pop();// \brief 弹出最前端数据，会阻塞等待队列非空或请求结束的信号
	}
    /*!
    * \brief 基于 ray-cast 方法从毫米波雷达障碍物栅格图中提取已知区域
    * \param srcimage 输入毫米波障碍物栅格图
    * \param center 车辆位置
    * \param radius 提取已知区域圆的半径
    */
    void known_area_extraction_for_radar(cv::Mat& srcimage, cv::Point center, int radius)
    {
        const int kgap_num_threshold = 5;
        cv::Mat dilateimg;
        cv::Mat elementlarge = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(2,2));
        cv::Mat elementsmall = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(1,1));
        cv::dilate(srcimage,dilateimg,elementlarge); //膨胀操作

        cv::Mat knownimg = cv::Mat::zeros(cv::Size(radius*2+1,radius*2+1), srcimage.type());
        cv::Point knownimg_center(radius,radius);
        cv::Point radar_center(center.x , center.y + 1.5/grid_resolution);//1.5为y_offset
        std::vector<cv::Point> circle_points;

        cv::ellipse2Poly(radar_center,cv::Size(radius,radius),0,0,180,1,circle_points);
        for(auto it=circle_points.begin();it!=circle_points.end();it++)
        {
            cv::LineIterator lit(dilateimg,radar_center,*it);
            cv::LineIterator litknown(knownimg,knownimg_center,*it - radar_center + knownimg_center);
            int i=0;
            auto& srcdata = *(*lit);
            // while (srcdata == UNKNOWN) {
            // 	i++; lit++; litknown++;
            // 	srcdata = *(*lit);
            // }
            bool last_known = true;
            int gap_num = 0;
            for(;i<lit.count;i++,litknown++,lit++)
            {
                srcdata = *(*lit);
                auto& data = *(*litknown);
                if(last_known)
                {
                    if(srcdata == RIGIDNOPASSABLE )
                    {
                        last_known = false;
                        gap_num = 0;
                    }
                    else data = PASSABLE;
                }
                else
                {
                    if(gap_num >= kgap_num_threshold && srcdata==PASSABLE)
                    {
                        last_known = true;
                        gap_num = 0;
                    }
                    else if(srcdata==RIGIDNOPASSABLE )
                        gap_num = 0;
                    else
                    {
                        gap_num++;
                        data = UNKNOWN;
                    }
                }

            }
        }

        cv::dilate(knownimg,knownimg,elementsmall);
        for(int j=0; j<knownimg.rows; j++)
        {
            int src_j = j - knownimg_center.y + radar_center.y;
            if(src_j<0 || src_j>=srcimage.rows)
                continue;
            unsigned char* pdata = knownimg.ptr<unsigned char>(j);
            unsigned char* srcdata = srcimage.ptr<unsigned char>(src_j);
            for(int i=0; i<knownimg.cols; i++)
            {
                int src_i = i - knownimg_center.x + radar_center.x;
                if(src_i<0 || src_i>=srcimage.cols)
                    continue;

                srcdata[src_i] = std::max(srcdata[src_i],pdata[i]);
            }
        }
    }

	/*!
	* \brief 基于 ray-cast 方法从高度差障碍物栅格图中提取已知区域
	* \param srcimage 输入高度差障碍物栅格图
	* \param center 车辆位置
	* \param radius 提取已知区域圆的半径
	*/
	void known_area_extraction(cv::Mat& srcimage, cv::Point center, int radius) 
	{
		const int kgap_num_threshold = 5;
		cv::Mat dilateimg;
		cv::Mat elementlarge = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(2,2));
		cv::Mat elementsmall = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(1,1));
		cv::dilate(srcimage,dilateimg,elementlarge); //膨胀操作

		cv::Mat knownimg = cv::Mat::zeros(cv::Size(radius*2+1,radius*2+1), srcimage.type());
		cv::Point knownimg_center(radius,radius);
		cv::Point lidar_center(center.x + x_offset/grid_resolution, center.y + y_offset/grid_resolution);
		std::vector<cv::Point> circle_points;
		// cv::ellipse2Poly(knownimg_center,cv::Size(radius,radius),0,0,360,1,circle_points);
		cv::ellipse2Poly(lidar_center,cv::Size(radius,radius),0,outer_angle_start,outer_angle_end,1,circle_points);
		for(auto it=circle_points.begin();it!=circle_points.end();it++)
		{
			cv::LineIterator lit(dilateimg,lidar_center,*it);
			cv::LineIterator litknown(knownimg,knownimg_center,*it - lidar_center + knownimg_center);
			int i=0;
			auto& srcdata = *(*lit);
			// while (srcdata == UNKNOWN) {
			// 	i++; lit++; litknown++;
			// 	srcdata = *(*lit);
			// }
			bool last_known = true, firt_time = true;
			int gap_num = 0;
			for(;i<lit.count;i++,litknown++,lit++)
			{
				srcdata = *(*lit);
				auto& data = *(*litknown);
				if(last_known)
				{
					if(srcdata == RIGIDNOPASSABLE || srcdata == REFINERIGIDNOPASSABLE)
					{
						last_known = false;
						gap_num = 0;
						firt_time = false;
					}
					else if (srcdata == PASSABLE) {
						data = PASSABLE;
						firt_time = false;
					}
					else if (firt_time) {
						data = UNKNOWN;
						// firt_time = false;
					}
					else data = PASSABLE;
				}
				else
				{
					if(gap_num >= kgap_num_threshold && srcdata==PASSABLE)
					{
						last_known = true;
						gap_num = 0;
					}
					else if(srcdata==RIGIDNOPASSABLE || srcdata == REFINERIGIDNOPASSABLE)
						gap_num = 0;
					else
					{
						gap_num++;
						data = UNKNOWN;
					}
				}

			}
		}

		cv::dilate(knownimg,knownimg,elementsmall);
		for(int j=0; j<knownimg.rows; j++)
		{
			int src_j = j - knownimg_center.y + lidar_center.y;
			if(src_j<0 || src_j>=srcimage.rows)
				continue;
			unsigned char* pdata = knownimg.ptr<unsigned char>(j);
			unsigned char* srcdata = srcimage.ptr<unsigned char>(src_j);
			for(int i=0; i<knownimg.cols; i++)
			{
				int src_i = i - knownimg_center.x + lidar_center.x;
				if(src_i<0 || src_i>=srcimage.cols)
					continue;

				srcdata[src_i] = std::max(srcdata[src_i],pdata[i]);
			}
		}
		if (!enable_unknown_area) return;
		std::vector<cv::Point> circle_points2, circle_points3;
		cv::ellipse2Poly(lidar_center,
			cv::Size(outer_unknown_radius/grid_resolution, outer_unknown_radius/grid_resolution),
			0,max(-90.0,outer_angle_start-1),inner_angle_start,1,circle_points); // cv::ellipse2Poly 的 起始角度和终止角度是相对像素坐标系（原点在左上角）来说的
		cv::ellipse2Poly(lidar_center,
			cv::Size(outer_unknown_radius/grid_resolution, outer_unknown_radius/grid_resolution),
			0,inner_angle_end,min(270.0,outer_angle_end+1),1,circle_points2);
		cv::ellipse2Poly(lidar_center,
			cv::Size(inner_unknown_radius/grid_resolution, inner_unknown_radius/grid_resolution),
			0,inner_angle_start,inner_angle_end,1,circle_points3);
		for(auto it=circle_points.begin(); it!=circle_points.end(); it++)
		{
			cv::LineIterator lit(srcimage, lidar_center, *it);
			auto& srcdata = *(*lit);
			for(int i=0; i<lit.count; i++,lit++)
			{
				srcdata = *(*lit);
				// *(*lit) = RIGIDNOPASSABLE;
				if (!(srcdata == RIGIDNOPASSABLE || srcdata == REFINERIGIDNOPASSABLE))
					*(*lit) = UNKNOWN;
			}
		}
		for(auto it=circle_points2.begin(); it!=circle_points2.end(); it++)
		{
			cv::LineIterator lit(srcimage, lidar_center, *it);
			auto& srcdata = *(*lit);
			for(int i=0; i<lit.count; i++,lit++)
			{
				srcdata = *(*lit);
				// *(*lit) = RIGIDNOPASSABLE;
				if (!(srcdata == RIGIDNOPASSABLE || srcdata == REFINERIGIDNOPASSABLE))
					*(*lit) = UNKNOWN;
			}
		}
		for(auto it=circle_points3.begin(); it!=circle_points3.end(); it++)
		{
			cv::LineIterator lit(srcimage, lidar_center, *it);
			auto& srcdata = *(*lit);
			for(int i=0; i<lit.count; i++,lit++)
			{
				srcdata = *(*lit);
				// *(*lit) = RIGIDNOPASSABLE;
				if (!(srcdata == RIGIDNOPASSABLE || srcdata == REFINERIGIDNOPASSABLE))
					*(*lit) = UNKNOWN;
			}
		}
		// cv::erode(srcimage,srcimage,elementlarge);
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
            OGMData<unsigned char> ogm_data_total(70,40,grid_resolution,20,20);
            cv::Mat src_image;
            cv::Point vehicle_pos(ogm_data_total.vehicle_x/ogm_data_total.ogmresolution
                    ,ogm_data_total.vehicle_y/ogm_data_total.ogmresolution);
            const object_msgs::ObjectStatesConstPtr smogsignalmsg = smogSignalMsgs_.PopWithTimeout(::ivcommon::FromSeconds(1)); //弹出最前端数据，但队列空时会阻塞，等待timeout时间后返回nullptr
            if(smogsignalmsg == nullptr)
                continue;
			smog_on = smogsignalmsg->smogstate;
			
			if(smog_on == true)
	        {
                const object_msgs::ObjectStatesConstPtr radarmsg = radarObjectsMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.2)); //弹出最前端数据，但队列空时会阻塞，等待timeout时间后返回nullptr
                if(radarmsg == nullptr)
                    continue;
                for(const auto objects : radarmsg->object)
                {
                    radar_pose.push_back(objects.pose);
                }

                OGMData<unsigned char> ogm_data_radar_initial(70,40,grid_resolution,20,20);
                obstacle_detection->obstacleDetectionForMMWRadar(radar_pose,ogm_data_radar_initial);

                cv::Mat radar_initial_mat(ogm_data_radar_initial.ogmheight_cell,ogm_data_radar_initial.ogmwidth_cell,CV_8U);

                int data_index=0;
			    for (int j = 0; j < radar_initial_mat.rows; j++) {
				    int index_j = j;
				    uchar* testdata = radar_initial_mat.ptr<uchar>(index_j);
				    for (int i = 0; i < radar_initial_mat.cols; i++) {
				    	testdata[i] = ogm_data_radar_initial.ogm[data_index];
				    	data_index++;
				    }
			    }

			    src_image = radar_initial_mat.clone();
			    known_area_extraction_for_radar(radar_initial_mat, vehicle_pos, 30/ogm_data_radar_initial.ogmresolution);

			    data_index = 0;
			    for (int j = 0; j < radar_initial_mat.rows; j++)
			    {
			    	int index_j = j;
			    	uchar* testdata = radar_initial_mat.ptr<uchar>(index_j);
			    	for (int i = 0; i < radar_initial_mat.cols; i++)
			    	{
                        ogm_data_radar_initial.ogm[data_index] = testdata[i];
			    		data_index++;
			    	}
			    }

                for(int i = 0; i < ogm_data_total.ogmcell_size;i++)
                {
                    if(ogm_data_radar_initial.ogm[i] == RIGIDNOPASSABLE)
                    {
                        ogm_data_total.ogm[i] = RIGIDNOPASSABLE;
                    }
                    else if(ogm_data_radar_initial.ogm[i] == PASSABLE )
                    {
                        ogm_data_total.ogm[i] = PASSABLE;
                    }
                    else
                    {
                        ogm_data_total.ogm[i] = UNKNOWN;
                    }
                }
                /*******************************************display*************************************************/
                // if(display_on_ && (outputclouds.size() > 1))
                // 	Obstacle_Detection::showOGM("negative", ogm_data_negative);
                negativeOGM.data.clear();
                negativeOGM.header.stamp = ros::Time(radarmsg->header.stamp);
                negativeOGM.ogmheight = ogm_data_total.ogmheight_cell;
                negativeOGM.ogmwidth = ogm_data_total.ogmwidth_cell;
                negativeOGM.ogmresolution = ogm_data_total.ogmresolution;
                negativeOGM.vehicle_x = ogm_data_total.vehicle_x / ogm_data_total.ogmresolution;
                negativeOGM.vehicle_y = ogm_data_total.vehicle_y / ogm_data_total.ogmresolution;

                for(int i = 0 ;i < ogm_data_total.ogmcell_size;i++)
                {
                    negativeOGM.data.push_back(ogm_data_total.ogm[i]);
                }
                pubNegativeOGM_.publish(negativeOGM);
                // sensor_msgs::PointCloud2 filtered_pointcloud_msg;
                // pcl::toROSMsg(*outputclouds[0], filtered_pointcloud_msg);
                // filtered_pointcloud_msg.header = cloudmsg->header;
                // filtered_pointcloud_pub.publish(filtered_pointcloud_msg);
            }


	        else
	        {
			// clock_t start_time = clock();
			const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(::ivcommon::FromSeconds(0.1)); //弹出最前端数据，但队列空时会阻塞，等待timeout时间后返回nullptr
			if(cloudmsg == nullptr)
				continue;
			pcl::PointCloud<pcl::PointXYZI> negativeCloud_Left;
			pcl::PointCloud<pcl::PointXYZI> negativeCloud_Right;
			std::vector<pcl::PointCloud<pcl::PointXYZI>> heightDiffClouds;

			pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系） 什么时候转换的? ros发布的不是在车体坐标系下吗? zmr?

			pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
			OGMData<unsigned char> ogm_data_negative_left(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_data_negative_right(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_data_negative(70,40,grid_resolution,20,20);

			OGMData<unsigned char> ogm_data_height_left(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_data_height_right(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_known_left(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_known_right(70,40,grid_resolution,20,20);
			OGMData<unsigned char> ogm_data_height_total(70,40,grid_resolution,20,20);

			std::vector<pcl::PointXYZI> lidarpropertys;
			pcl::PointXYZI point;
			analysisCloud(tempcloud,outputclouds,lidarpropertys); //outputclouds 多帧点云 lidarpropertys每帧点云的第一个点组成的向量 zmr
			// 移除车身点云及无效点云
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			pcl::ExtractIndices<pcl::PointXYZI> extract; //点云索引
			if (body_x_range.size() == 2 && body_y_range.size() == 2) {
				for (int i = 0; i < (*outputclouds[0]).size(); i++)
				{
					float& x = outputclouds[0]->points[i].x;
					float& y = outputclouds[0]->points[i].y;
					float& z = outputclouds[0]->points[i].z;
					if ((body_x_range[0] <= x && x <= body_x_range[1] &&
						body_y_range[0] <= y && y <= body_y_range[1]))
					{
						inliers->indices.push_back(i);
					}
				}
			}
			for (int i = 0; i < (*outputclouds[0]).size(); i++)
			{
				float& x = outputclouds[0]->points[i].x;
				float& y = outputclouds[0]->points[i].y;
				// float& z = outputclouds[0]->points[i].z;
				if ((x*x + y*y) > 10000) //zmr?
				{
					inliers->indices.push_back(i);
				}
			}
			extract.setInputCloud(outputclouds[0]);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*outputclouds[0]);

			if(outputclouds.size() > 1) {
				if (lidarpropertys[1].x < -0.1)
				{
					negativeCloud_Left = *outputclouds[1];
				}
				else negativeCloud_Right = *outputclouds[1];
			}
			if(outputclouds.size() > 2) {
				if (lidarpropertys[2].x > 0.1)
				{
					negativeCloud_Right = *outputclouds[2];
				}
				else negativeCloud_Left = *outputclouds[2];
			}
			obstacle_detection->negativeDetectionForLeftRslidar(negativeCloud_Left);
			obstacle_detection->negativeDetectionForRightRslidar(negativeCloud_Right);
			cloud_mutex_.lock();
			negativeDetectionCloud = negativeCloud_Left + negativeCloud_Right;
			cloud_mutex_.unlock();
			int count = 0;
			double average_x = 0, average_y = 0, average_range = 0;
			for(int i = 0; i < negativeCloud_Left.size();i++)
            {
                if(negativeCloud_Left.points[i].passibility > 3.1)
                {
                    count += 1;
                    average_x += negativeCloud_Left.points[i].x;
                    average_y += negativeCloud_Left.points[i].y;
                    average_range += negativeCloud_Left.points[i].intensity;
                }
            }
            if(count != 0)
            {
                average_x = average_x / count;
                average_y = average_y / count;
                average_range = average_range / count;
                //std::cout<<"average_x = "<<average_x<<std::endl;
                //std::cout<<"average_y = "<<average_y<<std::endl;
               // std::cout<<"average_range = "<<average_range<<std::endl;
            }

			Obstacle_Detection::cloud2OGMForRsLidar(negativeCloud_Left,ogm_data_negative_left,1);
			Obstacle_Detection::cloud2OGMForRsLidar(negativeCloud_Right,ogm_data_negative_right,1);
			for(int i=0;i< outputclouds.size();i++)
			{
				heightDiffClouds.push_back(Cloud());
				Obstacle_Detection::purecloud2OGM(*outputclouds[i], ogm_known_left, 2);
				if (is_floor_flat) {
					obstacle_detection->obstacleDetectionForFlatFloor(*outputclouds[i],ogm_data_height_left,grid_resolution,lower_height_bound,hyper_height_bound,heightdiffthreshold_,REFINERIGIDNOPASSABLE, heightdiffthreshold_higher, RIGIDNOPASSABLE);
				}
				else {
					obstacle_detection->heightdiffOgmDetection(*outputclouds[i],heightDiffClouds[i],ogm_data_height_left,grid_resolution,heightdiffthreshold_,1,REFINERIGIDNOPASSABLE);
					obstacle_detection->heightdiffOgmDetection(*outputclouds[i],heightDiffClouds[i],ogm_data_height_left,grid_resolution,heightdiffthreshold_higher,1,RIGIDNOPASSABLE);
				}
			}
			// if(N_SCANS_ == 32)
			// {
			// 	if(negative_on_)
			// 		obstacle_detection->negativeDetection32(negativeCloud,N_SCANS_);
			// }
			// else if(N_SCANS_ == 64)
			// {
			// 	if(negative_on_)
			// 		obstacle_detection->negativeDetection64(negativeCloud);
			// }
			for(int i = 0; i < ogm_data_height_total.ogmcell_size; i ++)
			{
				ogm_data_height_total.ogm[i] = ogm_data_height_left.ogm[i];
				// if(ogm_data_height_left.ogm[i] == RIGIDNOPASSABLE)
				// 	ogm_data_height_total.ogm[i] = RIGIDNOPASSABLE;
				// else if(ogm_data_height_left.ogm[i] == REFINERIGIDNOPASSABLE)
				// 	ogm_data_height_total.ogm[i] = REFINERIGIDNOPASSABLE;
				// else if(ogm_data_height_left.ogm[i] == PASSABLE || ogm_known_left.ogm[i] == PASSABLE)
				// 	ogm_data_height_total.ogm[i] = PASSABLE;
				// if (ogm_data_height_left.ogm[i] == UNKNOWN && ogm_known_left.ogm[i] == PASSABLE)
				// 	ogm_data_height_total.ogm[i] = PASSABLE;
			}

			cv::Mat height_total_mat(ogm_data_height_total.ogmheight_cell,ogm_data_height_total.ogmwidth_cell,CV_8U);
			//zmr?
			int data_index=0;
			for (int j = 0; j < height_total_mat.rows; j++) {
				int index_j = j;
				uchar* testdata = height_total_mat.ptr<uchar>(index_j);
				for (int i = 0; i < height_total_mat.cols; i++) {
					testdata[i] = ogm_data_height_total.ogm[data_index];
					data_index++;
				}
			}
			src_image = height_total_mat.clone();
			known_area_extraction(height_total_mat, vehicle_pos, 30/ogm_data_height_total.ogmresolution);

			data_index = 0;
			for (int j = 0; j < height_total_mat.rows; j++) {
				int index_j = j;
				uchar* testdata = height_total_mat.ptr<uchar>(index_j);
				for (int i = 0; i < height_total_mat.cols; i++) {
					ogm_data_height_total.ogm[data_index] = testdata[i];
					data_index++;
				}
			}
			for(int i = 0; i < ogm_data_negative.ogmcell_size;i++)
			{
				if(ogm_data_negative_left.ogm[i] == NEGATIVENOPASSABLE || ogm_data_negative_right.ogm[i] == NEGATIVENOPASSABLE)
					ogm_data_negative.ogm[i] = NEGATIVENOPASSABLE;
				else
					ogm_data_negative.ogm[i] = ogm_data_negative_left.ogm[i] || ogm_data_negative_right.ogm[i];
			}

			for(int i = 0; i < ogm_data_total.ogmcell_size;i++)
			{
				if(ogm_data_height_total.ogm[i] == RIGIDNOPASSABLE)
				{
					ogm_data_total.ogm[i] = RIGIDNOPASSABLE;
				}
				else if(ogm_data_negative.ogm[i] == NEGATIVENOPASSABLE)
				{
					ogm_data_total.ogm[i] = NEGATIVENOPASSABLE;
				}
				else if(ogm_data_height_total.ogm[i] == PASSABLE )
				{
					ogm_data_total.ogm[i] = PASSABLE;
				}
				else if(ogm_data_height_total.ogm[i] == REFINERIGIDNOPASSABLE )
				{
					ogm_data_total.ogm[i] = REFINERIGIDNOPASSABLE;
				}
				else
				{
					ogm_data_total.ogm[i] = UNKNOWN;
				}
			}

			/*******************************************display*************************************************/
			// if(display_on_ && (outputclouds.size() > 1))
			// 	Obstacle_Detection::showOGM("negative", ogm_data_negative);
			negativeOGM.data.clear();
			negativeOGM.header.stamp = ros::Time(cloudmsg->header.stamp);
			negativeOGM.ogmheight = ogm_data_total.ogmheight_cell;
			negativeOGM.ogmwidth = ogm_data_total.ogmwidth_cell;
			negativeOGM.ogmresolution = ogm_data_total.ogmresolution;
			negativeOGM.vehicle_x = ogm_data_total.vehicle_x / ogm_data_total.ogmresolution;
			negativeOGM.vehicle_y = ogm_data_total.vehicle_y / ogm_data_total.ogmresolution;

			for(int i = 0 ;i < ogm_data_total.ogmcell_size;i++)
			{
				negativeOGM.data.push_back(ogm_data_total.ogm[i]);
			}
			pubNegativeOGM_.publish(negativeOGM);
			// sensor_msgs::PointCloud2 filtered_pointcloud_msg;
			// pcl::toROSMsg(*outputclouds[0], filtered_pointcloud_msg);
			// filtered_pointcloud_msg.header = cloudmsg->header;
			// filtered_pointcloud_pub.publish(filtered_pointcloud_msg);
            }

			/*******************************************display*************************************************/
			if(display_on_)
			{
				display_image = cv::Mat::zeros(
						cv::Size(ogm_data_total.ogmheight_cell,ogm_data_total.ogmwidth_cell), CV_8UC3);
				int data_index = 0;
				for (int j = 0; j < display_image.rows; j++) {
					unsigned char* srcdata = src_image.ptr<unsigned char>(j);
					unsigned char* display_data = (unsigned char*) display_image.ptr<uchar>(j);
					for (int i = 0; i < display_image.cols; i++) {
						if (ogm_data_total.ogm[data_index] == NEGATIVENOPASSABLE)//BGR 洋红色
						{
							display_data[3 * i] = 255;
							display_data[3 * i + 1] = 0;
							display_data[3 * i + 2] = 255;
						}
						else if (ogm_data_total.ogm[data_index] == RIGIDNOPASSABLE)//黑色
						{
							display_data[3 * i] = 255;
							display_data[3 * i + 1] = 255;
							display_data[3 * i + 2] = 255;
						}
						else if (ogm_data_total.ogm[data_index] == REFINERIGIDNOPASSABLE)//红
						{
							display_data[3 * i] = 0;
							display_data[3 * i + 1] = 0;
							display_data[3 * i + 2] = 255;
						}
						else if(srcdata[i] == PASSABLE)
						{
							display_data[3 * i] = 100;
							display_data[3 * i + 1] = 100;
							display_data[3 * i + 2] = 100;
						}
						else if (ogm_data_total.ogm[data_index] == UNKNOWN)//白色
						{
							display_data[3 * i] = 0;
							display_data[3 * i + 1] = 0;
							display_data[3 * i + 2] = 0;
						}
						else if (ogm_data_total.ogm[data_index] == PASSABLE)//绿色
						{
							display_data[3 * i] = 0;
							display_data[3 * i + 1] = 255;
							display_data[3 * i + 2] = 0;
						}
						data_index++;
					}
				}
				int linestep = 10 / ogm_data_total.ogmresolution;
				int heightnum = display_image.rows / (linestep);
				int widthnum = display_image.cols / (linestep);
				for (int i = 0; i < heightnum; i++) {
					cv::line(display_image, cv::Point(0, linestep * i),
							cv::Point(display_image.cols - 1, linestep * i),
							cv::Scalar(255, 0, 0));
				}

				for (int i = 1; i < widthnum; i++) {
					cv::line(display_image, cv::Point(linestep * i, 0),
							cv::Point(linestep * i, display_image.rows - 1),
							cv::Scalar(255, 0, 0));
				}
				cv::circle(display_image,vehicle_pos,
						5, cv::Scalar(255, 0, 0), -1);
				cv::flip(display_image, display_image,0);

				// cv::namedWindow("known_img", 0);
				// cv::imshow("known_img", display_image);
				// cv::waitKey(1);
			}
			//LOG(INFO)<<"obstacle total cost time::"<<(ros::Time::now() - cloudmsg->header.stamp).toSec();
			// lidarOdoms_.Pop(); //需要雷达里程计信息时需要，否则可以注释掉
			// clock_t end_time = clock();
			// ROS_INFO("obstacle_detect process time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
		}
	}



    ros::Subscriber subLaserOdometry_ ;/*< 里程计接收*/
	ros::Publisher pubNegativeOGM_;/*< 正负障碍栅格发布*/
	obstacle_msgs::ObstacleOGM negativeOGM;/*< 正负障碍消息*/

	ros::Subscriber subRadarObjects;/*< 毫米波雷达消息接受*/
    ros::Subscriber subSmogState;/*< 烟雾环境消息接受*/
	ros::Subscriber subLaserCloudFullRes_ ;/*< 经过筛选且转换之后的点云*/
	ros::Subscriber subObstacleCheck;/*< 全局规划消息接受*/
	double heightdiffthreshold_,heightdiffthreshold_higher;/*< 两个不同的高度差阈值*/
	::ivcommon::BlockingQueue<std::unique_ptr<TimePosePair>> lidarOdoms_;/*< 里程计位姿*/
	::ivcommon::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;/*< 点云消息*/
    ::ivcommon::BlockingQueue<object_msgs::ObjectStatesConstPtr> radarObjectsMsgs_;/*< 毫米波雷达消息*/
	::ivcommon::BlockingQueue<signal_msgs::SmogEnvironmentConstPtr> smogSignalMsgs_;/*< 毫米波雷达消息*/
	 
	//	std::fstream file_;
	boost::mutex cloud_mutex_;/*< 点云赋值时需要用到的互斥锁*/
	pcl::PointCloud<pcl::PointXYZI> negativeDetectionCloud;/*< 负障碍检测得到的点云*/
	boost::thread processthread_;/*< 处理的总进程*/
	boost::thread* negativethread_;/*< 负障碍显示的进程*/
	ros::NodeHandle& nodehandle_;/*< ros节点*/
	bool processthreadfinished_;/*< 处理结束的标志位*/
	//negative detection
	Obstacle_Detection* obstacle_detection = NULL;/*< 障碍物检测的对象*/
	pcl::VelodyneGrabber* hdlgrabber;/*< 内部含有点云排布的类的对象*/
	bool display_on_;/*< 显示开关*/
	bool negative_on_;/*< 负障碍检测开关*/
	int N_SCANS_;/*< 雷达线束*/
	std::string hdlCalibration, pcapFile, srcip_H;/*< 初始化hdlgrabber的参数*/
	int grid_map_cols, grid_map_rows, vehicle_grid_x, vehicle_grid_y;
	vector<float> body_x_range, body_y_range, body_z_range; // 车身点云坐标范围
	// ros::Publisher filtered_pointcloud_pub;
	double grid_resolution;
	XmlConf xmlconfig_;/**< xml解析类 */
	std::string configstr_;/**< xml配置信息 */
	float x_offset, y_offset; // 激光雷达相对车体坐标系原点偏移量
	bool enable_unknown_area;
	double outer_angle_start, outer_angle_end, outer_unknown_radius;
	double inner_angle_start, inner_angle_end, inner_unknown_radius;
	bool is_floor_flat; // 如果地面是平面，则用lower_height_bound, hyper_height_bound来判断障碍物
	float lower_height_bound, hyper_height_bound, known_radius;
	std::shared_ptr<boost::thread> visualize_thread;
	bool keep_visualize_thread_running;
	cv::Mat display_image;

    std::vector<geometry_msgs::Pose> radar_pose;/*< 毫米波雷达发来的障碍物位置*/
    bool smog_on;/*< 烟雾环境开关*/
};

/*!
 * \brief main函数不多说了
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_detection_node");
	ros::NodeHandle nh;

	google::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
	FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
	ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
	sensor_driver_msgs::startconfig configsrv;

	while (!configclient.call(configsrv))
	{
		ros::Duration(0.01).sleep();
	}

	std::string startconfig = configsrv.response.configstr;
	ObstacleDetectionProcess obstacledetectionprocess(startconfig, nh);
	ros::spin();
	return 0;
}

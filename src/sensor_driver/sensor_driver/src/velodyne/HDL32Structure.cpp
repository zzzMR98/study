#include "HDL32Structure.h"

#include<boost/thread/thread.hpp>
#include<boost/bind.hpp>
#include<boost/thread/mutex.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "util/xmlconf/xmlconf.h"
#include "mytime.h"
#include "data_types.hpp"
//#define USE_OMP
#ifdef USE_OMP
#include <omp.h>
static omp_lock_t omplock;
#endif

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

using namespace boost;

boost::mutex displaymutex;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
        std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
        count = 0; \
        last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
    do \
{ \
}while(false)
#endif

#define doNothing()
#define doSomething()


//#define GRASS_DETECT
#define OPEN_POLAR_DETECT       0
#define OPEN_VACANT_DETECT      0
#define OPEN_WATER_DETECT      0
//

#define HDL_MIN(a,b)  (fabs(a)<fabs(b)?a:b)
#define HDL_MAX(a,b)  (fabs(a)>fabs(b)?a:b)

LidarProcess::~LidarProcess()
{
    if(isinited)
    {
        flag_close=true;
#ifdef USE_OMP
        omp_destroy_lock(&omplock);
#endif

        fs_boundary.close();
        delete [] laneminintensity_ogm;
        delete [] lanemaxintensity_ogm;

        delete [] maxintensity_ogm;
        delete [] minintensity_ogm;
//        delete [] cloudsections ;
        cvReleaseImage(&intensityogm_img);
        if(thread_displayPointCloud)
        {
            thread_displayPointCloud->join();  //timed_join(boost::posix_time::seconds(1));
            delete thread_displayPointCloud;
            thread_displayPointCloud = NULL;
        }
        cloud_connection_H.disconnect ();

        std::cout<<"~hdlviewer"<<std::endl;
    }
}

void LidarProcess::cloud_callback_H (const CloudConstPtr& cloud)
{
    FPS_CALC ("cloud callback_H");
    boost::mutex::scoped_lock lock (cloud_mutex_H_);
    cloud_H_ = cloud;
}

void LidarProcess::cloud_callback (const CloudConstPtr& cloud, float startAngle, float endAngle)
{
    FPS_CALC ("cloud callback_H");

    boost::mutex::scoped_lock lock (cloud_mutex_H_);
    cloud_H_ = cloud;
}

void LidarProcess::keyboard_callback (const KeyboardEvent& event, void* cookie)
{
    if (event.keyUp ())
    {
        if(event.getKeyCode() ==  '1')
            freeze_ = 1;
        else if(event.getKeyCode() ==  '2')
            freeze_ = 0;
        return;
    }
}

void LidarProcess::mouse_callback (const MouseEvent& mouse_event, void* cookie)
{
    if (mouse_event.getType () == MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == MouseEvent::LeftButton)
    {
        cout << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
}


void LidarProcess::coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz)
{
    newz = z;
    newx = x;
    newy = y;
}

void LidarProcess::setpointcloud (const CloudConstPtr& cloud)
{
    boost::mutex::scoped_lock lock (cloud_mutex_H_);
    cloud_H_ = cloud;
}

const Cloud& LidarProcess::getpointcloudaftercompute ()
{
  return *velodyne_pointcloud;
}

void LidarProcess::initHDL()
{
    if(isinited)
        return;
    isinited=true;

    boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cloud_cb_H;
    if(fpointcloud_cb_!=NULL)
      cloud_cb_H=*fpointcloud_cb_;
    else
     cloud_cb_H = boost::bind(&LidarProcess::cloud_callback_H, this, _1);
    cloud_connection_H = grabber_H_.registerCallback(cloud_cb_H);


    grabber_H_.start ();

//    cloudsections = new CloudPointSections[LASER_LAYER] ;

    cloudsections.resize(LASER_LAYER);
    //坐标转换矩阵

    //使用角度来制作旋转平移矩阵
    /*double alfa_h = 1.689058940988057*M_PI/180;
      double beta_h = 0.093133558156532*M_PI/180;
      double gama_h = -179.14838024769*M_PI/180;*/
//  17 3月tuoli数据
//    alfa_h = 0.0*M_PI/180;
//    beta_h = 0.0*M_PI/180;
//    gama_h = 2*M_PI/180;//-178
//
//    x_offset_h = 0;
//    y_offset_h = 0 ;//0.8;
//    z_offset_h = 1.9;

    //tuoli bisai hdl32
//    alfa_h = -1.5129*M_PI/180;
//    beta_h =-1.1719*M_PI/180;
//    gama_h = -135*M_PI/180;
//    x_offset_h = 0.0;
//    y_offset_h = 0.5;
//    z_offset_h = 2.3;

    alfa_h = calibvalue_.alfa*M_PI/180;
    beta_h = calibvalue_.beta*M_PI/180;
    gama_h = calibvalue_.gama*M_PI/180;//-178

    x_offset_h = calibvalue_.x_offset;
    y_offset_h = calibvalue_.y_offset ;//0.8;
    z_offset_h = calibvalue_.z_offset;
    transform_matrix_calibration_H2car_angle = Eigen::Matrix4f::Identity();

    transform_matrix_calibration_H2car_angle(0,0) = cos(beta_h)*cos(gama_h);
    transform_matrix_calibration_H2car_angle(0,1) = sin(alfa_h)*sin(beta_h)*cos(gama_h) - cos(alfa_h)*sin(gama_h);
    transform_matrix_calibration_H2car_angle(0,2) = cos(alfa_h)*sin(beta_h)*cos(gama_h) + sin(alfa_h)*sin(gama_h);
    transform_matrix_calibration_H2car_angle(0,3) = x_offset_h;
    transform_matrix_calibration_H2car_angle(1,0) = cos(beta_h)*sin(gama_h);
    transform_matrix_calibration_H2car_angle(1,1) = sin(alfa_h)*sin(beta_h)*sin(gama_h) + cos(alfa_h)*cos(gama_h);
    transform_matrix_calibration_H2car_angle(1,2) = cos(alfa_h)*sin(beta_h)*sin(gama_h) - sin(alfa_h)*cos(gama_h);
    transform_matrix_calibration_H2car_angle(1,3) = y_offset_h;
    transform_matrix_calibration_H2car_angle(2,0) = -sin(beta_h);
    transform_matrix_calibration_H2car_angle(2,1) = sin(alfa_h)*cos(beta_h);
    transform_matrix_calibration_H2car_angle(2,2) = cos(alfa_h)*cos(beta_h);
    transform_matrix_calibration_H2car_angle(2,3) = z_offset_h;
    transform_matrix_calibration_H2car_angle(3,0) = 0;
    transform_matrix_calibration_H2car_angle(3,1) = 0;
    transform_matrix_calibration_H2car_angle(3,2) = 0;
    transform_matrix_calibration_H2car_angle(3,3) = 1.0;


    laneminintensity_ogm = new float[laneogm.ogmcell_size];
    lanemaxintensity_ogm = new float[laneogm.ogmcell_size];



    maxintensity_ogm = new unsigned char[intensityogm.ogmcell_size];
    minintensity_ogm = new unsigned char[intensityogm.ogmcell_size];
    intensityogm_img = cvCreateImage(cvSize(intensityogm.ogmwidth_cell,intensityogm.ogmheight_cell),8,1);

    //任务1：确定车行人等障碍物
    //任务2：确定马路牙子等道路边界

    ogm_y_offset = 20.0f;//向y轴负方向移动了20
    farogm_y_offset = 20.0f - refineogm.ogmheight;//向y轴负方向移动了20 - ogmheightm
    laneogm_y_offset = 10.0f;//向y轴负方向移动了10m




    velodyne_pointcloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    passablecloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//
    rigid_nopassablecloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//
    positiveslopecloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//
    negativeslopecloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//
    boundarycloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//
    roadorientationcloud = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);//

    if(display_)
    {
        boost::function0<void> fdisplay = boost::bind(&LidarProcess::displayPointCloud,this);
        thread_displayPointCloud=new boost::thread(fdisplay);
    }
#ifdef USE_OMP
    omp_init_lock(&omplock);
#endif

    flag_ogmdetection=false;
    flag_diffdetection=false;
    flag_circletangential=false;
    flag_circleradius=false;

    if(!xmlconfig_.GetModuleParam("flag_ogmdetection",flag_ogmdetection))
    {
        std::cerr<<"no flag_ogmdetection!"<<std::endl;
        return ;
    };

    if(!xmlconfig_.GetModuleParam("flag_diffdetection",flag_diffdetection))
    {
        std::cerr<<"no flag_diffdetection!"<<std::endl;
        return ;
    };

    if(!xmlconfig_.GetModuleParam("flag_circletangential",flag_circletangential))
    {
        std::cerr<<"no flag_circletangential!"<<std::endl;
        return ;
    };

    if(!xmlconfig_.GetModuleParam("flag_circleradius",flag_circleradius))
    {
        std::cerr<<"no flag_circleradius!"<<std::endl;
        return ;
    };

    if(!xmlconfig_.GetModuleParam("layernum",LASER_LAYER))
    {
        std::cerr<<"no layernum!"<<std::endl;
        return ;
    };

    /*
#if LASER_LAYER == 64
float laserverticalangle[LASER_LAYER] = {
-30.67, -9.3299999, -29.33, -8, -28, -6.6700001, -26.67, -5.3299999,
-25.33, -4, -24, -2.6700001, -22.67, -1.33, -21.33, 0,
-20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
-14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
#elif LASER_LAYER == 32
float laserverticalangle[LASER_LAYER] = {
-30.67, -9.3299999, -29.33, -8, -28, -6.6700001, -26.67, -5.3299999,
-25.33, -4, -24, -2.6700001, -22.67, -1.33, -21.33, 0,
-20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
-14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
#elif LASER_LAYER == 16
float laserverticalangle[LASER_LAYER] = {
-15, 1, -13, 3, -11, 5, -9, 7,
-7, 9, -5, 11, -3, 13, -1, 15 };
#endif
     */
    cout<<"LASER_LAYER="<<LASER_LAYER<<endl;
    for(int i=0;i<LASER_LAYER;i++)
    {
        theorydis[i]=-z_offset_h/std::tan(grabber_H_.indexmaptable[i].angle*M_PI/180);
        if(theorydis[i]>0&&theorydis[i]<60)
            anglerange[i]=2.5/theorydis[i]*180/M_PI;
        cout<<"theorydis["<<i<<"]="<< theorydis[i]<<endl;
    }



}

void LidarProcess::displayPointCloud()
{
    boost::shared_ptr<PCLVisualizer> cloud_viewer_(new PCLVisualizer ("HDL Cloud"));

    cloud_viewer_->addCoordinateSystem (3.0);
    cloud_viewer_->setBackgroundColor (0, 0, 0);
    cloud_viewer_->initCameraParameters ();
    cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    cloud_viewer_->setCameraClipDistances (0.0, 100.0);
    cloud_viewer_->registerKeyboardCallback (&LidarProcess::keyboard_callback, *this);
    {
        //画车身
        float x1 = -1 , x2 = 1 , y1 = -1 , y2 = 3, z = 0;
        float newx1, newx2, newy1, newy2, newz;
        coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
        coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
        pcl::PointXYZI pt1, pt2, pt3, pt4;
        pt1.x = newx1 ;
        pt1.y = newy1 ;
        pt1.z = newz;
        pt2.x = newx1 ;
        pt2.y = newy2 ;
        pt2.z = newz;
        cloud_viewer_->addLine(pt1, pt2, "body1");

        pt1.x = newx2 ;
        pt1.y = newy2 ;
        pt1.z = newz;
        cloud_viewer_->addLine(pt1, pt2, "body2");

        pt2.x = newx2 ;
        pt2.y = newy1 ;
        pt2.z = newz;
        cloud_viewer_->addLine(pt1, pt2, "body3");

        pt1.x = newx1 ;
        pt1.y = newy1 ;
        pt1.z = newz;
        cloud_viewer_->addLine(pt1, pt2, "body4");



        //画上范围
        if(0)
        {
            float x1 = -20 , x2 = 20 , y1 = -1 , y2 = 40, z = Z_MAX;
            float newx1, newx2, newy1, newy2, newz;
            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
            pcl::PointXYZI pt1, pt2, pt3, pt4;
            pt1.x = newx1 ;
            pt1.y = newy1 ;
            pt1.z = newz;
            pt2.x = newx1 ;
            pt2.y = newy2 ;
            pt2.z = newz;
            cloud_viewer_->addLine(pt1, pt2, "upper1");

            pt1.x = newx2 ;
            pt1.y = newy2 ;
            pt1.z = newz;
            cloud_viewer_->addLine(pt1, pt2, "upper2");

            pt2.x = newx2 ;
            pt2.y = newy1 ;
            pt2.z = newz;
            cloud_viewer_->addLine(pt1, pt2, "upper3");

            pt1.x = newx1 ;
            pt1.y = newy1 ;
            pt1.z = newz;
            cloud_viewer_->addLine(pt1, pt2, "upper4");
        }

        //画网格线
        char linename[20];
        for(int i = 0 ; i < 10 ; i++)
        {
            x1 = -20 ;
            x2 = 20 ;
            y1 = (i - 2) * 10 ;
            y2 = (i - 2) * 10;
            z = 0;
            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
            pt1.x = min(newx1 , newx2) ;
            pt1.y = min(newy1 , newy2) ;
            pt1.z = newz;
            pt2.x = max(newx1 , newx2) ;
            pt2.y = max(newy1 , newy2) ;
            pt2.z = newz;
            memset(linename, 0 , 20);
            sprintf(linename , "lat%02d" , i);
            cloud_viewer_->addLine(pt1, pt2, linename);
        }

        for(int i = 0 ; i < 5 ; i++)
        {
            x1 = i * 10 - 20;
            x2 = i * 10 - 20;
            y1 = -20 ;
            y2 = 70 ;
            z = 0;
            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
            pt1.x = min(newx1 , newx2) ;
            pt1.y = min(newy1 , newy2) ;
            pt1.z = newz;
            pt2.x = max(newx1 , newx2) ;
            pt2.y = max(newy1 , newy2) ;
            pt2.z = newz;
            memset(linename, 0 , 20);
            sprintf(linename , "lng%02d" , i);
            cloud_viewer_->addLine(pt1, pt2, linename);
        }
    }

    while (!cloud_viewer_->wasStopped ()&&!flag_close)
    {
        if(display_ == 1)
        {
            if(freeze_==1&&replay_==1)
                cloud_viewer_->spinOnce();
            else
            {
                //display
                // if(velodyne_pointcloud->points.size()>0)
                if(pointcloud_updateflag)
                {

                    pointcloud_updateflag=false;
                    boost::mutex::scoped_lock lock(displaymutex);

                    cloud_viewer_->removeAllPointClouds();
                    //��ͨ������
                    if(passablecloud->points.size()>0)
                    {
                        //��ɫ

                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> passablecloudHandler (passablecloud, 0, 255, 0);
                        if (!cloud_viewer_->updatePointCloud (passablecloud, passablecloudHandler, "passablecloud")){
                            cloud_viewer_->addPointCloud (passablecloud, passablecloudHandler, "passablecloud");
                        }
                    }

                    if(rigid_nopassablecloud->points.size()>0)
                    {
                        //��
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rigid_nopassablecloudHandler (rigid_nopassablecloud, 255, 0, 0);
                        if (!cloud_viewer_->updatePointCloud (rigid_nopassablecloud, rigid_nopassablecloudHandler, "rigid_nopassablecloud")){
                            cloud_viewer_->addPointCloud (rigid_nopassablecloud, rigid_nopassablecloudHandler, "rigid_nopassablecloud");
                        }
                    }

                    //positive cloud
                    if(positiveslopecloud->points.size()>0)
                    {
                        //��
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> positiveslopecloudHandler (positiveslopecloud, 0, 0, 255);
                        if (!cloud_viewer_->updatePointCloud (positiveslopecloud, positiveslopecloudHandler, "positiveslopecloud")){
                            cloud_viewer_->addPointCloud (positiveslopecloud, positiveslopecloudHandler, "positiveslopecloud");
                        }
                    }

                    //negative cloud
                    if(negativeslopecloud->points.size()>0)
                    {
                        //��
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> negativeslopecloudHandler (negativeslopecloud, 0, 0, 122);
                        if (!cloud_viewer_->updatePointCloud (negativeslopecloud, negativeslopecloudHandler, "negativeslopecloud")){
                            cloud_viewer_->addPointCloud (negativeslopecloud, negativeslopecloudHandler, "negativeslopecloud");

                        }
                    }


                    //boundary cloud
                    if(boundarycloud->points.size()>0)
                    {
                        //��
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> boundarycloudHandler (boundarycloud, 255, 255, 255);
                        if (!cloud_viewer_->updatePointCloud (boundarycloud, boundarycloudHandler, "boundarycloud")){
                            cloud_viewer_->addPointCloud (boundarycloud, boundarycloudHandler, "boundarycloud");
                            cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "boundarycloud");
                        }
                    }

                    //roadorientation cloud
                    if(roadorientationcloud->points.size()>0)
                    {
                        //��
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> roadorientationcloudHandler (roadorientationcloud, 255, 255, 0);
                        if (!cloud_viewer_->updatePointCloud (roadorientationcloud, roadorientationcloudHandler, "roadorientationcloud")){
                            cloud_viewer_->addPointCloud (roadorientationcloud, roadorientationcloudHandler, "roadorientationcloud");
                            cloud_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "roadorientationcloud");
                        }
                    }

                    /*
                    //悬崖
                    if(stiff_pointcloud->points.size()>0)
                    {
                    //灰
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cliffcloudHandler (stiff_pointcloud, 150, 150, 150);
                    if (!cloud_viewer_->updatePointCloud (stiff_pointcloud, cliffcloudHandler, "cloud_vehicle_hdl_cliff")){
                    cloud_viewer_->addPointCloud (stiff_pointcloud, cliffcloudHandler, "cloud_vehicle_hdl_cliff");
                    }
                    }
                     */
                    //       if(velodyne_pointcloud)
                    cloud_viewer_->spinOnce();



                }
                if (!grabber_H_.isRunning())
                    cloud_viewer_->spin ();


                ////��ʾ********************/

#ifdef SIMULATION
                boost::this_thread::sleep (boost::posix_time::microseconds (50000));
#else
                boost::this_thread::sleep (boost::posix_time::microseconds (100));
#endif
            }
        }
    }
    //    if (replay_ == 1)
    //        grabber_H_.resume();

    flag_close=true;
    cloud_viewer_->close();
}
void LidarProcess::border_detection(const Cloud& pointcloud,Cloud& borderpointcloud,
		OGMData<unsigned char>& ogm_data,int LASER_LAYER,double x_offset,double y_offset)
{
	//路沿检测函数
	//对于点云的初始化，将其全视作可通行区域
	//雷达参数
	int col_count = pointcloud.size() / LASER_LAYER;

	//路沿检测参数
	float delta_angle_thresh = 40.0;//adjustable param 路沿检测的角度阈值
	int cloud_window_size = 10;//adjustable param 相邻的帧数
	if(1)
	{
		for(int i = 0 ; i < LASER_LAYER ; i++)
		{
			//这里是将每一线的角度转换成弧度，加上beta是对垂直角度偏移量的一个修正，根据雷达安装的情况

			for(int j = 0 ; j < col_count - cloud_window_size; j++)
			{
				//第i线雷达的数据
				int index = j * LASER_LAYER + i;

				if(pointcloud.points[index].range < 0.5)
					continue;

				//这个方位角是正切，是对于其切线的斜率的求取（好像没有什么用）

				int index1 = (j + cloud_window_size) * LASER_LAYER + i;

				if(pointcloud.points[index1].range < 0.5)
					continue;
				//第一个点的切线角度expected_tangential_angle
				float slope_angle = atan2(pointcloud.points[index].y-y_offset, pointcloud.points[index].x-x_offset) * 180 / M_PI;
				float expected_tangential_angle = slope_angle - 90;
				//������+-pi
				if(expected_tangential_angle > 180) expected_tangential_angle -= 360;
				else if(expected_tangential_angle <= -180) expected_tangential_angle += 360;
				//第十帧的点与第一帧的点连线的斜率
				float actual_tangential_angle = atan2(pointcloud.points[index1].y - pointcloud.points[index].y,
						pointcloud.points[index1].x - pointcloud.points[index].x) * 180 / M_PI;

				//这里就是用前后大约是10帧的点进行判断，用第一个点的切线与1点与10点之间连线的斜率进行比较，如果差距比较大，则判断是路沿
				//实际上路沿就是这种检测效果，实际点云当检测到路沿的时候就凹陷下去一块，之后就又弯回来，然后就又正常，因此比较这个角度应该可以判断ZhuBC
				float delta_angle = actual_tangential_angle - expected_tangential_angle;
				if(delta_angle > 180) delta_angle -= 360;
				else if(delta_angle < -180) delta_angle += 360;

				if(delta_angle>90)
					delta_angle = 180 - delta_angle;
				//如果大于阈值，则认为是路沿
				if(fabs(delta_angle) > delta_angle_thresh)
				{
					borderpointcloud.push_back(pointcloud.points[index]);
					borderpointcloud.back().passibility = 0.;
//					pointcloud.points[index].passibility = 0.0;
				}

			}

		}
	}

	//生成栅格地图并将passibility=0的点放在栅格里面
	if(1)
	{
	  memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
	  unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	  unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	  memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	  memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	  float ogm_y_offset = 20.0f;//对于y的偏移量，车体坐标系转成栅格地图的变化

	  for (int i = 0; i < borderpointcloud.points.size(); i++)
	  {
	      float x = borderpointcloud.points[i].x,
			      y = borderpointcloud.points[i].y,
			      z = borderpointcloud.points[i].z;

	      float newy = y + ogm_y_offset;//栅格地图中的实际y值
	      //判断是否在栅格地图内部
	      if((x >=-ogm_data.ogmwidth/2 &&x <= ogm_data.ogmwidth/2 ) && (newy >=0 && newy < ogm_data.ogmheight) && (z >= - 1.5 && z <=  Z_MAX))
	      {
		      int col = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;//横向栅格
		      int row = boost::math::round(newy / ogm_data.ogmresolution) ;//纵向栅格

		      if((row >=0 && row < ogm_data.ogmheight_cell) && (col >=0 && col < ogm_data.ogmwidth_cell))
		      {
				  int index = row * ogm_data.ogmwidth_cell + col;
				  //如果为之前检测到的路沿，则计数器加1
				  if(borderpointcloud.points[i].passibility < 0.5)
					  nopassible_point_count_ogm_count[index]++;
				  else
					  passible_point_count_ogm_count[index]++;
		      }
	      }
	  }
	  int nopassible_count_thresh = 8;//同一个栅格内的5个点都是那种状态才满足。
	  for(int i = 0 ; i < ogm_data.ogmcell_size ; i++)
	  {
		  if(nopassible_point_count_ogm_count[i] > nopassible_count_thresh)
		  {
			  //同一个点连续扫描五次，如果都是障碍物的话，则判断为是不可通行区域ZhuBC
			  ogm_data.ogm[i] = RIGIDNOPASSABLE;
		  }
		  else
		  {
			  ogm_data.ogm[i] = PASSABLE;
		  }
	  }
	  for (int i = 0; i < borderpointcloud.points.size(); i++)
	  {
	      float x = borderpointcloud.points[i].x,
			      y = borderpointcloud.points[i].y,
			      z = borderpointcloud.points[i].z;

	      float newy = y + ogm_y_offset;//栅格地图中的实际y值
	      //判断是否在栅格地图内部
	      if((x >=-ogm_data.ogmwidth/2 &&x <= ogm_data.ogmwidth/2 ) && (newy >=0 && newy < ogm_data.ogmheight) && (z >= - 1.5 && z <=  Z_MAX))
	      {
		      int col = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;//横向栅格
		      int row = boost::math::round(newy / ogm_data.ogmresolution) ;//纵向栅格

		      if((row >=0 && row < ogm_data.ogmheight_cell) && (col >=0 && col < ogm_data.ogmwidth_cell))
		      {
				  int index = row * ogm_data.ogmwidth_cell + col;
				  //如果为之前检测到的路沿，则计数器加1
				  if(ogm_data.ogm[index] != RIGIDNOPASSABLE)
					  borderpointcloud.points[i].passibility = 1;
		      }
	      }
		  }
	  delete [] passible_point_count_ogm_count;
	  delete [] nopassible_point_count_ogm_count;
	}
}

void LidarProcess::cloud2OGM(const Cloud& pointcloud,OGMData<unsigned char>& ogm_data,int countthredshold)
{
	memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
	unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);

	for (int i = 0; i < pointcloud.size(); i++) {
		float x = pointcloud.points[i].x
		    , y =pointcloud.points[i].y
		    , z =pointcloud.points[i].z;
		float newx = x + ogm_data.vehicle_x;
		float newy = y + ogm_data.vehicle_y;	  //栅格地图中的实际y值
		//判断是否在栅格地图内部
		if ((newx >= 0
				&& newx < ogm_data.ogmwidth)
				&& (newy >= 0 && newy < ogm_data.ogmheight)
				&& (z >= -1.5 && z <= Z_MAX)) {
			int col = boost::math::round(
					newx / ogm_data.ogmresolution);			  //横向栅格
			int row = boost::math::round(
					newy / ogm_data.ogmresolution);		  //纵向栅格
			if ((row >= 0 && row < ogm_data.ogmheight_cell)
					&& (col >= 0 && col < ogm_data.ogmwidth_cell)) {
				int index = row * ogm_data.ogmwidth_cell + col;
				//如果为之前检测到的路沿，则计数器加1
				if (pointcloud.points[i].passibility < 0.5)
					nopassible_point_count_ogm_count[index]++;
				else
					passible_point_count_ogm_count[index]++;
			}
		}
	}
	for(int i = 0 ; i < ogm_data.ogmcell_size; i++)
	{
		if(nopassible_point_count_ogm_count[i] > countthredshold)
		  {
		//同一个点连续扫描五次，如果都是障碍物的话，则判断为是不可通行区域ZhuBC
			ogm_data.ogm[i] = RIGIDNOPASSABLE;
		  }
		else
		  {
			ogm_data.ogm[i] = PASSABLE;
		  }
	}
	delete [] passible_point_count_ogm_count;
	delete [] nopassible_point_count_ogm_count;
}

void LidarProcess::ogmDetection()
{

    //网格初始化
    //refineogm.ogm属性有unknown,rigidnopassable和passable
    memset(hdl_ogm_data.ogm , 0 , hdl_ogm_data.ogmcell_size);
    memset(refineogm.ogm , UNKNOWN , refineogm.ogmcell_size);
    for(int i = 0 ; i < minzogm.ogmcell_size ; i++)
        minzogm.ogm[i] = 1000.0f;
    for(int i = 0 ; i < minzrefineogm.ogmcell_size ; i++)
        minzrefineogm.ogm[i] = 1000.0f;


    //farrefinedogm.ogm属性有unknown,rigidnopassable
    memset(farrefinedogm.ogm , UNKNOWN , farrefinedogm.ogmcell_size);
    for(int i = 0 ; i < farminzogm.ogmcell_size ; i++)
        farminzogm.ogm[i] = 1000.0f;

    //laneogm.ogm属性有unknown,rigidnopassable
    memset(laneogm.ogm , UNKNOWN , laneogm.ogmcell_size);
    for(int i = 0 ; i < laneogm.ogmcell_size ; i++){
        laneminintensity_ogm[i] = 1000.0f;
        lanemaxintensity_ogm[i] = -1000.0f;
    }
    //intensityogm.ogm
    memset(intensityogm.ogm, UNKNOWN, intensityogm.ogmcell_size);
    for(int i = 0 ; i < intensityogm.ogmcell_size; i++){
        minintensity_ogm[i] = 255;
        maxintensity_ogm[i] = 0;
    }
    Cloud::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0;i < velodyne_pointcloud->size();i++)
    {

	    if(velodyne_pointcloud->points[i].range >= 0.5  )
	    {

		    PointXYZI newpoint = velodyne_pointcloud->points[i];
		    pointcloud->points.push_back(newpoint);

	    }
    }

    if(1){
        //近处网格，用大网格分析高度差，然后投影到小网格中去。对于远距离马路沿子有较好结果，但是噪声可能产生干扰
        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;
            int intensity  = pointcloud->points[i].intensity;
            float newy = y + ogm_y_offset;//整体往后移动了ogm_y_offset

            if((x >=-intensityogm.ogmwidth / 2  && x <= intensityogm.ogmwidth / 2) &&
                    (newy >=0 && newy < intensityogm.ogmheight) &&
                    (z >= - 1.5 && z <=  Z_MAX)){
                int col = boost::math::round(x / intensityogm.ogmresolution) + (intensityogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / intensityogm.ogmresolution) ;

                if((row >=0 && row < intensityogm.ogmheight_cell)
                        && (col >=0 && col < intensityogm.ogmwidth_cell)){
                    int index = row * intensityogm.ogmwidth_cell + col;
                    if (intensityogm.ogm[index] < intensity)
                    {
                        intensityogm.ogm[index] = intensity;
                    }

                }
            }

            if(pointcloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
                continue;

            if(x>DEADZONE_LEFT&&x<DEADZONE_RIGHT&&y>DEADZONE_BACK&&y<DEADZONE_FRONT)
                continue;
            newy = y + minzrefineogm.ogmheight/2;
            if((x >=-minzrefineogm.ogmwidth / 2  && x <= minzrefineogm.ogmwidth / 2) &&
                    (newy >=0 && newy < minzrefineogm.ogmheight) &&
                    (z >= - 2 && z <=  Z_MAX)){
                int col = boost::math::round(x / minzrefineogm.ogmresolution) + ( minzrefineogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / minzrefineogm.ogmresolution) ;

                if((row >=0 && row < minzrefineogm.ogmheight_cell)
                        && (col >=0 && col < minzrefineogm.ogmwidth_cell)){
                    int index = row * minzrefineogm.ogmwidth_cell + col;
                    if( minzrefineogm.ogm[index] >  z)
                        minzrefineogm.ogm[index] = z;
                }
            }

            newy = y + refineogm.ogmheight/2;
            if((x >=-refineogm.ogmwidth / 2  && x <= refineogm.ogmwidth / 2) &&
                    (newy >=0 && newy < refineogm.ogmheight) &&
                    (z >= - 1.5 && z <=  Z_MAX)){
                int col = boost::math::round(x / minzogm.ogmresolution) + ( minzogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / minzogm.ogmresolution) ;

                if((row >=0 && row < minzogm.ogmheight_cell)
                        && (col >=0 && col < minzogm.ogmwidth_cell)){
                    int index = row * minzogm.ogmwidth_cell + col;
                    if( minzogm.ogm[index] >  z)
                        minzogm.ogm[index] = z;
                }
            }
        }

        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;

            float newy = y + ogm_y_offset;//整体往下移动了ogm_y_offset米

            if((x >=-refineogm.ogmwidth / 2  && x <= refineogm.ogmwidth / 2) &&
                    (newy >=0 && newy < refineogm.ogmheight) &&
                    (z >= - 1.5 && z <=  Z_MAX)){

                int col = boost::math::round(x / minzogm.ogmresolution) + ( minzogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / minzogm.ogmresolution) ;

                if((row >=0 && row < minzogm.ogmheight_cell)
                        && (col >=0 && col < minzogm.ogmwidth_cell)){

                    int index = row * minzogm.ogmwidth_cell + col;

                    if( z -  minzogm.ogm[index] > rigid_heightdiffthreshold_){

                        int col1 = boost::math::round(x / refineogm.ogmresolution) + ( refineogm.ogmwidth_cell - 1 ) / 2;
                        int row1 = boost::math::round(newy / refineogm.ogmresolution) ;

                        int index1 = row1 * refineogm.ogmwidth_cell + col1;
                        refineogm.ogm[index1] = RIGIDNOPASSABLE;
                    }
                }
            }
        }
    }

    //近处网格，用绝对高度，对于远距离的较低障碍效果不明显，对于车辆效果很好
    if(0){
        float slopethreshold = (3 * M_PI / 180);
        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;

            float newy = y + ogm_y_offset;//整体往下移动了ogm_y_offset米

            //精细网格
            int col = boost::math::round(x / refineogm.ogmresolution) + ( refineogm.ogmwidth_cell - 1 ) / 2;
            int row = boost::math::round(newy / refineogm.ogmresolution) ;

            if((row >=0 && row < refineogm.ogmheight_cell)
                    && (col >=0 && col < refineogm.ogmwidth_cell)){
                int index = row * refineogm.ogmwidth_cell + col;

                float dist = sqrt(x * x + y * y);
                float slope = fabs(z) / dist;

                if(slope > slopethreshold)
                    refineogm.ogm[index] = RIGIDNOPASSABLE;
            }
        }
    }

    //近处车道线分析
    if(0){
        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z,
                  intensity = pointcloud->points[i].intensity;

            float newy = y + laneogm_y_offset;//整体往下移动了laneogm_y_offset米

            //精细网格
            int col = boost::math::round(x / laneogm.ogmresolution) + ( laneogm.ogmwidth_cell - 1 ) / 2;
            int row = boost::math::round(newy / laneogm.ogmresolution) ;

            if((row >=0 && row < laneogm.ogmheight_cell)
                    && (col >=0 && col < laneogm.ogmwidth_cell)){
                int index = row * laneogm.ogmwidth_cell + col;

                if(laneminintensity_ogm[index] >  intensity)
                    laneminintensity_ogm[index]  = intensity;

                if(lanemaxintensity_ogm[index] < intensity)
                    lanemaxintensity_ogm[index]  = intensity;

                //if(refineogm.ogm[index] != RIGIDNOPASSABLE){

                //	if(intensity > 10.0f){
                //
                //		pcl::PointXYZRGB newpoint_intensity;
                //		newpoint_intensity.x = pointcloud->points[i].x;
                //		newpoint_intensity.y = pointcloud->points[i].y;
                //		newpoint_intensity.z = pointcloud->points[i].z;
                //		newpoint_intensity.b = 8.77192982456f * pointcloud->points[i].intensity;
                //		newpoint_intensity.g = 3.70357751278f * pointcloud->points[i].intensity;
                //		newpoint_intensity.r = 5.34448160535f * pointcloud->points[i].intensity;
                //		cloud_vehicle_hdl_intensity->points.push_back(newpoint_intensity);
                //	}
                //}
            }
        }

        //laneogm.ogm初始值是Unknown，在障碍物附近的清空为passable，不作考虑
        int range = 0.6f / laneogm.ogmresolution;
        for(int row = range; row < laneogm.ogmheight_cell - range; row++){
            for(int col = range ; col < laneogm.ogmwidth_cell - range; col++){
                int row1 = row + 10.0f / laneogm.ogmresolution;
                int col1 = col + 10.0f / laneogm.ogmresolution;
                int index1 = row1 * refineogm.ogmwidth_cell + col1;
                if(refineogm.ogm[index1] == RIGIDNOPASSABLE){

                    for(int i = row - range; i < row + range; i++){
                        for(int j = col - range; j < col + range; j++){
                            int index = i * laneogm.ogmwidth_cell + j;
                            laneogm.ogm[index] = PASSABLE;
                        }
                    }
                }
            }
        }

        for(int row = 0; row < laneogm.ogmheight_cell; row++){
            for(int col = 1 ; col < laneogm.ogmwidth_cell - 1; col++){
                int index = row * laneogm.ogmwidth_cell + col;
                if(laneogm.ogm[index] != PASSABLE){
                    if(fabs(laneminintensity_ogm[index] - 1000.0f) > EPSILON){
                        if(lanemaxintensity_ogm[index] - laneminintensity_ogm[index] > 7.0f){
                            laneogm.ogm[index] = RIGIDNOPASSABLE;
                        }


                        if(fabs(laneminintensity_ogm[index - 1] - 1000.0f) > EPSILON){
                            if(lanemaxintensity_ogm[index] - laneminintensity_ogm[index - 1] > 7.0f){
                                laneogm.ogm[index] = RIGIDNOPASSABLE;
                            }
                        }

                        if(fabs(laneminintensity_ogm[index + 1] - 1000.0f) > EPSILON){
                            if(lanemaxintensity_ogm[index] - laneminintensity_ogm[index + 1] > 7.0f){
                                laneogm.ogm[index] = RIGIDNOPASSABLE;
                            }
                        }
                    }
                }
            }
        }
    }


    if(1){
        //远处网格，用大网格分析高度差，然后投影到小网格中去。
        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;

            float newy = y + farogm_y_offset;

            if((x >=-farminzogm.ogmwidth / 2  && x <= farminzogm.ogmwidth / 2) &&
                    (newy >= 0 && newy < farminzogm.ogmheight) &&
                    (z >= - Z_MAX && z <=  Z_MAX)){

                int col = boost::math::round(x / farminzogm.ogmresolution) + ( farminzogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / farminzogm.ogmresolution) ;

                if((row >=0 && row < farminzogm.ogmheight_cell)
                        && (col >=0 && col < farminzogm.ogmwidth_cell)){
                    int index = row * farminzogm.ogmwidth_cell + col;
                    if(farminzogm.ogm[index] >  z)
                        farminzogm.ogm[index] = z;
                }
            }
        }

        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;

            float newy = y + farogm_y_offset;

            if((x >=-farminzogm.ogmwidth / 2  && x <= farminzogm.ogmwidth / 2) &&
                    (newy >= 0 && newy < farminzogm.ogmheight) &&
                    (z >= - Z_MAX && z <=  Z_MAX)){

                int col = boost::math::round(x / farminzogm.ogmresolution) + ( farminzogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / farminzogm.ogmresolution) ;

                if((row >=0 && row < farminzogm.ogmheight_cell)
                        && (col >=0 && col < farminzogm.ogmwidth_cell)){

                    int index = row * farminzogm.ogmwidth_cell + col;

                    if( z - farminzogm.ogm[index] > 0.3f){

                        int col1 = boost::math::round(x / farrefinedogm.ogmresolution) + ( farrefinedogm.ogmwidth_cell - 1 ) / 2;
                        int row1 = boost::math::round(newy / farrefinedogm.ogmresolution) ;

                        int index1 = row1 * farrefinedogm.ogmwidth_cell + col1;
                        farrefinedogm.ogm[index1] = RIGIDNOPASSABLE;
                    }
                }
            }
        }
    }

    if(0){
        //远处数据，依赖于反射率
        for (int i = 0; i < pointcloud->points.size(); i++){
            float x = pointcloud->points[i].x,
                  y = pointcloud->points[i].y,
                  z = pointcloud->points[i].z;

            float newy = y + farogm_y_offset;

            if((x >=-farrefinedogm.ogmwidth / 2  && x <= farrefinedogm.ogmwidth / 2) &&
                    (newy >= 0 && newy < farrefinedogm.ogmheight) &&
                    (z >= - Z_MAX && z <=  Z_MAX)){

                int col = boost::math::round(x / farrefinedogm.ogmresolution) + ( farrefinedogm.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / farrefinedogm.ogmresolution) ;

                if((row >=0 && row < farrefinedogm.ogmheight_cell) &&
                        (col >=0 && col < farrefinedogm.ogmwidth_cell)){
                    int index = row * farrefinedogm.ogmwidth_cell + col;

                    //计算网格内最高点和最低点用于分析高度差RIGIDNOPASSABLE
                    if (0)
                    {
                        if(pointcloud->points[i].intensity > 15.0f)
                            farrefinedogm.ogm[index] = RIGIDNOPASSABLE;//绝对不可行
                    }

                }
            }

        }
    }

//    if(1)
//    {
//        int index=0;
//        for(int i=0; i<refineogm.ogmcell_size;i++,index++)
//        {
//            if(refineogm.ogm[i]==RIGIDNOPASSABLE)
//                hdl_ogm_data.ogm[index]=refineogm.ogm[i];
//            //refineogm.ogm[i]=ogm_data.ogm[index];
//        }
//        for(int i=0; i<farrefinedogm.ogmcell_size ;i++,index++)
//        {
//            if(farrefinedogm.ogm[i]==RIGIDNOPASSABLE)
//                hdl_ogm_data.ogm[index]=farrefinedogm.ogm[i];
//            //farrefinedogm.ogm[i]=ogm_data.ogm[index];
//        }
//        hdl_ogm_data.updateflag=true;
//        hdl_ogm_data.updateStamp();
//    }

}


void LidarProcess::heightdiffOgmDetection(const Cloud& pointcloud,Cloud& heightdiffpointcloud
				,OGMData<unsigned char>& ogm_data
				,double resolution
				,double heightdiffthreshold , int countthreshold)
{

    double vehicle_x = ogm_data.vehicle_x;
    double vehicle_y = ogm_data.vehicle_y;
    OGMData<float> minzogm(ogm_data.ogmheight,ogm_data.ogmwidth,resolution);
    unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
    //网格初始化
    //ogm_data.ogm属性有unknown,rigidnopassable和passable

    memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
    for(int i = 0;i<minzogm.ogmcell_size;i++)
	{
    	minzogm.ogm[i]=1000;
	}


    if(1){
        //近处网格，用大网格分析高度差，然后投影到小网格中去。对于远距离马路沿子有较好结果，但是噪声可能产生干扰
        for (int i = 0; i < pointcloud.points.size(); i++){
            if(pointcloud.points[i].range < 0.5)
              continue;
            float x = pointcloud.points[i].x,
                  y = pointcloud.points[i].y,
                  z = pointcloud.points[i].z;
            int intensity  = pointcloud.points[i].intensity;
            float newy = y + vehicle_y;//整体往后移动了ogm_y_offset
            float newx = x + vehicle_x;

            if((newx >=0  && newx <= minzogm.ogmwidth) &&
                    (newy >=0 && newy < minzogm.ogmheight) &&
                    (z >= - 2 && z <=  Z_MAX)){

                int col = boost::math::round(newx / minzogm.ogmresolution) ;
                int row = boost::math::round(newy / minzogm.ogmresolution) ;

                if((row >=0 && row < minzogm.ogmheight_cell)
                        && (col >=0 && col < minzogm.ogmwidth_cell)){
                    int index = row * minzogm.ogmwidth_cell + col;
                    if( minzogm.ogm[index] >  z)
                        minzogm.ogm[index] = z;
                }
            }

        }

        for (int i = 0; i < pointcloud.points.size(); i++){
            if(pointcloud.points[i].range < 0.5)
              continue;
            float x = pointcloud.points[i].x,
                  y = pointcloud.points[i].y,
                  z = pointcloud.points[i].z;

            float newy = y + vehicle_y;//整体往下移动了ogm_y_offset米
            float newx = x + vehicle_x;

            if((newx >= 0  && newx <= ogm_data.ogmwidth) &&
                    (newy >=0 && newy < ogm_data.ogmheight) &&
                    (z >= - 1.5 && z <=  Z_MAX)){

                int col = boost::math::round(newx / minzogm.ogmresolution) ;
                int row = boost::math::round(newy / minzogm.ogmresolution) ;

                if((row >=0 && row < minzogm.ogmheight_cell)
                        && (col >=0 && col < minzogm.ogmwidth_cell)){

                    int index = row * minzogm.ogmwidth_cell + col;

                    if( z -  minzogm.ogm[index] > heightdiffthreshold){
                    	heightdiffpointcloud.push_back(pointcloud.points[i]);
                    	heightdiffpointcloud.back().passibility = 0;
                        int col1 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
                        int row1 = boost::math::round(newy / ogm_data.ogmresolution) ;

                        int index1 = row1 * ogm_data.ogmwidth_cell + col1;
//                        ogm_data.ogm[index1] = RIGIDNOPASSABLE;
                        nopassible_point_count_ogm_count[index1]++;
                    }
                    else
                    {
//                    	pointcloud.points[i].passibility = 1;
                    	int col1 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
						int row1 = boost::math::round(newy / ogm_data.ogmresolution) ;
						int index1 = row1 * ogm_data.ogmwidth_cell + col1;
						passible_point_count_ogm_count[index1]++;
                    }
                }
            }
        }
    for(int i = 0; i < ogm_data.ogmcell_size;i++)
	{
	    if(nopassible_point_count_ogm_count[i]>countthreshold &&passible_point_count_ogm_count[i]>countthreshold)
		    ogm_data.ogm[i] = RIGIDNOPASSABLE;
	}
	for(int i = 0;i < heightdiffpointcloud.points.size(); i++)
	{
	    if(heightdiffpointcloud.points[i].passibility <=0.1)
	    {
		float x = heightdiffpointcloud.points[i].x, y = heightdiffpointcloud.points[i].y,z = heightdiffpointcloud.points[i].z;
		float newy = y + vehicle_y;//整体往下移动了ogm_y_offset米
		float newx = x + vehicle_x;
		if((newx >= 0  && newx <= ogm_data.ogmwidth) &&(newy >=0 && newy < ogm_data.ogmheight)
		    &&	(z >= - 1.5 && z <=  Z_MAX))
		{
		  int col_2 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
		  int row_2 = boost::math::round(newy / ogm_data.ogmresolution) ;
		  int index_2 =  row_2 * ogm_data.ogmwidth_cell + col_2;
		  if(ogm_data.ogm[index_2]==RIGIDNOPASSABLE)
			  heightdiffpointcloud.points[i].passibility = 0;
		  else
			  heightdiffpointcloud.points[i].passibility = 1;
		}
	    }
	}
    }
        ogm_data.updateStamp();
        delete [] passible_point_count_ogm_count;
        delete [] nopassible_point_count_ogm_count;
     //  pointcloud = *pointcloud;
    }

bool LidarProcess::DoorDetection(Cloud& pointcloud,int pointnum_threshold,double xmin,double xmax,double ymin,double ymax,double zmin,double zmax)
{

	int pointnum = 0;
	for (int i = 0; i < pointcloud.points.size(); i++){
		if(pointcloud.points[i].range < 0.5)
		  continue;
		float x = pointcloud.points[i].x,
			  y = pointcloud.points[i].y,
			  z = pointcloud.points[i].z;


		if((x >=xmin  && x <= xmax) &&
				(y >=ymin && y < ymax) &&
				(z >= zmin && z <=  zmax)){
			pointnum++;
		}

	}

	if(pointnum > pointnum_threshold)
		return true;
	else
		return false;


}



void LidarProcess::heightdiffDetection()
{

    //
    int col_count = velodyne_pointcloud->size() / LASER_LAYER;
    float deltaz_thresh = 0.3;//adjustable param //0.3
    float negative_deltaz_thresh=0.6;//adjustable param
    float slope_thresh = tan(25* M_PI / 180);// adjustable param //30


    for(int i = 0 ; i < col_count ; i++)
    {

        float azimuth = velodyne_pointcloud->points[i * LASER_LAYER].azimuth;

        for(int j = 0 ; j < LASER_LAYER/**2/3*/ ; j++)
        {


            int index_j=grabber_H_.indexmaptable[j].number;
            int ori_index =i * LASER_LAYER + grabber_H_.indexmaptable[j].number;
            pcl::PointXYZI temppointj=velodyne_pointcloud->points[ori_index];

            if(temppointj.range < 0.5|| temppointj.range>60)
                continue;
            //                                        if(temppointj.range>maxrange)
            //                                            break;

//            if(velodyne_pointcloud->points[ori_index].y<4&&velodyne_pointcloud->points[ori_index].y>-4&&velodyne_pointcloud->points[ori_index].x>-2.5&&velodyne_pointcloud->points[ori_index].x<2.5) //back exist false detection , so need delete it
//                continue;

            if(velodyne_pointcloud->points[ori_index].y< -20||velodyne_pointcloud->points[ori_index].x< -30||velodyne_pointcloud->points[ori_index].x > 30) //back exist false detection , so need delete it
                continue;

            if(velodyne_pointcloud->points[ori_index].z< Z_MIN||velodyne_pointcloud->points[ori_index].z > Z_MAX) //back exist false detection , so need delete it
                continue;

/*            if(j>5 && temppointj.range < 5 && temppointj.z>0.3)
                velodyne_pointcloud->points[ori_index].passibility = 0;*/
            //last point in circle j
            int i_start,i_stop;
            if(i>col_count/2)
            {
                i_start=i-6;
                i_stop=i-2;
            }
            else
            {
                i_start=i+2;
                i_stop=i+6;
            }

            double slope_circle=0;


            for(int t = j + 1 ; t < LASER_LAYER && t< j+6 ; t+=1)
            {
                int index_t=grabber_H_.indexmaptable[t].number;
                int ori_index1 =i * LASER_LAYER + grabber_H_.indexmaptable[t].number;



                pcl::PointXYZI temppointt=velodyne_pointcloud->points[ori_index1];

                double tempazimuthj=temppointj.azimuth ;
                int mat_i=tempazimuthj * 10;

                int mat_i_start=mat_i-2;
                if (mat_i_start<0)
                    mat_i_start=0;

                int mat_i_stop=mat_i+2;
                if (mat_i_stop>3599)
                    mat_i_stop=3599;
                int disminindex=-1;
                int disminmat_i=mat_i_start;
                double dismin = 1000;
                for(int temp_i = mat_i_start ; temp_i<=mat_i_stop;temp_i++)
                {
                    double tempdis= polaraxismat_[index_t][temp_i].distance;
                    int index_temp = polaraxismat_[index_t][temp_i].index;
                    if(tempdis<0.5)
                        continue;
                    if(velodyne_pointcloud->points[index_temp].passibility < 0.5)
                        continue;
                    if  (dismin > tempdis)
                    {
                        dismin = tempdis;
                        disminindex=polaraxismat_[index_t][temp_i].index;
                        disminmat_i=temp_i;
                    }
                }
                if(disminindex < 0)
                    continue;

                ori_index1=disminindex;
                temppointt=velodyne_pointcloud->points[ori_index1];

                // cout<<"t-j="<<t-j<<"\tazimuthj="<<temppointj.azimuth<<"\tazimutht="<<temppointt.azimuth<<endl;
                if(temppointt.range < 0.5)
                    continue;



                bool obstacle_found = false;


                if(0)
                    //if(fabs(temppointt.range - temppointj.range)<0.4 || t-j == 1)
                {
                    //height delta
                    float deltaz = temppointt.z - temppointj.z;
                    float deltar = temppointt.range - temppointj.range;
                    //                                                 cout<<"t-j="<<t-j<<"\tazimuthj="<<temppointj.azimuth<<"\tazimutht="<<temppointt.azimuth<<endl;
                    if(temppointt.z - temppointj.z > deltaz_thresh && fabs(temppointt.range - temppointj.range)<0.4 )
                    {
#ifdef USE_OMP
                        omp_set_lock(&omplock); //获得互斥器
#endif
                        velodyne_pointcloud->points[ori_index1].passibility = 0.0;
                        int index_tempt=ori_index1-LASER_LAYER;
                        int index_tempj=ori_index-LASER_LAYER;
                        if(index_tempt>=0&&index_tempj>=0 && (fabs(velodyne_pointcloud->points[index_tempt].z - velodyne_pointcloud->points[index_tempj].z)> deltaz_thresh-0.05))
                            velodyne_pointcloud->points[index_tempt].passibility = 0.0;

                        index_tempt=ori_index1+LASER_LAYER;
                        index_tempj=ori_index+LASER_LAYER;
                        if(index_tempt<col_count*LASER_LAYER &&index_tempj<col_count*LASER_LAYER
                                && (fabs(velodyne_pointcloud->points[index_tempt].z - velodyne_pointcloud->points[index_tempj].z)> deltaz_thresh-0.05))
                            velodyne_pointcloud->points[index_tempt].passibility = 0.0;


#ifdef USE_OMP
                        omp_unset_lock(&omplock); //释放互斥器
#endif
                        obstacle_found = true;
                    }
                    // jkj 2016/10/28 use double thresh because of a little false detection
                    else if(temppointj.z - temppointt.z > negative_deltaz_thresh &&t-j == 1)//detect negative
                    {

                        obstacle_found = true;
#ifdef USE_OMP
                        omp_set_lock(&omplock); //获得互斥器
#endif
                        velodyne_pointcloud->points[ori_index].passibility = 0.0;
#ifdef USE_OMP
                        omp_unset_lock(&omplock); //释放互斥器
#endif
                    }

                    if(obstacle_found)
                        break;//

                    // if(0)
                    if(deltaz<-0.1&&t<LASER_LAYER/2&&t-j==1)  //negative obstacle
                    {
                        int index_next=grabber_H_.indexmaptable[t+1].number;
                        double tempazimuthnext=temppointt.azimuth ;
                        int mat_i=tempazimuthnext * 10;

                        int mat_i_start=mat_i-2;
                        if (mat_i_start<0)
                            mat_i_start=0;

                        int mat_i_stop=mat_i+2;
                        if (mat_i_stop>3599)
                            mat_i_stop=3599;
                        int disminindex=-1;
                        double dismin = 1000;
                        for(int temp_i = mat_i_start ; temp_i<=mat_i_stop;temp_i++)
                        {
                            double tempdis= polaraxismat_[index_next][temp_i].distance;
                            if(tempdis<0.5)
                                continue;
                            int index_temp = polaraxismat_[index_next][temp_i].index;
                            if(velodyne_pointcloud->points[index_temp].passibility < 0.5)
                                continue;
                            if  (dismin > tempdis)
                            {
                                dismin = tempdis;
                                disminindex=polaraxismat_[index_next][temp_i].index;
                            }
                        }
                        if(disminindex < 0)
                            continue;



                        ori_index1=disminindex;
                        pcl::PointXYZI temppointnext=velodyne_pointcloud->points[ori_index1];
                        if(temppointnext.z-temppointt.z>0.1&&2*(temppointnext.range-temppointt.range)<temppointt.range-temppointj.range)
                        {
#ifdef USE_OMP
                            omp_set_lock(&omplock); //获得互斥器
#endif
                            velodyne_pointcloud->points[ori_index1].passibility = 0.0;
                            velodyne_pointcloud->points[ori_index].passibility = 0.0;
                            obstacle_found = true;
#ifdef USE_OMP
                            omp_unset_lock(&omplock); //释放互斥器
#endif
                        }
                    }

                }
                if(obstacle_found)
                    break;//

                //if(0)
                if(/*t-j>=3&&*/fabs(temppointt.range - temppointj.range)>0.5||t-j>2&&t-j<6)  //slope detect
                {
                    if(temppointj.passibility<0.7)
                        continue;
                    //
                    float deltaz = temppointt.z - temppointj.z;
                    float deltar = temppointt.range - temppointj.range;
                    float slope ;
                    float slope_radial;

                    //                                                pcl::PointXYZI temppointi;

                    //                                                        temppointi=velodyne_pointcloud->points[ori_index1];
                    //                                                if(disminmat_i)

                    slope_radial= fabs(deltaz / deltar);
                    slope=sqrt(slope_circle*slope_circle+slope_radial*slope_radial);
                    if(slope > slope_thresh)
                    {
                        if(/*temppointt.z - temppointj.z */ deltaz > 0/*&&t-j>3*//*&& (fabs(temppointt.range - temppointj.range)>0.2||temppointj.range>10)*/)  //positive scope
                        {
#ifdef USE_OMP
                            omp_set_lock(&omplock); //获得互斥器
#endif
                            velodyne_pointcloud->points[ori_index1].passibility = HDL_MIN(0.6,velodyne_pointcloud->points[ori_index1].passibility);

                            int index_tempt=ori_index1-LASER_LAYER;
                            int index_tempj=ori_index -LASER_LAYER;
                            if(index_tempt>=0 &&index_tempj>=0 )
                            {
                                deltaz=velodyne_pointcloud->points[index_tempt].z - velodyne_pointcloud->points[index_tempj].z;
                                deltar=velodyne_pointcloud->points[index_tempt].range - velodyne_pointcloud->points[index_tempj].range;
                                slope_radial= fabs(deltaz / deltar);
                                slope=sqrt(slope_circle*slope_circle+slope_radial*slope_radial);
                                if(slope > slope_thresh)
                                    velodyne_pointcloud->points[index_tempt].passibility = HDL_MIN(0.6,velodyne_pointcloud->points[index_tempt].passibility);
                            }
                            index_tempt=ori_index1+LASER_LAYER;
                            index_tempj=ori_index + LASER_LAYER;
                            if(index_tempt<col_count*LASER_LAYER &&index_tempj<col_count*LASER_LAYER)
                            {
                                deltaz=velodyne_pointcloud->points[index_tempt].z - velodyne_pointcloud->points[index_tempj].z;
                                deltar=velodyne_pointcloud->points[index_tempt].range - velodyne_pointcloud->points[index_tempj].range;
                                slope_radial= fabs(deltaz / deltar);
                                slope=sqrt(slope_circle*slope_circle+slope_radial*slope_radial);
                                if(slope > slope_thresh)
                                    velodyne_pointcloud->points[index_tempt].passibility = HDL_MIN(0.6,velodyne_pointcloud->points[index_tempt].passibility);
                            }
#ifdef USE_OMP
                            omp_unset_lock(&omplock); //释放互斥器
#endif

                            if( velodyne_pointcloud->points[ori_index1].passibility<0.5)
                                obstacle_found = true;
                        }

                    }

                }

                if(obstacle_found)
                    break;//
            }
        }

    }

}

void LidarProcess::circletangentialDetection()
{

    //slope detect
    int col_count = velodyne_pointcloud->size() / LASER_LAYER;
    float delta_angle_thresh = 30.0;//adjustable param delta_angle = actual_tangential_angle - expected_tangential_angle;
    int cloud_window_size = 10;//adjustable param cloud_window_size

    for(int laser_i = 0 ; laser_i < LASER_LAYER *2/3; laser_i++)
    {
        int i=grabber_H_.indexmaptable[laser_i].number;


        //                   if(verticalangle < verticalangle_thresh)
        {

            for(int j = cloud_window_size ; j < col_count - cloud_window_size; j++)
            {
                int index = j * LASER_LAYER + i;


                if(velodyne_pointcloud->points[index].range < 0.5)
                    continue;
                if(velodyne_pointcloud->points[index].x>-2&&velodyne_pointcloud->points[index].x<2&&velodyne_pointcloud->points[index].y<0)
                    continue;
                if(velodyne_pointcloud->points[index].z< Z_MIN&&velodyne_pointcloud->points[index].z > Z_MAX)
                    continue;

                //
                float azimuth = 360.0 - velodyne_pointcloud->points[index].azimuth + 90.0 + gama_h * 180 / M_PI;
                while(azimuth > 360.0) azimuth -=360.0;
                while(azimuth <=0) azimuth +=360.0;
                //if( azimuth> 0 && azimuth < 180)
                {
                    int index1 = (j + cloud_window_size) * LASER_LAYER + i;
                    int index2 = (j - cloud_window_size) * LASER_LAYER + i;

                    int index3 = (j + cloud_window_size/2) * LASER_LAYER + i;
                    int index4 = (j - cloud_window_size/2) * LASER_LAYER + i;

                    if(velodyne_pointcloud->points[index1].range < 0.5)
                        continue;
                    if(velodyne_pointcloud->points[index2].range < 0.5)
                        continue;
                    if(velodyne_pointcloud->points[index3].range < 0.5)
                        continue;
                    if(velodyne_pointcloud->points[index4].range < 0.5)
                        continue;

                    float slope_angle = atan2(velodyne_pointcloud->points[index].y, velodyne_pointcloud->points[index].x) * 180 / M_PI;
                    float expected_tangential_angle = slope_angle - 90;
                    //+-M_PI
                    if(expected_tangential_angle > 180) expected_tangential_angle -= 360;
                    else if(expected_tangential_angle <= -180) expected_tangential_angle += 360;

                    float actual_tangential_angle = atan2(velodyne_pointcloud->points[index1].y - velodyne_pointcloud->points[index2].y,
                            velodyne_pointcloud->points[index1].x - velodyne_pointcloud->points[index2].x) * 180 / M_PI;

                    float actual_tangential_angle34 = atan2(velodyne_pointcloud->points[index3].y - velodyne_pointcloud->points[index4].y,
                            velodyne_pointcloud->points[index3].x - velodyne_pointcloud->points[index4].x) * 180 / M_PI;

                    float actual_tangential_angle1 = atan2(velodyne_pointcloud->points[index].y - velodyne_pointcloud->points[index1].y,
                            velodyne_pointcloud->points[index].x - velodyne_pointcloud->points[index1].x) * 180 / M_PI;

                    float actual_tangential_angle2 = atan2(velodyne_pointcloud->points[index2].y - velodyne_pointcloud->points[index].y,
                            velodyne_pointcloud->points[index2].x - velodyne_pointcloud->points[index].x) * 180 / M_PI;

                    //+-M_PI
                    float delta_angle = actual_tangential_angle - expected_tangential_angle;
                    if(delta_angle > 180) delta_angle -= 360;
                    else if(delta_angle < -180) delta_angle += 360;

                    if(fabs(delta_angle) > delta_angle_thresh/*&&fabs(actual_tangential_angle-actual_tangential_angle34)<delta_angle_thresh/2*/
                            /*&&fabs(actual_tangential_angle1-actual_tangential_angle2)<delta_angle_thresh/2*/)
                    {
                        //cout << " " << velodyne_pointcloud->points[index].azimuth << " " <<
                        //	slope_angle << " " << expected_tangential_angle << " " << actual_tangential_angle << " " << fabs(delta_angle) << endl;
#ifdef USE_OMP
                        omp_set_lock(&omplock); //获得互斥器
#endif
                        if(delta_angle>0)
                            velodyne_pointcloud->points[index].passibility =  HDL_MIN(0.8,velodyne_pointcloud->points[index].passibility);  //negative slope
                        else
                            velodyne_pointcloud->points[index].passibility =  HDL_MIN(-0.8,velodyne_pointcloud->points[index].passibility);  //negative slope

#ifdef USE_OMP
                        omp_unset_lock(&omplock); //释放互斥器
#endif
                    }
                }
            }
        }
    }


}
void LidarProcess::circleradiusDetection()
{


    int col_count = velodyne_pointcloud->size() / LASER_LAYER;
    float delta_dis_thresh = 1.2;//adjustable param
    float delta_dis_thresh2=delta_dis_thresh/2;
    int max_min_counter_thresh=2;
    int cloud_window_size = 10;//adjustable param cloud_window_size

    for(int dis_i = 0 ; dis_i < LASER_LAYER ; dis_i++)
    {
        if(theorydis[dis_i] > 40 || theorydis[dis_i]<0)
            continue;
        //float verticalangle = laserverticalangle[i] * M_PI / 180 + beta_h;
        float verticalangle=grabber_H_.indexmaptable[dis_i].angle*M_PI/180+beta_h;
        int i=grabber_H_.indexmaptable[dis_i].number;

        float temptheorydis=theorydis[dis_i];
        float temptheoryz=0;
        bool flag_matchtheory=false;
        // if(verticalangle < verticalangle_thresh)
        {

            float disminold=100;
            float dismaxold=-100;
            int disminposold=0;
            int dismaxposold=0;
            for(int j = 0 ; j < col_count - cloud_window_size; j+=2)
            {
                int ori_index = (j) * LASER_LAYER + i;
                if(velodyne_pointcloud->points[ori_index].y<4&&velodyne_pointcloud->points[ori_index].y>-4&&velodyne_pointcloud->points[ori_index].x>-2.5&&velodyne_pointcloud->points[ori_index].x<2.5) //back exist false detection , so need delete it
                    continue;

                if(velodyne_pointcloud->points[ori_index].y< -20||velodyne_pointcloud->points[ori_index].x< -30||velodyne_pointcloud->points[ori_index].x > 30) //back exist false detection , so need delete it
                    continue;



                float dismin=100;
                float dismax=-100;
                int disminpos=0;
                int dismaxpos=0;
                int window_maxz=-100;
                int window_minz=100;
                //get min max
                for(int window_j=0;window_j<cloud_window_size;window_j+=1)
                {
                    int index = (j+ window_j) * LASER_LAYER + i;
                    if(velodyne_pointcloud->points[index].y<4&&velodyne_pointcloud->points[index].y>-4&&velodyne_pointcloud->points[index].x>-2.5&&velodyne_pointcloud->points[index].x<2.5) //back exist false detection , so need delete it
                        continue;
                    float temprange=velodyne_pointcloud->points[index].range;
                    float tempz=velodyne_pointcloud->points[index].z;
                    if(temprange < 0.5)
                        continue;

                    if(temprange<dismin)
                    {
                        dismin=temprange;
                        disminpos=window_j;
                    }
                    if(temprange>dismax)
                    {
                        dismax=temprange;
                        dismaxpos=window_j;
                    }

                    if(tempz<window_minz)
                    {
                        window_minz=tempz;
                    }

                    if(tempz>window_maxz)
                    {
                        window_maxz=tempz;
                    }

                }
                //count num
                if(dismax-dismin>delta_dis_thresh&&dismax-dismin>delta_dis_thresh*0.1*dismin||dismax-dismin>delta_dis_thresh*3)
                {
                    int mincounter=0;
                    int maxcounter=0;
                    int zerocounter=0;
                    for(int window_j=0;window_j<cloud_window_size;window_j++)
                    {
                        int index = (j+ window_j) * LASER_LAYER + i;
                        if(velodyne_pointcloud->points[index].y<4&&velodyne_pointcloud->points[index].y>-4&&velodyne_pointcloud->points[index].x>-2.5&&velodyne_pointcloud->points[index].x<2.5) //back exist false detection , so need delete it
                        {
                            zerocounter++;
                            continue;
                        }
                        float temprange=velodyne_pointcloud->points[index].range;
                        if(temprange < 0.5)
                        {
                            zerocounter++;
                            continue;

                        }

                        if(temprange-dismin<delta_dis_thresh2)
                        {
                            mincounter++;
                        }
                        if(dismax-temprange<delta_dis_thresh2)
                        {
                            maxcounter++;
                        }



                    }

                    if(mincounter>=max_min_counter_thresh&&maxcounter>=max_min_counter_thresh&&zerocounter<max_min_counter_thresh*2)
                    {
                        for(int window_j=0;window_j<cloud_window_size;window_j++)
                        {
                            int index = (j+ window_j) * LASER_LAYER + i;
                            float temprange=velodyne_pointcloud->points[index].range;
                            if(temprange-dismin<delta_dis_thresh2)
                            {
#ifdef USE_OMP
                                omp_set_lock(&omplock); //获得互斥器
#endif
                                velodyne_pointcloud->points[index].passibility = 0.0;
#ifdef USE_OMP
                                omp_unset_lock(&omplock); //释放互斥器
#endif
                            }
                        }
                    }
                }

                disminold=dismin;
                dismaxold=dismax;
                disminposold=disminpos+j;
                dismaxposold=dismaxpos+j;

            }
        }
    }

}


void LidarProcess::ogmProcess()
{
    //build ogm

    if(1){
        //
        int nopassible_count_thresh = 2;// adjustable param

        memset(hdl_ogm_data.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(char));
        memset(slope_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(circle_slope_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(stiff_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));

        memset(passible_point_count_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(nopassible_point_count_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(slope_point_count_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(circle_slope_point_count_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));
        memset(stiff_point_count_ogm.ogm, 0, hdl_ogm_data.ogmcell_size * sizeof(int));

        float ogm_y_offset = 20.0f;//

        for (int i = 0; i < velodyne_pointcloud->points.size(); i++){
            float x = velodyne_pointcloud->points[i].x,
                  y = velodyne_pointcloud->points[i].y,
                  z = velodyne_pointcloud->points[i].z;

            float newy = y + ogm_y_offset;//ogm_y_offset

            if((x >=-hdl_ogm_data.ogmwidth / 2  && x <= hdl_ogm_data.ogmwidth / 2) &&
                    (newy >=0 && newy < hdl_ogm_data.ogmheight) &&
                    (z >= - 2 && z <=  Z_MAX)){

                int col = boost::math::round(x / hdl_ogm_data.ogmresolution) + ( hdl_ogm_data.ogmwidth_cell - 1 ) / 2;
                int row = boost::math::round(newy / hdl_ogm_data.ogmresolution) ;

                if((row >=0 && row < hdl_ogm_data.ogmheight_cell)
                        && (col >=0 && col < hdl_ogm_data.ogmwidth_cell)){
                    int index = row * hdl_ogm_data.ogmwidth_cell + col;
                    if(fabs(velodyne_pointcloud->points[i].passibility) < 0.5)
                        nopassible_point_count_ogm.ogm[index] ++;
                    else if(fabs(velodyne_pointcloud->points[i].passibility) <0.7 )
                    {
                        slope_point_count_ogm.ogm[index]++;
                    }
                    else if(fabs(velodyne_pointcloud->points[i].passibility) < 0.9)
                    {
                        if(velodyne_pointcloud->points[i].passibility > 0)
                            circle_slope_point_count_ogm.ogm[index]++;
                        else
                            circle_slope_point_count_ogm.ogm[index]--;
                        //                                                      if(fabs(velodyne_pointcloud->points[i].passibility) < 0.7)
                        //                                                         slope_point_count_ogm.ogm[index]++;
                    }
                    else
                        passible_point_count_ogm.ogm[index] ++;

                }
            }
        }


        for(int i = 0 ; i < hdl_ogm_data.ogmcell_size ; i++)
        {
            if(nopassible_point_count_ogm.ogm[i] > nopassible_count_thresh )
            {
                hdl_ogm_data.ogm[i] = RIGIDNOPASSABLE;
            }

            if(slope_point_count_ogm.ogm[i] > 0.6*(passible_point_count_ogm.ogm[i] ))
            {
                slope_ogm.ogm[i]=1;
            }

            if(abs(circle_slope_point_count_ogm.ogm[i])  > 0.3*passible_point_count_ogm.ogm[i])
            {
                if(circle_slope_point_count_ogm.ogm[i]>0)
                    circle_slope_ogm.ogm[i]=1;
                else
                    circle_slope_ogm.ogm[i]=-1;
            }

            if(passible_point_count_ogm.ogm[i] + nopassible_point_count_ogm.ogm[i]+slope_point_count_ogm.ogm[i] <2)
            {
                stiff_ogm.ogm[i]=1;
            }
        }
    }

    //                            mytime.stop();
    //                            mytime.show_s();

    //slope ogm
    if(1)
    {
        int window_halfnum=1;
        int num_thresh=5;
        for(int j=window_halfnum;j<hdl_ogm_data.ogmheight_cell-window_halfnum;j++)
        {
            for(int i=window_halfnum;i<hdl_ogm_data.ogmwidth_cell-window_halfnum;i++)
            {
                int counter=0;
                for(int window_j=j-window_halfnum ; window_j<=j+window_halfnum ; window_j++)
                {
                    for(int window_i=i-window_halfnum ; window_i<=i+window_halfnum ; window_i++)
                    {
                        counter+=slope_ogm.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell];
                    }
                }
                if(counter>num_thresh) //?
                {
                    for(int window_j=j-1 ; window_j<=j+1 ; window_j++)
                    {
                        for(int window_i=i-1 ; window_i<=i+1 ; window_i++)
                        {
                            hdl_ogm_data.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell]=RIGIDNOPASSABLE;
                        }
                    }
                }
            }
        }
    }

    //negative slope ogm
    if(1)
    {
        int window_halfnum=4;
        int num_thresh = 40;//16
        for(int j=window_halfnum;j<hdl_ogm_data.ogmheight_cell-window_halfnum;j++)
        {
            for(int i=window_halfnum;i<hdl_ogm_data.ogmwidth_cell-window_halfnum;i++)
            {
                int counter=0;
                for(int window_j=j-window_halfnum ; window_j<=j+window_halfnum ; window_j++)
                {
                    for(int window_i=i-window_halfnum ; window_i<=i+window_halfnum ; window_i++)
                    {
                        counter+=circle_slope_ogm.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell];
                    }
                }
                if(abs(counter)>num_thresh)
                {
                    for(int window_j=j-1 ; window_j<=j+1 ; window_j++)
                    {
                        for(int window_i=i-1 ; window_i<=i+1 ; window_i++)
                        {
                            hdl_ogm_data.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell]=RIGIDNOPASSABLE;
                        }
                    }
                }
            }
        }
    }

    //stiff ogm

    if(0)
    {
        int window_halfnum=10;
        int num_thresh = 440;
        for(int j=window_halfnum;j<hdl_ogm_data.ogmheight_cell/2-window_halfnum;j+=3)
        {
            for(int i=window_halfnum;i<hdl_ogm_data.ogmwidth_cell-window_halfnum;i+=3)
            {
                if((i-hdl_ogm_data.ogmwidth_cell/2)*(i-hdl_ogm_data.ogmwidth_cell/2)+(j-100)*(j-100)<600)
                    continue;
                if(i>hdl_ogm_data.ogmwidth_cell/2-25&&j<100&&i<hdl_ogm_data.ogmwidth_cell/2+25&&j>40)
                    continue;
                int counter=0;
                for(int window_j=j-window_halfnum ; window_j<=j+window_halfnum ; window_j++)
                {
                    for(int window_i=i-window_halfnum ; window_i<=i+window_halfnum ; window_i++)
                    {
                        counter+=stiff_ogm.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell];
                    }
                }
                if(counter>num_thresh)
                {
                    for(int window_j=j-window_halfnum/2 ; window_j<=j+window_halfnum/2 ; window_j++)
                    {
                        for(int window_i=i-window_halfnum/2 ; window_i<=i+window_halfnum /2; window_i++)
                        {
                            hdl_ogm_data.ogm[window_i+window_j*hdl_ogm_data.ogmwidth_cell]=RIGIDNOPASSABLE;
                        }
                    }
                }
            }
        }
    }

    if(1)
    {
        int index=0;
        for(int i=0; i<refineogm.ogmcell_size;i++,index++)
        {
            if(refineogm.ogm[i]==RIGIDNOPASSABLE)
                hdl_ogm_data.ogm[index]=refineogm.ogm[i];
            refineogm.ogm[i]=hdl_ogm_data.ogm[index];
        }
        for(int i=0; i<farrefinedogm.ogmcell_size ;i++,index++)
        {
            if(farrefinedogm.ogm[i]==RIGIDNOPASSABLE)
                hdl_ogm_data.ogm[index]=farrefinedogm.ogm[i];
            farrefinedogm.ogm[i]=hdl_ogm_data.ogm[index];
        }
    }
    hdl_ogm_data.updateStamp();
}

void LidarProcess::haarGradient(const OGMData<float>& minzogm,OGMData<float>& slopexogm, OGMData<float>& slopeyogm , int window_halfnum, float slopeminthreshold ,float heightdiffminthreshold)
{

    for(int i = 0 ; i < slopeyogm.ogmcell_size ; i++)
        slopeyogm.ogm[i] = 1000.0f;
    {
        for(int j=window_halfnum;j<minzogm.ogmheight_cell-window_halfnum;j++)
        {
            for(int i=window_halfnum;i<minzogm.ogmwidth_cell-window_halfnum;i++)
            {
                int upcounter=0;
                int downcounter=0;
                double upzsum = 0;
                double downzsum = 0;
                float upz = 0;
                float downz = 0;
                float heightdiff = 0;
                float slopeval = 0;
                for(int window_i=i-window_halfnum ; window_i<=i+window_halfnum ; window_i++)
                {
                    for(int window_j=j-window_halfnum ; window_j<=j ; window_j++)
                    {
                        float val = minzogm.ogm[window_i + window_j*minzogm.ogmwidth_cell];
                        unsigned char val1 = hdl_ogm_data.ogm[window_i + window_j*hdl_ogm_data.ogmwidth_cell];
                        if(val<500 && val1 !=RIGIDNOPASSABLE)
                        {
                            downzsum +=val;
                            downcounter+=1;

                        }
                    }
                }
                if(downcounter<1)
                    continue;
                for(int window_i=i-window_halfnum ; window_i<=i+window_halfnum ; window_i++)
                {
                    for(int window_j=j; window_j<=j + window_halfnum ; window_j++)
                    {
                        float val = minzogm.ogm[window_i + window_j*minzogm.ogmwidth_cell];
                        unsigned char val1 = hdl_ogm_data.ogm[window_i + window_j*hdl_ogm_data.ogmwidth_cell];
                        if(val<500 && val1 !=RIGIDNOPASSABLE)
                        {
                            upzsum +=val;
                            upcounter+=1;

                        }
                    }
                }
                if(upcounter<1)
                    continue;
                upz = upzsum/upcounter;
                downz = downzsum/downcounter;

                heightdiff = downz - upz  ;
                slopeval = heightdiff/(window_halfnum*minzogm.ogmresolution);
                if(fabs(heightdiff) > heightdiffminthreshold && fabs(slopeval) > slopeminthreshold) //?
                {
                    slopeyogm.ogm[i + j * slopeyogm.ogmwidth_cell] = slopeval;

                }

            }
        }
    }


    for(int i = 0 ; i < slopexogm.ogmcell_size ; i++)
        slopexogm.ogm[i] = 1000.0f;

    //slope ogm
    if(1)
    {
        for(int j=window_halfnum;j<minzogm.ogmheight_cell-window_halfnum;j++)
        {
            for(int i=window_halfnum;i<minzogm.ogmwidth_cell-window_halfnum;i++)
            {
                int leftcounter=0;
                int rightcounter=0;
                double leftzsum = 0;
                double rightzsum = 0;
                float leftz = 0;
                float rightz = 0;
                float heightdiff = 0;
                float slopeval = 0;
                for(int window_j=j-window_halfnum ; window_j<=j+window_halfnum ; window_j++)
                {
                    for(int window_i=i-window_halfnum ; window_i<i ; window_i++)
                    {
                        float val = minzogm.ogm[window_i + window_j*minzogm.ogmwidth_cell];
                        unsigned char val1 = hdl_ogm_data.ogm[window_i + window_j*hdl_ogm_data.ogmwidth_cell];
                        if(val<500 && val1 !=RIGIDNOPASSABLE)
                        {
                            leftzsum +=val;
                            leftcounter+=1;

                        }
                    }
                }
                if(leftcounter<1)
                    continue;
                for(int window_j=j-window_halfnum ; window_j<=j+window_halfnum ; window_j++)
                {
                    for(int window_i=i+1; window_i<=i + window_halfnum ; window_i++)
                    {
                        float val = minzogm.ogm[window_i + window_j*minzogm.ogmwidth_cell];
                        unsigned char val1 = hdl_ogm_data.ogm[window_i + window_j*hdl_ogm_data.ogmwidth_cell];
                        if(val<500 && val1 !=RIGIDNOPASSABLE)
                        {
                            rightzsum +=val;
                            rightcounter+=1;

                        }
                    }
                }
                if(rightcounter<1)
                    continue;
                leftz = leftzsum/leftcounter;
                rightz = rightzsum/rightcounter;

                heightdiff = rightz - leftz;
                if(i<minzogm.ogmwidth_cell/2)
                    heightdiff = -heightdiff;  //reverse the gradient's direction for left point of ogm map
                slopeval = heightdiff/(window_halfnum*minzogm.ogmresolution);
                if(fabs(heightdiff) > heightdiffminthreshold && fabs(slopeval) > slopeminthreshold) //?
                {
                    slopexogm.ogm[i + j * slopexogm.ogmwidth_cell] = slopeval;

                }

            }
        }

    }

}

    template<class T>
void LidarProcess::showHaarOGM(const char* windowname ,const OGMData<T>& ogmdata)
{
    cvNamedWindow(windowname,0);
    CvMat *slopemat = cvCreateMat(ogmdata.ogmheight_cell,ogmdata.ogmwidth_cell,CV_8UC3);
    cvZero(slopemat);
    for(int j=0;j<ogmdata.ogmheight_cell;j++)
    {
        unsigned char* pdata = (unsigned char*)(slopemat->data.ptr + (ogmdata.ogmheight_cell - 1 - j)* slopemat->step);
        for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
        {
            float val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];
            if(val<500)
            {
                if(val<=-1)
                    pdata[3*i+2] = 0;
                else if(val >= 1)
                    pdata[3*i+2] = 250;
                else
                    pdata[3*i+2] =static_cast<unsigned char>((val - (-1))*250/(1 - (-1)));

            }
            else
            {
                pdata[3*i+1] = 125;
            }
        }

    }
    cvShowImage(windowname,slopemat);
    cvWaitKey(1);

    cvReleaseMat(&slopemat);

}

    template<class T>
void LidarProcess::showOGM(const char* windowname ,const OGMData<T>& ogmdata)
{
//    cvNamedWindow(windowname,0);
    CvMat *slopemat = cvCreateMat(ogmdata.ogmheight_cell,ogmdata.ogmwidth_cell,CV_8UC3);
    cvZero(slopemat);
    int heightnum = ogmdata.ogmheight_cell/(10/ogmdata.ogmresolution);
    int widthnum = ogmdata.ogmwidth_cell/(10/ogmdata.ogmresolution);
    for(int i = 0; i<heightnum ;i++)
      {
        cvLine(slopemat,cvPoint(0,slopemat->height*i/heightnum),
                cvPoint(slopemat->width-1,slopemat->height*i/heightnum),cvScalar(255,0,0));
      }

    for(int i=1;i<widthnum;i++)
    {
        cvLine(slopemat,cvPoint(slopemat->width*i/widthnum,0),
                cvPoint(slopemat->width*i/widthnum,slopemat->height-1),cvScalar(255,0,0));
    }

    float ogmresolution = ogmdata.ogmresolution;

    cvRectangle(slopemat,cvPoint(slopemat->width/2-1/ogmresolution,
				 slopemat->height-(20+2)/ogmresolution),
            cvPoint(slopemat->width/2+1/ogmresolution,slopemat->height-(20-2)/ogmresolution),
            cvScalar(0,255,0));
    for(int j=0;j<ogmdata.ogmheight_cell;j++)
    {
        unsigned char* pdata = (unsigned char*)(slopemat->data.ptr + (ogmdata.ogmheight_cell - 1 - j)* slopemat->step);
        for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
        {
            T val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];
            if(val > 0)
            {

        	T temp;
        	if(val == RIGIDNOPASSABLE)
        	  {
        	    temp = 255;
                    pdata[3*i] = temp;
                    pdata[3*i+1] = temp;
                    pdata[3*i+2] = temp;
        	  }
//                    if(pdata[i]>250)
//                      pdata[i] = 250;
//                    if(pdata[i]<0)
//                      pdata[i] = 0;


            }

        }

    }
    cvShowImage(windowname,slopemat);
    cvReleaseMat(&slopemat);

}
void LidarProcess::haarProcess()
{
    if(1)
    {
        cvNamedWindow("minogmmat",0);
        int window_halfnum=7;
        float heightdiffminthreshold = 0.2;
        float slopeminthreshold = 0.2;
        float minlimit = -2;
        float maxlimit = 3;

        haarGradient(minzrefineogm,slope5xogm,slope5yogm,window_halfnum,slopeminthreshold);

        showHaarOGM("slope5xogm",slope5xogm);
        showHaarOGM("slope5yogm",slope5yogm);

//        haarGradient(minzrefineogm,slope2xogm,slope2yogm,2,0.3);

//        showOGM("slope2xogm",slope2xogm);
//        showOGM("slope2yogm",slope2yogm);



        CvMat *minogmmat = cvCreateMat(minzrefineogm.ogmheight_cell,minzrefineogm.ogmwidth_cell,CV_8UC1);
        cvZero(minogmmat);
        for(int j=0;j<minzrefineogm.ogmheight_cell;j++)
        {
            unsigned char* pdata = (unsigned char*)(minogmmat->data.ptr + (minzrefineogm.ogmheight_cell - 1 - j)* minogmmat->step);
            for(int i=0 ;i < minzrefineogm.ogmwidth_cell ; i++)
            {
                float val = minzrefineogm.ogm[i + j*minzrefineogm.ogmwidth_cell];
                if(val<500)
                {
                    if(val<=minlimit)
                        pdata[i] = 0;
                    else if(val >= maxlimit)
                        pdata[i] = 250;
                    else
                        pdata[i] =static_cast<unsigned char>((val - minlimit)*250/(maxlimit - minlimit));

                }
            }

        }
        cvShowImage("minogmmat",minogmmat);
        //                        cvWaitKey(10);
        cvReleaseMat(&minogmmat);
    }


}
/*
int LidarProcess::getMainRoadPoints(std::vector<POINT_2F>& leftpos_vec , std::vector<POINT_2F>& rightpos_vec)
{
    if(velodyne_pointcloud->points.size())
    {
        float azimuthfront = 90;
        float azimuthback = 270;
        int angleindexfront = azimuthfront * 10;
        int angleindexback = azimuthback * 10;
        bool left_found = false;
        bool right_found = false;
        POINT_2F left_point[2];
        POINT_2F right_point[2];
        for(int j = 0;j<LASER_LAYER*2/3;j++)
        {
            for()
        }
    }
}
*/
int LidarProcess::process()
{
    int successed = computepointcloud();
    if(!successed)
        return 0;

    pointCloudProcess();
    ogmProcess();
    //haarProcess();

    boost::mutex::scoped_lock lock(displaymutex);
    if(1)
        splitPointCloud();
    showOGM("ogm",hdl_ogm_data);
    buildIntegral();
    getCloudSections();
    getRoadPath();
    bool gotroad = roadboundaries.getRoadBoundary(roadpaths,cloudsections,90);
    //DrawBoundary();
    //std::cout<<"display:"<<display_<<std::endl;
    if(gotroad)
      return 2;
    else
      return 1;
}
void LidarProcess::getRoadEdgePoints(Cloud& roadedgepoints,Cloud& roadedgepoints_less,int mode)
{
  std::vector<std::vector<int> > sectionmask;
  sectionmask.reserve(cloudsections.size());
  for(int j=0 ; j<cloudsections.size();j++)
  {
      std::vector<int> tempmask(cloudsections[j].sections.size(),0);
      sectionmask.push_back(tempmask);
  }

  for(std::vector<RoadPath>::iterator it_path=roadpaths.paths.begin() ; it_path!=roadpaths.paths.end();it_path++)
         {
             RoadPath path(*it_path);
             {

                 for(std::vector<RoadNode>::iterator it=path.nodes.begin();it!=path.nodes.end();it++)
                 {
                     int j = it-path.nodes.begin();

                     sectionmask.at(j).at(it->index) = 1;


                 }

             }
         }

  for(int j=0 ; j<cloudsections.size();j++)
  {
    PointXYZI startpoint;
    if(cloudsections[j].sections.size()==0)
      continue;
    else if(cloudsections[j].sections.size()==1)
      {
	if(cloudsections[j].sections[0].anglerange>358)
	  continue;
	startpoint.x=cloudsections[j].sections[0].minpoint.x;
	startpoint.y=cloudsections[j].sections[0].minpoint.y;
	startpoint.z=cloudsections[j].sections[0].minpoint.height;
	startpoint.azimuth=cloudsections[j].sections[0].minpoint.angle;
	startpoint.intensity=j;
	PointXYZI stoppoint;
	stoppoint.x=cloudsections[j].sections[0].maxpoint.x;
	stoppoint.y=cloudsections[j].sections[0].maxpoint.y;
	stoppoint.z=cloudsections[j].sections[0].maxpoint.height;
	stoppoint.azimuth=cloudsections[j].sections[0].maxpoint.angle;
	stoppoint.intensity=j;
	roadedgepoints_less.push_back(startpoint);
	roadedgepoints_less.push_back(stoppoint);
      }
    else
      {

	float startangle=0;
	float lastangle = 0;


	bool firstflag=true;
	for(std::vector<CloudPointSection>::iterator it=cloudsections[j].sections.begin();it!=cloudsections[j].sections.end();it++)
	  {
	    int i = it - cloudsections[j].sections.begin();
	    if(sectionmask.at(j).at(i)==0)
	      continue;
	    if(firstflag)
	      {
		startpoint.x=cloudsections[j].sections[0].minpoint.x;
		startpoint.y=cloudsections[j].sections[0].minpoint.y;
		startpoint.z=cloudsections[j].sections[0].minpoint.height;
		startpoint.azimuth=cloudsections[j].sections[0].minpoint.angle;
		startpoint.intensity=j;
		PointXYZI stoppoint;
		stoppoint.x=cloudsections[j].sections[0].maxpoint.x;
		stoppoint.y=cloudsections[j].sections[0].maxpoint.y;
		stoppoint.z=cloudsections[j].sections[0].maxpoint.height;
		stoppoint.azimuth=cloudsections[j].sections[0].maxpoint.angle;
		stoppoint.intensity=j;
		startangle=cloudsections[j].sections[0].minpoint.angle;
		lastangle=cloudsections[j].sections[0].maxpoint.angle;
		roadedgepoints_less.push_back(startpoint);
		roadedgepoints_less.push_back(stoppoint);
		firstflag = false;
	      }
	    PointXYZI temppoint;
	    if(fabs(it->minpoint.angle-lastangle)>1)
	    {
	      temppoint.x=it->minpoint.x;
	      temppoint.y=it->minpoint.y;
	      temppoint.z=it->minpoint.height;
	      temppoint.azimuth=it->minpoint.angle;
	      temppoint.intensity=j;
	      roadedgepoints_less.push_back(temppoint);
	    }
	    lastangle = it->minpoint.angle;
	    if(fabs(it->maxpoint.angle-lastangle)>1 && fabs(it->maxpoint.angle-startangle)>1 )
	    {
	      temppoint.x=it->maxpoint.x;
	      temppoint.y=it->maxpoint.y;
	      temppoint.z=it->maxpoint.height;
	      temppoint.azimuth=it->maxpoint.angle;
	      temppoint.intensity=j;
	      roadedgepoints_less.push_back(temppoint);
	    }
	    lastangle = it->maxpoint.angle;

	  }
      }
  }

  if(mode==2)
  {
      std::vector<POINT_2F>& templeftpoints = roadboundaries.mainroad_leftboundary.points;
      for(std::vector<POINT_2F>::iterator it = templeftpoints.begin();it!=templeftpoints.end();it++)
	{
	  PointXYZI temppoint;
	  temppoint.x=it->x;
	  temppoint.y=it->y;
	  temppoint.z=it->height;
	  temppoint.azimuth=it->angle;
	  temppoint.intensity=it->layer;
	  roadedgepoints.push_back(temppoint);
	}

      std::vector<POINT_2F>& temprightpoints = roadboundaries.mainroad_rightboundary.points;
      for(std::vector<POINT_2F>::iterator it = temprightpoints.begin();it!=temprightpoints.end();it++)
	{
	  PointXYZI temppoint;
	  temppoint.x=it->x;
	  temppoint.y=it->y;
	  temppoint.z=it->height;
	  temppoint.azimuth=it->angle;
	  temppoint.intensity=it->layer;
	  roadedgepoints.push_back(temppoint);
	}

  }

}

void LidarProcess::buildIntegral()
{
    memset(passable_count_integral,0,sizeof(int)*LASER_LAYER*3601);
    memset(passable_height_integral,0,sizeof(double)*LASER_LAYER*3601);
    memset(passable_distance_integral,0,sizeof(double)*LASER_LAYER*3601);

    int col_count = velodyne_pointcloud->size() / LASER_LAYER;
    for(int j = 0;j<LASER_LAYER ;j++)
    {
        int laser_num = grabber_H_.indexmaptable[j].number;
        for(int mat_i=0;mat_i<3601;mat_i++)
        {
            int countadded = 0;
            double heightadded = 0.0;
            double distanceadded = 0.0;

            PolarPointDI temppolarpoint = polaraxismat_[laser_num][mat_i];
            if(temppolarpoint.distance<0.1)
            {
                if(mat_i==0)
                {
                    passable_count_integral[laser_num][mat_i]=countadded;
                    passable_height_integral[laser_num][mat_i]=heightadded;
                    passable_distance_integral[laser_num][mat_i]=distanceadded;
                }
                else
                {
                    passable_count_integral[laser_num][mat_i]=passable_count_integral[laser_num][mat_i-1]+countadded;
                    passable_height_integral[laser_num][mat_i]=passable_height_integral[laser_num][mat_i-1]+heightadded;
                    passable_distance_integral[laser_num][mat_i]=passable_distance_integral[laser_num][mat_i-1]+distanceadded;
                }

                continue;
            }
            int index = temppolarpoint.index;
            pcl::PointXYZI temppoint = velodyne_pointcloud->points[index];
            if(temppoint.passibility >= 1.0)
            {
                countadded = 1;
                double height = temppoint.z;
                if(j<5&&height>0)
                    height = 0;
                heightadded = height;
                distanceadded = temppoint.range;
//                std::cout<<"height:"<<heightadded<<std::endl;
            }
                if(mat_i==0)
                {
                    passable_count_integral[laser_num][mat_i]=countadded;
                    passable_height_integral[laser_num][mat_i]=heightadded;
                    passable_distance_integral[laser_num][mat_i]=distanceadded;
                }
                else
                {
                    passable_count_integral[laser_num][mat_i]=passable_count_integral[laser_num][mat_i-1]+countadded;
                    passable_height_integral[laser_num][mat_i]=passable_height_integral[laser_num][mat_i-1]+heightadded;
                    passable_distance_integral[laser_num][mat_i]=passable_distance_integral[laser_num][mat_i-1]+distanceadded;
                }


        }
    }
}

void LidarProcess::getCloudSections()
{
    int layer = 0;
    int col_count = velodyne_pointcloud->size() / LASER_LAYER;
    int layercut=6;
    for(int j=0 ; j<LASER_LAYER *2/3 ;j++)
    {
        int laser_num = grabber_H_.indexmaptable[j].number;
        if(theorydis[j]>32||theorydis[j]<=0)
            continue;
        cloudsections[j].sections.clear();
        bool found_first_layer=false;
//        if(j==0)
        {
            float anglestart =0;
            float anglestop = 360;
            int totalstart =anglestart *10;
            int totalstop = anglestop*10;
            int start = totalstart;
            int stop = totalstop;
            int num =0;
            bool startflag=false;
            bool stopflag=false;
            POINT_2F point_start;
            POINT_2F point_stop;
            point_start.distance = 0;
            point_stop.distance = 0;
            CloudPointSection tempsection;
			tempsection.passablepointnum=0;
            int last_num_stop=totalstart;
            int last_num_start=totalstop;
            int starti=totalstop,stopi=totalstart;
            double heightsum = 0;
            double distancesum = 0;
            int countsum = 0;

            for( ; starti > stopi ;)
            {

                PolarPointDI temppolarpoint_start = polaraxismat_[laser_num][starti];
                PolarPointDI temppolarpoint_stop = polaraxismat_[laser_num][stopi];

                if(!startflag&&fabs(temppolarpoint_start.distance)>0.15)
                {

                    int index = temppolarpoint_start.index;
                    pcl::PointXYZI temppoint = velodyne_pointcloud->points[index];
                    if(abs(starti - last_num_start)>anglerange[j]*10 && temppoint.range>-0.15)
                    {
                        startflag = true;
                        point_start.weight=0.2;
                    }
                    else if(temppoint.passibility<0.5)
                    {


                        if(fabs(temppoint.range - point_start.distance)<0.5 && fabs(temppoint.z - point_start.height) < 0.2 )
                        {
                            starti-- ;
                            last_num_start = starti;
                           // point_start.height = temppoint.z;
                        }
                        else
                        {
                            startflag = true;
                        }
                        if(!startflag||temppoint.range < point_start.distance)
                          {
			    point_start.x = temppoint.x;
			    point_start.y = temppoint.y;
			    point_start.angle = temppoint.azimuth;
			    point_start.distance = temppoint.range;
                          }

                            if(fabs(temppoint.passibility)<0.01)
                                point_start.weight = 1.0;
                            else
                                point_start.weight = 0.5;
                    }
                    else
                    {
                        if(j<layercut && temppoint.z>0)
                            temppoint.z = 0;

                        if(temppoint.range > 0.1 && point_start.distance>0.1&&fabs(temppoint.z-point_start.height)>0.5)
                        {
 //                           stopflag=true;
                            //std::cout<<"last point:"<<point_start.x<<","<<point_start.y<<"\t"<<"temppoint:"<<temppoint.x<<","<<temppoint.y<<" heightdiff over!!!"<<std::endl;
                        }
                        else if(temppoint.range < -0.15)
                          {
                            last_num_start = starti;
                          }
                        else
                        {
			  point_start.x = temppoint.x;
			  point_start.y = temppoint.y;
			  point_start.angle = temppoint.azimuth;
			  point_start.distance = temppoint.range;

			  point_start.height = temppoint.z;
			  last_num_start = starti;
                        }
                        starti--;
                    }

                }
                else if(!startflag)
                {
                    starti--;
                }

                if(startflag&&point_start.distance<=0.1)
                  {
                    stopflag = true;
                    break;
                  }

                if(point_start.distance > 0.1)
                  {

		    if(!stopflag&&fabs(temppolarpoint_stop.distance)>0.15)
		    {

			int index = temppolarpoint_stop.index;
			pcl::PointXYZI temppoint = velodyne_pointcloud->points[index];
			if(abs(stopi - last_num_stop)>anglerange[j]*10 && temppoint.range>-0.15)
			{
			    stopflag = true;
			    point_stop.weight=0.2;
			}
			else if(temppoint.passibility<0.5)
			{

			    if(fabs(temppoint.range - point_stop.distance)<0.5 && fabs(temppoint.z - point_stop.height) < 0.2 )
			    {
				stopi++ ;
				last_num_stop = stopi;
				//point_stop.height = temppoint.z;
			    }
			    else
			    {
				stopflag = true;
			    }
    //                        if(temppoint.x>-6 && temppoint.x<0)
    //                          std::cout<<stopflag<<" "<<j<<" "<<temppoint.range<<" " <<temppoint.azimuth
    //			  <<" "<<point_stop.distance<<" "<<point_stop.angle<<std::endl;
			    if(!stopflag||temppoint.range < point_stop.distance)
			      {
    //                            if(temppoint.x>-6 && temppoint.x<0)
    //                              std::cout<<stopflag<<" "<<j<<" "<<temppoint.range<<" " <<temppoint.azimuth
    //    			  <<" "<<point_stop.distance<<" "<<point_stop.angle<<std::endl;
				point_stop.x = temppoint.x;
				point_stop.y = temppoint.y;
				point_stop.angle = temppoint.azimuth;
				point_stop.distance = temppoint.range;
			      }


				if(fabs(temppoint.passibility)<0.01)
				    point_stop.weight = 1.0;
				else
				    point_stop.weight = 0.5;
			}
			else
			{
			    if(j<layercut && temppoint.z>0)
				temppoint.z = 0;

			    if(temppoint.range > 0.1 && point_stop.distance>0.1&&fabs(temppoint.z-point_stop.height)>0.5)
			    {
     //                           stopflag=true;
				//std::cout<<"last point:"<<point_stop.x<<","<<point_stop.y<<"\t"<<"temppoint:"<<temppoint.x<<","<<temppoint.y<<" heightdiff over!!!"<<std::endl;
			    }
			    else if(temppoint.range < -0.15)
			      {
				last_num_stop = stopi;
			      }
			    else
			    {

			    point_stop.x = temppoint.x;
			    point_stop.y = temppoint.y;
			    point_stop.angle = temppoint.azimuth;
			    point_stop.distance = temppoint.range;

			    point_stop.height = temppoint.z;
			    last_num_stop = stopi;
			    }
			    stopi++;
			}

		    }
		    else if(!stopflag)
		    {
			stopi++;
		    }
                  }
                if(startflag&&stopflag)
                  {
                    break;
                  }
            }
            if(last_num_stop>0 &&last_num_start<3600)
            {
                int temp_num=last_num_stop+totalstop -last_num_start;
                float tempangle = temp_num /10.0;
                totalstart = last_num_stop;
                totalstop = last_num_start;
                if(tempangle > anglerange[j]&&point_start.distance>0.1&&point_stop.distance>0.1)
                {
                    tempsection.minpoint = point_start;
                    tempsection.maxpoint = point_stop;
                    tempsection.anglerange = point_start.angle>point_stop.angle?360-point_start.angle+point_stop.angle:point_stop.angle-point_start.angle;
                    tempsection.angle = tempsection.minpoint.angle>tempsection.maxpoint.angle
                           ?((tempsection.minpoint.angle+tempsection.maxpoint.angle)>360?(tempsection.minpoint.angle+tempsection.maxpoint.angle-360)/2:(tempsection.minpoint.angle+tempsection.maxpoint.angle+360)/2)
                           :(tempsection.minpoint.angle+tempsection.maxpoint.angle)/2;


//                    tempsection.distance = distancesum / tempsection.passablepointnum; // (point_start.distance + point_stop.distance)/2;
//                    tempsection.height = heightsum/ tempsection.passablepointnum;
                    double tempanglebegin = tempsection.minpoint.angle;
                    double tempangleend = tempsection.maxpoint.angle;
                    if(tempangleend>=tempanglebegin)
                    {
                        int mat_begin = tempanglebegin*10;
                        int mat_end = tempangleend*10;
                        countsum = passable_count_integral[laser_num][mat_end] - passable_count_integral[laser_num][mat_begin];
                        heightsum = passable_height_integral[laser_num][mat_end] - passable_height_integral[laser_num][mat_begin];
                        distancesum = passable_distance_integral[laser_num][mat_end] - passable_distance_integral[laser_num][mat_begin];
                    }
                    else
                    {
                        int mat_begin = tempanglebegin*10;
                        int mat_end = tempangleend*10;
                        countsum = passable_count_integral[laser_num][mat_end] + passable_count_integral[laser_num][3600] - passable_count_integral[laser_num][mat_begin];
                        heightsum = passable_height_integral[laser_num][mat_end] + passable_height_integral[laser_num][3600] - passable_height_integral[laser_num][mat_begin];
                        distancesum = passable_distance_integral[laser_num][mat_end] + passable_distance_integral[laser_num][3600] - passable_distance_integral[laser_num][mat_begin];
                    }

                    tempsection.passablepointnum = countsum;
                    tempsection.height = heightsum / countsum;
                    tempsection.distance = distancesum / countsum;

//                    if(j<5)
//                        tempsection.height = 0;
                    cloudsections[j].sections.push_back(tempsection);
//                    std::cout<<"1-"<<j<<"\tstarti"<<starti<<"\tstart:"<<point_start.angle<<"\tstopi"<<stopi<<"\tstop:"<<point_stop.angle<<"\theight:"<<tempsection.height<<"\tdistance:"<<tempsection.distance<<std::endl;
//                    std::cout<<point_start.x<<" "<<point_start.y<<" "<<point_start.angle<<" "<<last_num_start<<" "<<point_start.distance<<std::endl;
//                    std::cout<<point_stop.angle<<" "<<last_num_stop<<" "<<point_stop.distance<<std::endl;
                }


            }
                tempsection.passablepointnum = 0;
                heightsum = 0;
                distancesum = 0;
                countsum = 0;


            if(startflag&&stopflag)
            {
                int last_num = totalstart;
                bool stopflag =false;
                bool startflag = false;
                point_start = point_stop;
                memset(&point_stop ,0 ,sizeof(point_stop));

//                std::cout<<"totalstart:"<<totalstart<<"\ttotalstop"<<totalstop<<std::endl;
                float tempstartweight=0.2;
                for(int i = totalstart ;i <= totalstop ;i++ )
                {
                    PolarPointDI temppolarpoint = polaraxismat_[laser_num][i];
                    int index=0;
                    pcl::PointXYZI temppoint;
                    if(i != totalstop && fabs(temppolarpoint.distance)<=0.15)
                        continue;
                    else if(i != totalstop)
                    {
                        index = temppolarpoint.index;
                        temppoint = velodyne_pointcloud->points[index];
                    }
                    if(i == totalstop || abs(i - last_num)>anglerange[j]*10)
                    {
                        point_stop.weight=0.2;
                        if(startflag)
                            stopflag = true;
                        else
                            last_num = i;
                        tempstartweight = point_stop.weight;
                    }
                    else if(temppoint.passibility<0.5)
                    {


                        if(startflag &&fabs(temppoint.range- point_stop.distance)<0.5 && fabs(temppoint.z - point_stop.height) < 0.2)
                        {
                            last_num = i;
                            //point_stop.height = temppoint.z;
                        }
                        else
                        {
                            if(startflag)
                                stopflag = true;
                            else
                                last_num = i;
                        }
                        if(!stopflag||temppoint.range < point_stop.distance)
                          {
			    point_stop.x = temppoint.x;
			    point_stop.y = temppoint.y;
			    point_stop.angle = temppoint.azimuth;
			    point_stop.distance = temppoint.range;
                          }

                            if(fabs(temppoint.passibility)<0.01)
                                point_stop.weight = 1.0;
                            else
                                point_stop.weight = 0.5;
                            tempstartweight = point_stop.weight;


                    }
                    else
                    {
                        if(j<5 && temppoint.z>0)
                            temppoint.z = 0;

                        if(startflag&&temppoint.range > 0.1&&fabs(temppoint.z-point_stop.height)>0.5)
                        {
 //                           stopflag=true;
                            //std::cout<<"last point:"<<point_stop.x<<","<<point_stop.y<<"\t"<<"temppoint:"<<temppoint.x<<","<<temppoint.y<<" heightdiff over!!!"<<std::endl;
                        }
                        else if(temppoint.range < -0.1)
                          {
                            last_num = i;
                          }
                        else if(startflag)
                        {
                        point_stop.x = temppoint.x;
                        point_stop.y = temppoint.y;
                        point_stop.angle = temppoint.azimuth;
                        point_stop.distance = temppoint.range;

                        point_stop.height = temppoint.z;
                        last_num = i;
                        }
                        else if(!startflag)
                        {
                            if(temppoint.range < point_stop.distance||point_stop.distance<0.1)
                              {
				point_stop.x = temppoint.x;
				point_stop.y = temppoint.y;
				point_stop.angle = temppoint.azimuth;
				point_stop.distance = temppoint.range;
                              }
                            point_stop.height = temppoint.z;
                            last_num = i;
                            point_start = point_stop;
                            point_start.weight = tempstartweight;
                            startflag = true;
                        }

                    }

                    if(stopflag)
                    {
                        i = last_num;
                        stopflag = false;
                        startflag = false;
                        int temp_num = last_num-totalstart;
                        float tempangle = abs(temp_num) / 10.0;
                        totalstart = i;
                        last_num = i;
                        if(tempangle > anglerange[j]&&point_start.distance>0.1&&point_stop.distance>0.1)
                        {
                            tempsection.minpoint = point_start;
                            tempsection.maxpoint = point_stop;
                            tempsection.anglerange = point_start.angle>point_stop.angle?360-point_start.angle+point_stop.angle:point_stop.angle-point_start.angle;
                            tempsection.angle = tempsection.minpoint.angle>tempsection.maxpoint.angle
                               ?((tempsection.minpoint.angle+tempsection.maxpoint.angle)>360?(tempsection.minpoint.angle+tempsection.maxpoint.angle-360)/2:(tempsection.minpoint.angle+tempsection.maxpoint.angle+360)/2)
                               :(tempsection.minpoint.angle+tempsection.maxpoint.angle)/2;

                            double tempanglebegin = tempsection.minpoint.angle;
                            double tempangleend = tempsection.maxpoint.angle;
                            if(tempangleend>=tempanglebegin)
                            {
                                int mat_begin = tempanglebegin*10;
                                int mat_end = tempangleend*10;
                                countsum = passable_count_integral[laser_num][mat_end] - passable_count_integral[laser_num][mat_begin];
                                heightsum = passable_height_integral[laser_num][mat_end] - passable_height_integral[laser_num][mat_begin];
                                distancesum = passable_distance_integral[laser_num][mat_end] - passable_distance_integral[laser_num][mat_begin];
                            }
                            else
                            {
                                int mat_begin = tempanglebegin*10;
                                int mat_end = tempangleend*10;
                                countsum = passable_count_integral[laser_num][mat_end] + passable_count_integral[laser_num][3600] - passable_count_integral[laser_num][mat_begin];
                                heightsum = passable_height_integral[laser_num][mat_end] + passable_height_integral[laser_num][3600] - passable_height_integral[laser_num][mat_begin];
                                distancesum = passable_distance_integral[laser_num][mat_end] + passable_distance_integral[laser_num][3600] - passable_distance_integral[laser_num][mat_begin];
                            }

                            tempsection.passablepointnum = countsum;
                            tempsection.height = heightsum/ countsum;
                            tempsection.distance = distancesum/ countsum;

                                   // if(j<6)
                                   //     tempsection.height = 0;

                            cloudsections[j].sections.push_back(tempsection);
//                            std::cout<<"2-"<<j<<"start:"<<point_start.angle<<"\tstop:"<<point_stop.angle<<"\theight:"<<tempsection.height<<"\tdistance:"<<tempsection.distance<<std::endl;
                        }
                        tempsection.passablepointnum = 0;
                        heightsum = 0;
                        distancesum = 0;
                        countsum = 0;
                    }



                }
            }
        }

    }
}



//float LidarProcess::getSectionOverlap(const CloudPointSection& section1 , const CloudPointSection& section2 ,float& beginangle ,float& endangle)
float LidarProcess::getAngleOverlap(float s1anglebegin ,float s1angleend ,float s2anglebegin,float s2angleend ,float& beginangle ,float& endangle)
{
    bool startflag1 = false;
    bool startflag2 = false;
    bool stopflag1 = false;
    bool stopflag2 = false;
    if(s1anglebegin>s1angleend)
    {
        if((s2anglebegin<=360&&s2anglebegin>=s1anglebegin)||(s2anglebegin>=0&&s2anglebegin<=s1angleend))
        {
            startflag1 = true;
            beginangle = s2anglebegin;
        }
        if((s2angleend<=360&&s2angleend>=s1anglebegin)||(s2angleend>=0&&s2angleend<=s1angleend))
        {
            stopflag1 = true;
            endangle = s2angleend;
        }
    }
    else
    {
        if(s2anglebegin<=s1angleend&&s2anglebegin>=s1anglebegin)
        {
            startflag1=true;
            beginangle = s2anglebegin;
        }
        if(s2angleend<=s1angleend&&s2angleend>=s1anglebegin)
        {
            stopflag1=true;
            endangle = s2angleend;
        }

    }

    if(s2anglebegin>s2angleend)
    {
        if((s1anglebegin<=360&&s1anglebegin>s2anglebegin)||(s1anglebegin>=0&&s1anglebegin<s2angleend))
        {
            startflag2 = true;
            beginangle = s1anglebegin;
        }
        if((s1angleend<=360&&s1angleend>=s2anglebegin)||(s1angleend>=0&&s1angleend<s2angleend))
        {
            stopflag2 = true;
            endangle = s1angleend;
        }
    }
    else
    {
        if(s1anglebegin<s2angleend&&s1anglebegin>s2anglebegin)
        {
            startflag2=true;
            beginangle = s1anglebegin;
        }
        if(s1angleend<s2angleend&&s1angleend>=s2anglebegin)
        {
            stopflag2=true;
            endangle = s1angleend;
        }

    }
    if((startflag1&&startflag2)||(stopflag1&&stopflag2))
    {
        if(s1anglebegin > s1angleend &&s1anglebegin<181&&s1anglebegin>=179&&s1angleend<=181&&s1angleend>=179)
        {
            beginangle=s2anglebegin;
            endangle = s2angleend;
			return  endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;;
        }
        else if(s2anglebegin > s2angleend &&s2anglebegin<181&&s2anglebegin>=179&&s2angleend<=181&&s2angleend>=179)
        {
            beginangle=s1anglebegin;
            endangle = s1angleend;
			return   endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;;
        }

        float anglediff1= s2angleend>=s1anglebegin?s2angleend-s1anglebegin:360+s2angleend-s1anglebegin;
        float anglediff2= s1angleend>=s2anglebegin?s1angleend-s2anglebegin:360+s1angleend-s2anglebegin;
        if(anglediff1>anglediff2)
        {

            beginangle=s1anglebegin;
            endangle=s2angleend;
            return anglediff1;
        }
        else
        {
            beginangle = s2anglebegin;
            endangle = s1angleend;
            return anglediff2;
        }

    }
    if((stopflag1||stopflag2)&&(startflag1||startflag2))
        return endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;
    else
        return 0;

}

std::vector<float> LidarProcess::getAngleOverlap(float s1anglebegin ,float s1angleend ,float s2anglebegin,float s2angleend ,std::vector<float>& beginangle_vec ,std::vector<float>& endangle_vec)
{
	bool startflag1 = false;
	bool startflag2 = false;
	bool stopflag1 = false;
	bool stopflag2 = false;
	float beginangle ,endangle;
	std::vector<float> diffangle_vec;
	if(s1anglebegin>s1angleend)
	{
		if((s2anglebegin<=360&&s2anglebegin>=s1anglebegin)||(s2anglebegin>=0&&s2anglebegin<=s1angleend))
		{
			startflag1 = true;
			beginangle = s2anglebegin;
		}
		if((s2angleend<=360&&s2angleend>=s1anglebegin)||(s2angleend>=0&&s2angleend<=s1angleend))
		{
			stopflag1 = true;
			endangle = s2angleend;
		}
	}
	else
	{
		if(s2anglebegin<=s1angleend&&s2anglebegin>=s1anglebegin)
		{
			startflag1=true;
			beginangle = s2anglebegin;
		}
		if(s2angleend<=s1angleend&&s2angleend>=s1anglebegin)
		{
			stopflag1=true;
			endangle = s2angleend;
		}

	}

	if(s2anglebegin>s2angleend)
	{
		if((s1anglebegin<=360&&s1anglebegin>s2anglebegin)||(s1anglebegin>=0&&s1anglebegin<s2angleend))
		{
			startflag2 = true;
			beginangle = s1anglebegin;
		}
		if((s1angleend<=360&&s1angleend>=s2anglebegin)||(s1angleend>=0&&s1angleend<s2angleend))
		{
			stopflag2 = true;
			endangle = s1angleend;
		}
	}
	else
	{
		if(s1anglebegin<s2angleend&&s1anglebegin>s2anglebegin)
		{
			startflag2=true;
			beginangle = s1anglebegin;
		}
		if(s1angleend<s2angleend&&s1angleend>=s2anglebegin)
		{
			stopflag2=true;
			endangle = s1angleend;
		}

	}
	if((startflag1&&startflag2)||(stopflag1&&stopflag2))
	{
		if(s1anglebegin > s1angleend &&s1anglebegin<181&&s1anglebegin>=179&&s1angleend<=181&&s1angleend>=179)
		{
			beginangle=s2anglebegin;
			endangle = s2angleend;
			beginangle_vec.push_back(beginangle);
			endangle_vec.push_back(endangle);
			float diffangle =  endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;

			diffangle_vec.push_back(diffangle);
			return diffangle_vec;
		}
		else if(s2anglebegin > s2angleend &&s2anglebegin<181&&s2anglebegin>=179&&s2angleend<=181&&s2angleend>=179)
		{
			beginangle=s1anglebegin;
			endangle = s1angleend;
			beginangle_vec.push_back(beginangle);
			endangle_vec.push_back(endangle);
			float diffangle =  endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;
			/*(beginangle>endangle
				?((beginangle+endangle)>360?(beginangle+endangle-360)/2:(beginangle+endangle+360)/2)
				:(beginangle+endangle)/2);*/
			diffangle_vec.push_back(diffangle);
			return diffangle_vec;
		}

		float anglediff1= s2angleend>=s1anglebegin?s2angleend-s1anglebegin:360+s2angleend-s1anglebegin;
		//s1anglebegin>s2angleend
		//	?((s1anglebegin+s2angleend)>360?(s1anglebegin+s2angleend-360)/2:(s1anglebegin+s2angleend+360)/2)
		//	:(s1anglebegin+s2angleend)/2;
		float anglediff2= s1angleend>=s2anglebegin?s1angleend-s2anglebegin:360+s1angleend-s2anglebegin;
		if(s2angleend>s1angleend)
		{

			if(anglediff1>0)
			{
				beginangle_vec.push_back(s1anglebegin);
				endangle_vec.push_back(s2angleend);
				diffangle_vec.push_back(anglediff1);
			}

			if(anglediff2>0)
			{
				beginangle_vec.push_back(s2anglebegin);
				endangle_vec.push_back(s1angleend);
				diffangle_vec.push_back(anglediff2);
			}

			return diffangle_vec;
		}
		else
		{

			if(anglediff2>0)
			{
				beginangle_vec.push_back(s2anglebegin);
				endangle_vec.push_back(s1angleend);
				diffangle_vec.push_back(anglediff2);
			}

			if(anglediff1>0)
			{
				beginangle_vec.push_back(s1anglebegin);
				endangle_vec.push_back(s2angleend);
				diffangle_vec.push_back(anglediff1);
			}

			return diffangle_vec;
		}

	}
	if((stopflag1||stopflag2)&&(startflag1||startflag2))
	{
		beginangle_vec.push_back(beginangle);
		endangle_vec.push_back(endangle);
		float diffangle = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;
		diffangle_vec.push_back(diffangle);
		return diffangle_vec;
	}
	else
		return diffangle_vec;

}
void LidarProcess::getRoadPath()
{
    roadpaths.paths.clear();
    std::vector<RoadPath> temproadpaths;
    CloudPointSections* tempsections = &cloudsections[0];
    if(cloudsections[0].sections.size()>0)
    {
        for(int i=0;i<cloudsections[0].sections.size();i++)
        {
			if(fabs(cloudsections[0].sections[i].height)>0.2)
				continue;
            RoadPath temproad(LASER_LAYER);
			RoadNode tempnode;
			tempnode.index = i;
			tempnode.valid=true;

			tempnode.anglebegin=cloudsections[0].sections[i].minpoint.angle;
			tempnode.angleend=cloudsections[0].sections[i].maxpoint.angle;
			tempnode.anglerange=cloudsections[0].sections[i].anglerange;
			tempnode.distance=cloudsections[0].sections[i].distance;
			tempnode.height=cloudsections[0].sections[i].height;
			tempnode.lastdistance=0;
			tempnode.lastheight=0;
            tempnode.angle = tempnode.anglebegin>tempnode.angleend
               ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
               :(tempnode.anglebegin+tempnode.angleend)/2;


			temproad.nodes.push_back(tempnode);
            temproadpaths.push_back(temproad);
//            std::cout<<"capacity:"<<roadpath[i].capacity()<<std::endl;
        }

        for(int j=1 ;cloudsections[j].sections.size()>0;j++)
        {

            for(std::vector<RoadPath>::iterator it_path=temproadpaths.begin() ; it_path!=temproadpaths.end();it_path++)
            {
                RoadPath path(*it_path);
//                if(path.size()<j )
                if(path.nodes.size()<j)
                    continue;
                std::vector<RoadNode> next_nodes ;
                int this_node_index = path.nodes[j-1].index;     //the last node of this path now;
//                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j-1<<"\t"<<&tempsections[size_t(j-1)]<<"\t"<<&tempsections[0]<<"\t"<<sizeof(size_t)<<std::endl;
//                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j-1<<"\t"<<&cloudsections[j-1]<<std::endl;
//                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j<<"\t"<<&cloudsections[j]<<std::endl;
                if(&cloudsections[j-1] +1 != &cloudsections[j] )
                    continue;
                const CloudPointSection& this_section = cloudsections[j-1].sections[this_node_index];
                int num = 0;
                std::vector<RoadPath> nextpaths;
                for(int i=0;i<cloudsections[j].sections.size();i++)
                {
 //                   std::cout<<"i="<<i<<"\tsize="<<cloudsections[j].sections.size()<<std::endl;
                    const CloudPointSection& tempsection = cloudsections[j].sections[i];
                    float s1anglebegin=this_section.minpoint.angle;
                    float s1angleend=this_section.maxpoint.angle;
                    float s2anglebegin=tempsection.minpoint.angle;
                    float s2angleend=tempsection.maxpoint.angle;

					std::vector<float> beginangle_vec,endangle_vec;   //there may be two overlap section.
                    float minangle = this_section.anglerange>tempsection.anglerange?tempsection.anglerange:this_section.anglerange;
                    std::vector<float> anglediff_vec = getAngleOverlap(s1anglebegin,s1angleend,s2anglebegin,s2angleend,beginangle_vec,endangle_vec);
					for(int vec_i=0;vec_i<anglediff_vec.size();vec_i++)
					{

//                        std::cout<<"anglediffsize="<<anglediff_vec.size()<<std::endl;
						float anglediff = anglediff_vec[vec_i];
						float beginangle = beginangle_vec[vec_i];
						float endangle = endangle_vec[vec_i];
                        float angletotaldiff = anglediff;
                        if(anglediff_vec.size()>1)
                            angletotaldiff = anglediff_vec[0]+anglediff_vec[1];
//							std::cout<<j<<" "<<this_node_index<<" "<<i<<" begin:"<<beginangle<<" end:"<<endangle<<" overlap:"<<anglediff<<" minangle"<<minangle<<" capacity:"<<it_path->nodes.capacity()<<std::endl;
						if(fabs(anglediff)>anglerange[j-1]&&fabs(angletotaldiff/minangle)>0.5)
						{
							RoadNode tempnode;
							tempnode.index=i;
							//float anglerange = getAngleOverlap(path.nodes[j-1].anglebegin,path.nodes[j-1].angleend,beginangle,endangle,tempnode.anglebegin,tempnode.angleend);

					        std::vector<float> beginangle_vec1,endangle_vec1;   //there may be two overlap section.
                            std::vector<float> anglediff_vec1 = getAngleOverlap(path.nodes[j-1].anglebegin,path.nodes[j-1].angleend,beginangle,endangle,beginangle_vec1,endangle_vec1);

                            if(anglediff_vec1.size()==0)
                            {
                                tempnode.valid = true;
                                tempnode.anglebegin = beginangle;
                                tempnode.angleend = endangle;
                                tempnode.anglerange = 0;
                                tempnode.angle = tempnode.anglebegin>tempnode.angleend
                                   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
                                   :(tempnode.anglebegin+tempnode.angleend)/2;

								RoadPath temppath(path);
								temppath.nodes.push_back(tempnode);
                                nextpaths.push_back(temppath);
                            }
					        for(int vec_j=0;vec_j<anglediff_vec1.size();vec_j++)
							{
                                double tempanglebegin = beginangle_vec1[vec_j];
                                double tempangleend = endangle_vec1[vec_j];
                                float heightdiff,disdiff,this_height,this_distance,last_height,last_distance;
                                if(tempangleend>=tempanglebegin)
                                {
                                    int laser_num_this = grabber_H_.indexmaptable[j].number;
                                    int laser_num_last = grabber_H_.indexmaptable[j-1].number;
                                    int mat_begin = tempanglebegin*10;
                                    int mat_end = tempangleend*10;
                                    int this_countsum = passable_count_integral[laser_num_this][mat_end] - passable_count_integral[laser_num_this][mat_begin];
                                    int last_countsum = passable_count_integral[laser_num_last][mat_end] - passable_count_integral[laser_num_last][mat_begin];
                                    double this_heightsum = passable_height_integral[laser_num_this][mat_end] - passable_height_integral[laser_num_this][mat_begin];
                                    double last_heightsum = passable_height_integral[laser_num_last][mat_end] - passable_height_integral[laser_num_last][mat_begin];
                                    double this_distancesum = passable_distance_integral[laser_num_this][mat_end] - passable_distance_integral[laser_num_this][mat_begin];
                                    double last_distancesum = passable_distance_integral[laser_num_last][mat_end] - passable_distance_integral[laser_num_last][mat_begin];

                                    this_height = this_heightsum/ this_countsum;
                                    last_height = last_heightsum/ last_countsum;

                                    this_distance = this_distancesum/ this_countsum;
                                    last_distance = last_distancesum / last_countsum;
                                    heightdiff = (this_height - last_height);
                                    disdiff = (this_distance - last_distance);
//                                    std::cout<<"this_countsum:"<<this_countsum<<"\tlast_countsum"<<last_countsum<<std::endl;

                                }
                                else
                                {
                                    int laser_num_this = grabber_H_.indexmaptable[j].number;
                                    int laser_num_last = grabber_H_.indexmaptable[j-1].number;
                                    int mat_begin = tempanglebegin*10;
                                    int mat_end = tempangleend*10;
                                    int this_countsum = passable_count_integral[laser_num_this][mat_end] + passable_count_integral[laser_num_this][3600] - passable_count_integral[laser_num_this][mat_begin];
                                    int last_countsum = passable_count_integral[laser_num_last][mat_end] + passable_count_integral[laser_num_last][3600] - passable_count_integral[laser_num_last][mat_begin];
                                    double this_heightsum = passable_height_integral[laser_num_this][mat_end] + passable_height_integral[laser_num_this][3600] - passable_height_integral[laser_num_this][mat_begin];
                                    double last_heightsum = passable_height_integral[laser_num_last][mat_end] + passable_height_integral[laser_num_last][3600] - passable_height_integral[laser_num_last][mat_begin];
                                    double this_distancesum = passable_distance_integral[laser_num_this][mat_end] + passable_distance_integral[laser_num_this][3600] - passable_distance_integral[laser_num_this][mat_begin];
                                    double last_distancesum = passable_distance_integral[laser_num_last][mat_end] + passable_distance_integral[laser_num_last][3600] - passable_distance_integral[laser_num_last][mat_begin];

                                    this_height = this_heightsum/ this_countsum;
                                    last_height = last_heightsum/ last_countsum;

                                    this_distance = this_distancesum/ this_countsum;
                                    last_distance = last_distancesum/ last_countsum;
                                    heightdiff = (this_height - last_height);
                                    disdiff = (this_distance - last_distance);
//                                    std::cout<<"this_countsum:"<<this_countsum<<"\tlast_countsum"<<last_countsum<<std::endl;

                                }

//                                std::cout<<"heightdiff:"<<heightdiff<<"\tbegin:"<<tempanglebegin<<"\tend:"<<tempangleend<<std::endl;

                                if((heightdiff)>0.2&&(heightdiff/disdiff)>0.1)
                                    continue;

                                tempnode.anglebegin = beginangle_vec1[vec_j];
                                tempnode.angleend = endangle_vec1[vec_j];
                                tempnode.anglerange = anglediff_vec1[vec_j];
                                tempnode.distance = this_distance;
							    tempnode.height = this_height;
                                tempnode.lastdistance = last_distance;
							    tempnode.lastheight = last_height;

                                float anglerange = anglediff_vec1[vec_j];
								RoadPath temppath(path);
								if(fabs(anglerange)>0)
								{
									temppath.nodes[j-1].valid=false;
									tempnode.valid = true;
									tempnode.angle = tempnode.anglebegin>tempnode.angleend
									   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
					    			   :(tempnode.anglebegin+tempnode.angleend)/2;
								}
								else
								{
									tempnode.valid = true;
									tempnode.anglebegin = beginangle;
									tempnode.angleend = endangle;
									tempnode.angle = tempnode.anglebegin>tempnode.angleend
									   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
									   :(tempnode.anglebegin+tempnode.angleend)/2;

								}
								temppath.nodes.push_back(tempnode);
	//                            std::cout<<"iter num:"<<it_path-temproadpaths.begin()<<" size"<<temproadpaths.size()<<std::endl;
                                nextpaths.push_back(temppath);
								//it_path=temproadpaths.insert(it_path+1,temppath);

	//                            std::cout<<"iter num:"<<it_path-temproadpaths.begin()<<" size"<<temproadpaths.size()<<std::endl;
							}
							//next_nodes.push_back(i);
						}
					}
                }

                float maxheight=-2;
                float heightdiff_thre=0.3;
                float anglediff_thre = 50;
                float totalbeginangle = 0;
                float lastheight;
                float lastendangle;
                float lastangle;
                int lastnum = 0;
                if(nextpaths.size()>0)
                {
                    int pathnum = nextpaths.size();
                    unsigned char isvalid[pathnum];
                    memset(isvalid,1,pathnum);
//                    std::cout<<j<<" "<<pathnum<<std::endl;
                    for(std::vector<RoadPath>::iterator it=nextpaths.begin();it!=nextpaths.end();it++)
                    {
                        int num = it - nextpaths.begin();
//                        std::cout<<"num:"<<j<<std::endl;
 //                       std::cout<<"j:"<<j<<"\tsize:"<<(int)pathnum<<"\tnum:"<<(int)num<<"\tlastnum:"<<(int)lastnum<<"\tlastangle"<<lastangle<<std::endl;
                        if(num==0)
                        {
                            lastheight = it->nodes.at(j).height;
                            lastendangle = it->nodes.at(j).angleend;
                            totalbeginangle = it->nodes.at(j).anglebegin;
                            lastangle = it->nodes.at(j).angle;
                            lastnum = 0;
                        }
                        else
                        {
                            if((it->nodes.at(j).lastheight - it->nodes.at(j).height > heightdiff_thre) && (lastheight - it->nodes.at(j).height)>heightdiff_thre )
                            {
                                if(it->nodes.at(j).angle - lastendangle< anglediff_thre )
                                {
                                    isvalid[num] = 0;
                                }
                                else
                                {
                                    lastheight = it->nodes.at(j).height;
                                    lastendangle = it->nodes.at(j).angleend;
                                    lastangle = it->nodes.at(j).angle;
                                    lastnum = num;

                                }
                            }
                            else if((nextpaths.at(lastnum).nodes.at(j).lastheight/*last node of same path*/ - lastheight/*last section of same beam*/) >heightdiff_thre
                                    && (it->nodes.at(j).height - lastheight)>heightdiff_thre)
                            {
                                float endangle = it->nodes.at(j).anglebegin;
                                float beginangle = lastangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;
                                if(anglediff < anglediff_thre)
                                {
                                    isvalid[lastnum] = 0;
                                }
                                lastheight = it->nodes.at(j).height;
                                lastendangle = it->nodes.at(j).angleend;
                                lastangle = it->nodes.at(j).angle;
                                lastnum = num;

                            }
                            else
                            {
                                lastheight = it->nodes.at(j).height;
                                lastendangle = it->nodes.at(j).angleend;
                                lastangle = it->nodes.at(j).angle;
                                lastnum = num;

                            }
                        }
                        if(it+1==nextpaths.end()&&lastnum!=0)
                        {
                            if((lastheight - nextpaths[0].nodes.at(j).height)>heightdiff_thre )
                            {
                                float endangle = nextpaths[0].nodes.at(j).angle;
                                float beginangle = lastendangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;

                                if(anglediff < anglediff_thre)
                                {
                                    isvalid[0] = 0;
                                }
                            }
                            else if((nextpaths[0].nodes.at(j).height - lastheight)>heightdiff_thre)
                            {
                                float endangle = nextpaths[0].nodes.at(j).anglebegin;
                                float beginangle = lastangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;

                                if(anglediff < anglediff_thre)

                                if(nextpaths[0].nodes.at(j).angle - lastendangle< anglediff_thre )
                                {
                                    isvalid[lastnum] = 0;
                                }

                            }

                        }


                    }


                    int i=0;
                    for(;i<nextpaths.size();i++)
                    {
                        if(isvalid[i])
                        {
                            (*it_path)=nextpaths[i];
                            i++;
                            break;
                        }
                    }
//                    std::cout<<j<<" "<<(*it_path)[j]<<std::endl;
                    for(std::vector<RoadPath>::iterator it=nextpaths.begin()+i;it!=nextpaths.end();it++)
                    {

                        if(isvalid[it-nextpaths.begin()])
                            it_path=temproadpaths.insert(it_path+1,*it);
                    }

                }


            }

        }
//        std::cout<<temproadpaths.size()<<std::endl;

        for(std::vector<RoadPath>::iterator it_path=temproadpaths.begin() ; it_path!=temproadpaths.end();it_path++)
        {
            RoadPath path(*it_path);
            if(path.nodes.size()>16)
            {

                for(std::vector<RoadNode>::iterator it=path.nodes.begin();it!=path.nodes.end();it++)
                {
                    //if(it->index<0)
                    //    break;

//                    std::cout<<it->index;
                    if(it->valid)
                    {
                        path.validnum.push_back(it-path.nodes.begin());
//                        std::cout<<"("<<it->anglebegin<<","<<it->angle<<","<<it->angleend<<","<<it->distance<<")";
                    }
//                    std::cout<<"->";
                }
//                std::cout<<std::endl;

                if(path.validnum[0]<14||path.validnum.size()>2)
                    continue;

                roadpaths.paths.push_back(path);
            }
        }

        if(roadpaths.paths.size()==0)
            return;
        try{

        std::sort(roadpaths.paths.begin(),roadpaths.paths.end(),std::greater<RoadPath>());
        }
        catch(std::exception &e){
            std::cerr<<"sort err"<<std::endl;
        }

//        std::cout<<"roadpaths.paths.size()="<<roadpaths.paths.size()<<std::endl;
        if(roadpaths.paths.size()>6)
            roadpaths.paths.erase(roadpaths.paths.begin()+6,roadpaths.paths.end());

        try{
            std::sort(roadpaths.paths.begin(),roadpaths.paths.end(),RoadPath::angleendsort);
        }
        catch(std::exception &e){
            std::cerr<<"sort err"<<std::endl;
        }

        for(std::vector<RoadPath>::iterator it_path=roadpaths.paths.begin() ; it_path!=roadpaths.paths.end()-1;it_path++)
        {
            RoadPath path(*it_path);
            RoadPath nextpath(*(it_path+1));
            int backnum = path.validnum.at(0);
            int nextbacknum = nextpath.validnum.at(0);

            double anglediff = nextpath.nodes.at(nextbacknum).anglebegin>nextpath.nodes.at(nextbacknum).angleend
                ? nextpath.nodes.at(nextbacknum).anglebegin- 360 - path.nodes.at(backnum).angleend
                : nextpath.nodes.at(nextbacknum).anglebegin - path.nodes.at(backnum).angleend;

            if(anglediff <8 )
            {

                if(nextpath>path)
                {
                    int diffnum = 0;
                    for(int i=0;i<=backnum;i++)
                    {
                        if(path.nodes.at(i).index != nextpath.nodes.at(i).index)
                            diffnum++;
                    }
                    if(diffnum==0)
                    {
                        it_path = roadpaths.paths.erase(it_path);
                        it_path --;
                    }
                    else if(diffnum==1)
                    {
                        it_path = roadpaths.paths.erase(it_path);
                        it_path->nodes.at(nextbacknum).anglebegin = path.nodes.at(backnum).anglebegin;
                        double anglebegin = it_path->nodes.at(nextbacknum).anglebegin;
                        double angleend = it_path->nodes.at(nextbacknum).angleend;
                        it_path->nodes.at(nextbacknum).angle = anglebegin>angleend
									   ?((anglebegin+angleend)>360?(anglebegin+angleend-360)/2:(anglebegin+angleend+360)/2)
					    			   :(anglebegin+angleend)/2;

		                it_path->nodes.at(nextbacknum).anglerange = angleend>=anglebegin?angleend-anglebegin:360+angleend-anglebegin;
                        it_path --;
                    }
                }
                else
                {
                    int diffnum = 0;
                    for(int i=0;i<=nextbacknum;i++)
                    {
                        if(path.nodes.at(i).index != nextpath.nodes.at(i).index)
                            diffnum++;
                    }
                    if(diffnum==0)
                    {
                        it_path = roadpaths.paths.erase(it_path+1);
                        it_path -=2;
                    }
                    else if(diffnum==1)
                    {
                        it_path = roadpaths.paths.erase(it_path+1);
                        it_path --;
                        it_path->nodes.at(backnum).angleend = nextpath.nodes.at(nextbacknum).angleend;
                        double anglebegin = it_path->nodes.at(backnum).anglebegin;
                        double angleend = it_path->nodes.at(backnum).angleend;
                        it_path->nodes.at(backnum).angle = anglebegin>angleend
									   ?((anglebegin+angleend)>360?(anglebegin+angleend-360)/2:(anglebegin+angleend+360)/2)
					    			   :(anglebegin+angleend)/2;

		                it_path->nodes.at(backnum).anglerange = angleend>=anglebegin?angleend-anglebegin:360+angleend-anglebegin;
                        it_path --;
                    }
                }

            }
        }


//        std::cout<<roadpaths.paths.size()<<std::endl;

        for(std::vector<RoadPath>::iterator it_path=roadpaths.paths.begin() ; it_path!=roadpaths.paths.end();it_path++)
        {
            RoadPath path(*it_path);
            {

                for(std::vector<RoadNode>::iterator it=path.nodes.begin();it!=path.nodes.end();it++)
                {
                    //if(it->index<0)
                    //    break;

//                    std::cout<<it->index;
                    if(it->valid)
                    {
                        path.validnum.push_back(it-path.nodes.begin());
//                        std::cout<<"("<<it->anglebegin<<","<<it->angle<<","<<it->angleend<<","<<it->distance<<")";
                    }
//                    std::cout<<"->";
                }
//                std::cout<<std::endl;
            }
        }

    }

}


void LidarProcess::DrawBoundary()
{
    boundarycloud->clear();
    roadorientationcloud->clear();
    double resolution = 0.01;
    for(std::vector<POINT_2F>::iterator it = roadboundaries.mainroad_leftboundary.points.begin() ; it != roadboundaries.mainroad_leftboundary.points.end() ;it++ )
    {
        PointXYZI temppoint;
        temppoint.x = it->x;
        temppoint.y = it->y;
        temppoint.z = it->height;
        boundarycloud->push_back(temppoint);
    }

    for(std::vector<POINT_2F>::iterator it = roadboundaries.mainroad_rightboundary.points.begin() ; it != roadboundaries.mainroad_rightboundary.points.end() ;it++ )
    {
        PointXYZI temppoint;
        temppoint.x = it->x;
        temppoint.y = it->y;
        temppoint.z = it->height;
        boundarycloud->push_back(temppoint);
    }

    if(roadboundaries.left_found)
    {
        if(roadboundaries.mainroad_leftboundary.pose.style==1 || roadboundaries.mainroad_leftboundary.pose.style==2)
        {
            double a0 = roadboundaries.mainroad_leftboundary.pose.a0;
            double a1 = roadboundaries.mainroad_leftboundary.pose.a1;
            double a2 = roadboundaries.mainroad_leftboundary.pose.a2;
            for(double y=-20;y<60;y+=resolution)
            {
                double x=a0+a1*y+a2*y*y;
                double tempx,tempy;
                PointXYZI temppoint ;
                roadboundaries.RotatePoint(x,y,tempx,tempy,roadboundaries.mainroad_leftboundary.pose.theta);
                temppoint.x = tempx;
                temppoint.y = tempy;

                boundarycloud->push_back(temppoint);
            }
        }
    }

    if(roadboundaries.right_found)
    {
        if(roadboundaries.mainroad_rightboundary.pose.style==1 || roadboundaries.mainroad_rightboundary.pose.style==2)
        {
            double a0 = roadboundaries.mainroad_rightboundary.pose.a0;
            double a1 = roadboundaries.mainroad_rightboundary.pose.a1;
            double a2 = roadboundaries.mainroad_rightboundary.pose.a2;
            for(double y=-20;y<60;y+=resolution)
            {
                double x=a0+a1*y+a2*y*y;
                double tempx,tempy;
                PointXYZI temppoint ;
                roadboundaries.RotatePoint(x,y,tempx,tempy,roadboundaries.mainroad_rightboundary.pose.theta);
                temppoint.x = tempx;
                temppoint.y = tempy;

                boundarycloud->push_back(temppoint);
            }
        }
    }
/*
    if(roadpaths.paths.size()>0)
    {
        for(int i=0;i<roadpaths.paths.size()&&i<6;i++)
        {
            const RoadPath& temppath= roadpaths.paths.at(i);
            if(temppath.validnum.at(0)<18)
                break;
            const RoadNode& tempnode = temppath.nodes.at(temppath.validnum.at(0));
            double angle = tempnode.angle*M_PI/180;
            std::cout<<"distance:"<<tempnode.distance<<std::endl;
            std::cout<<"angle:"<<tempnode.angle<<std::endl;
            for(double r=0;r<tempnode.distance;r+=resolution)
            {
                double x=r*cos(angle);
                double y=r*sin(angle);
                double z=r * tempnode.height / tempnode.distance;
                PointXYZI temppoint;
                temppoint.x = x;
                temppoint.y = y;
                temppoint.z = z;
                roadorientationcloud->push_back(temppoint);
            }
        }
    }*/

}

void LidarProcess::splitPointCloud()
{

    //
    if(1)
    {

        passablecloud->clear();
        rigid_nopassablecloud->clear();

        positiveslopecloud->clear();
        negativeslopecloud->clear();

        //passablecloud.swap(Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>));
        //rigid_nopassablecloud.swap(Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>));

        if(0)
        {
            //
            for (int m = 0; m < velodyne_pointcloud->points.size(); m++)
            {
                if(velodyne_pointcloud->points[m].passibility < 0.5)
                    rigid_nopassablecloud->points.push_back(velodyne_pointcloud->points[m]);
                else
                    passablecloud->points.push_back(velodyne_pointcloud->points[m]);//��ͨ������
            }
        }
        else
        {
            //�������񻯽�����ʾ
            for (int m = 0; m < velodyne_pointcloud->points.size(); m++)
            {
                float x = velodyne_pointcloud->points[m].x,
                      y = velodyne_pointcloud->points[m].y,
                      z = velodyne_pointcloud->points[m].z;

                float ogm_y_offset = 20.0f;//��y�Ḻ�����ƶ���20
                float newy = y + ogm_y_offset;
                if((x >=-hdl_ogm_data.ogmwidth / 2  && x <= hdl_ogm_data.ogmwidth / 2) &&
                        (newy >=0 && newy < hdl_ogm_data.ogmheight))
                {
                    int col = boost::math::round(x / hdl_ogm_data.ogmresolution) + ( hdl_ogm_data.ogmwidth_cell - 1 ) / 2;
                    int row = boost::math::round(newy / hdl_ogm_data.ogmresolution) ;


                    if(row >=0 && row < hdl_ogm_data.ogmheight_cell && col >=0 && col < hdl_ogm_data.ogmwidth_cell)
                    {
                        int index = row * hdl_ogm_data.ogmwidth_cell + col;

                        //����Σ������
                        if(hdl_ogm_data.ogm[index] == RIGIDNOPASSABLE)
                        {
                            rigid_nopassablecloud->points.push_back(velodyne_pointcloud->points[m]);
                            velodyne_pointcloud->points[m].passibility = 0;
                            continue;
                        }
                    }

                    newy = y + slope5xogm.ogmheight/2;
                    if((x >=-slope5xogm.ogmwidth / 2  && x <= slope5xogm.ogmwidth / 2) &&
                            (newy >=0 && newy < slope5xogm.ogmheight))
                    {
                        int col = boost::math::round(x / slope5xogm.ogmresolution) + ( slope5xogm.ogmwidth_cell - 1 ) / 2;
                        int row = boost::math::round(newy / slope5xogm.ogmresolution) ;

                        if(row >=0 && row < slope5xogm.ogmheight_cell && col >=0 && col < slope5xogm.ogmwidth_cell)
                        {
                            int index = row * slope5xogm.ogmwidth_cell + col;

                            //����Σ������
                            float val = slope5xogm.ogm[index];
                            if(val < 500)
                            {
                                if(val > 0.01)
                                {
                                    positiveslopecloud->points.push_back(velodyne_pointcloud->points[m]);
                                    velodyne_pointcloud->points[m].passibility = 0.2;
                                    continue;
                                }
                                else if(val < -0.01)
                                {
                                    negativeslopecloud->points.push_back(velodyne_pointcloud->points[m]);//��ͨ������
                                    velodyne_pointcloud->points[m].passibility = -0.2;
                                    continue;
                                }
                            }
                        }

                    }

                    passablecloud->points.push_back(velodyne_pointcloud->points[m]);//��ͨ������
                    velodyne_pointcloud->points[m].passibility = 1.0;
                }
            }
        }

        pointcloud_updateflag=true;
    }//if(display_)


}

int LidarProcess::computepointcloud()
{
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloud;
  if(freeze_ == 1 && replay_ == 1)
  {
      grabber_H_.pause();
      //  cloud_viewer_->spinOnce();
      return 0;
  }
  else
  {
      grabber_H_.resume();
  if (cloud_mutex_H_.try_lock ())
   {
       cloud_H_.swap (pointcloud);
       cloud_mutex_H_.unlock ();
   }

   if (pointcloud){
       //
       int col_count = pointcloud->size() / LASER_LAYER;
       if(col_count > 5000)
       {

           cout<<"cloud too large \tcount="<<col_count<<endl;
           return 0;
       }
       int residual = pointcloud->size() % LASER_LAYER;
       if (residual > 0)
       {
           cout<<"residual non zero"<<endl;
           return 0;
       }
       //cout<<col_count << " " << pointcloud->points[0].azimuth << "-" << pointcloud->points[LASER_LAYER].azimuth << endl;
       pcl::transformPointCloud (*pointcloud, *velodyne_pointcloud,transform_matrix_calibration_H2car_angle);
   }
   else{
       // cout<<"not get cloud"<<endl;
       return 0;
   }




   if(pointcloud)
   {
       //                            cloud_viewer_->removeAllShapes();


//       cout<<"pointnum="<<velodyne_pointcloud->size()<<endl;
       memset(polaraxismat_ , 0 ,  sizeof(PolarPointDI)*LASER_LAYER*3601);
       for (int i = 0; i < velodyne_pointcloud->points.size(); i++)
       {
           //range < 0.5 ��Ч
	   velodyne_pointcloud->points[i].passibility=1.0;
           //if(velodyne_pointcloud->points[i].range <0 )
	   float error = 0.00001;
           if(fabs(pointcloud->points[i].x-0.1)<error&&fabs(pointcloud->points[i].y-0.1)<error&&fabs(pointcloud->points[i].y-0.1)<error)
           {

               //std::cout<<"-range "<<i<<" "<<velodyne_pointcloud->points[i].range<<std::endl;
               velodyne_pointcloud->points[i].range = -0.1;
               float azimuth= 360.0 - velodyne_pointcloud->points[i].azimuth + 90.0 + gama_h * 180 / M_PI;
               while(azimuth > 360.0) azimuth -=360.0;
               while(azimuth <=0) azimuth +=360.0;
               velodyne_pointcloud->points[i].azimuth=azimuth;
               continue;

           }

           //limit
//           if(velodyne_pointcloud->points[i].x > -2 && velodyne_pointcloud->points[i].x < 2 &&
//                   velodyne_pointcloud->points[i].y < 1.5 && velodyne_pointcloud->points[i].y > -5)
           if(velodyne_pointcloud->points[i].x > -2 && velodyne_pointcloud->points[i].x < 2 &&
                   velodyne_pointcloud->points[i].y < 2.5 && velodyne_pointcloud->points[i].y > -2.5)
           {
               velodyne_pointcloud->points[i].range =- 0.01;
//                double tempdistance=sqrt(pointcloud->points[i].x * pointcloud->points[i].x +
//					 pointcloud->points[i].y * pointcloud->points[i].y );
//               velodyne_pointcloud->points[i].range = tempdistance;
               float azimuth=atan2(pointcloud->points[i].y,pointcloud->points[i].x)*180/M_PI;
               azimuth = azimuth + gama_h * 180 / M_PI;
               while(azimuth > 360.0) azimuth -=360.0;
               while(azimuth <=0) azimuth +=360.0;
               velodyne_pointcloud->points[i].azimuth=azimuth;
               velodyne_pointcloud->points[i].z=0;
               int mat_i=azimuth*10;
               PolarPointDI temppolarpoint;
               temppolarpoint.distance=velodyne_pointcloud->points[i].range;
               temppolarpoint.index=i;
               polaraxismat_[i%LASER_LAYER][mat_i]=temppolarpoint;

           }
           else
           {
               double tempdistance=sqrt(pointcloud->points[i].x * pointcloud->points[i].x +
					pointcloud->points[i].y * pointcloud->points[i].y );
               velodyne_pointcloud->points[i].range = tempdistance;
               float azimuth=atan2(pointcloud->points[i].y,pointcloud->points[i].x)*180/M_PI;
               azimuth = azimuth + gama_h * 180 / M_PI;
               while(azimuth > 360.0) azimuth -=360.0;
               while(azimuth <=0) azimuth +=360.0;
               velodyne_pointcloud->points[i].azimuth=azimuth;
               int mat_i=azimuth*10;
               PolarPointDI temppolarpoint;
               temppolarpoint.distance=tempdistance;
               temppolarpoint.index=i;
               polaraxismat_[i%LASER_LAYER][mat_i]=temppolarpoint;
           }
       }
   }

   return 1;
}
}

int LidarProcess::pointCloudProcess ()
{
    {
        //                    pcl::PointCloud<pcl::PointXYZI>::Ptr  velodyne_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);//

        //velodyne_pointcloud.reset();//.swap(Cloud::Ptr());


            if(velodyne_pointcloud)
            {
            timer t;
            MyTime mytime;
            mytime.start();
                //std::cout<<"start"<<std::endl;
                //                            mytime.stop();
                //                            mytime.show_s();

#ifdef USE_OMP
#pragma omp parallel sections
#endif
                {
#ifdef USE_OMP
#pragma omp section
#endif
                    if(flag_ogmdetection)
                    {
                        ogmDetection();
//                        std::cout<<"four end"<<std::endl;
//                        mytime.stop();
//                        mytime.show_s();

                    }

#ifdef USE_OMP
#pragma omp section
#endif
                    {
                        //use circle's tangential_angle //2016/12/03
                        if(flag_circletangential)
                        {
                            circletangentialDetection();
//                            std::cout<<"one end"<<std::endl;
//                            mytime.stop();
//                            mytime.show_s();
                        }
                    }
#ifdef USE_OMP
#pragma omp section
#endif
                    {
                        //slope detection and hight diff detection //2016/12/03
                        if(flag_diffdetection)  //tahe sousuo 0
                        {
                            heightdiffDetection();
//                            std::cout<<"two end"<<std::endl;
//                            mytime.stop();
//                            mytime.show_s();
                        }
                    }
#ifdef USE_OMP
#pragma omp section
#endif
                    {
                        //use circle's radius
                        if(flag_circleradius)
                        {
                            circleradiusDetection();
//                            std::cout<<"three end"<<std::endl;
//                            mytime.stop();
//                            mytime.show_s();
                        }
                    }
                }
                //stiff detect
                //                            pcl::PointCloud<pcl::PointXYZI>::Ptr  stiff_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);

//                mytime.stop();
//                mytime.show_s();
            }//if(velodyne_pointcloud


        }

    return 1;
    //delete[] lane_ogm;

    //        cout<<"count"<<cloud_viewer_.use_count()<<endl;
}



#ifndef HDL64STRUCTURE_H
#define HDL64STRUCTURE_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/io/hdl_grabber.h>
//#include <pcl/visualization/image_viewer.h>
//#include <pcl/common/pca.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>

#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/date_time.hpp>

#include <fstream>
#include "util/xmlconf/xmlconf.h"
#include "data_types.hpp"
//#include "tree/st_tree.h"
#include "myhdl_grabber.h"
#include "roadboundary.h"

#include <stdio.h>
#include <vector>
#include <string>
#include <typeinfo>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>


using namespace std;
//using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

using namespace boost;

#define Z_MAX					2.3
#define Z_MIN					-2
#define DEADZONE_LEFT -2
#define DEADZONE_RIGHT 2
#define DEADZONE_FRONT 3
#define DEADZONE_BACK -3


#define SAC_MAX_ITERATIONS		100
#define MIN_DISTANCE_THRESHOLD	1.0
#define MAX_DISTANCE_THRESHOLD	80.0
#define EPSILON					0.001


#define UNKNOWN					0
#define PASSABLE				1
#define RIGIDNOPASSABLE			2
#define RADARNOPASSABLE			4
#define UNFLATNESS				6
#define WATERNOPASS             3
#define GRASS					5
#define VACANT                  7
#define NEW_VACANT              8

#define MAX_LAYER 64
#define ROTATION_SPEED 20	//1200 RPM
#define ROTATION_RESOLUTION 0.2 //1200 RPM

//#define srcip "192.168.0.112"
//#define dstip "192.168.0.111"

//#define srcip  "127.0.0.1"
//#define dstip  "127.0.0.1"

//#define SRCIP  "192.168.0.112"
//#define DSTIP  "192.168.0.111"

#define SRCIP  "127.0.0.1"
#define DSTIP  "127.0.0.1"
#define OGMWIDTH 40.0
#define OGMHEIGHT 80.0
#define OGMRESOLUTION 0.2
struct PolarPointDI{
    double distance;
    int index;
};

union _uintbyte
{
    UINT16 _uint;
    UINT8	_uchar[2];
};


typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


class LidarProcess
{
    public:
        LidarProcess (pcl::VelodyneGrabber& grabber_H,
                bool replay,
                bool display,
                float velodyneheight,
                float rigid_heightdiffthreshold,
                XmlConf& xmlconfig,const pcl::CalibrationValue& calibvalue,
		boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)>* fpointcloud_cb=NULL)
            : grabber_H_(grabber_H)
              , replay_(replay)
              , display_ (display)
              , rigid_heightdiffthreshold_(rigid_heightdiffthreshold)
              , xmlconfig_(xmlconfig)
	      , calibvalue_(calibvalue)
              , hdl_ogm_data(80,40,0.2)
              , refineogm(40,40,0.2)
              , minzogm(40,40,1)
              , minzslopeogm(40,40,0.2)
              , minzrefineogm(60,40,0.2)
              , slope5xogm(minzrefineogm.ogmheight,minzrefineogm.ogmwidth,minzrefineogm.ogmresolution)
              , slope5yogm(minzrefineogm.ogmheight,minzrefineogm.ogmwidth,minzrefineogm.ogmresolution)
              , slope2xogm(minzrefineogm.ogmheight,minzrefineogm.ogmwidth,minzrefineogm.ogmresolution)
              , slope2yogm(minzrefineogm.ogmheight,minzrefineogm.ogmwidth,minzrefineogm.ogmresolution)

              , farrefinedogm(40,40,0.2)
              , farminzogm(40,40,1.0)
              , laneogm(20,20,0.2)
              , intensityogm(40,40,0.1)
              ,slope_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,circle_slope_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,stiff_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,passible_point_count_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,nopassible_point_count_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,slope_point_count_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,circle_slope_point_count_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
              ,stiff_point_count_ogm(OGMHEIGHT,OGMWIDTH,OGMRESOLUTION)
	      ,fpointcloud_cb_(fpointcloud_cb)
    {
        flag_close=false;

        pointcloud_updateflag=false;
        freeze_ = 0;
        isinited=false;
        initHDL();
    }

        ~LidarProcess();

        void cloud_callback_H (const CloudConstPtr& cloud);

        void cloud_callback (const CloudConstPtr& cloud, float startAngle, float endAngle);

        void keyboard_callback (const KeyboardEvent& event, void* cookie);

        void mouse_callback (const MouseEvent& mouse_event, void* cookie);


        void coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz);
        static void heightdiffOgmDetection(const Cloud& pointcloud,Cloud& heightdiffpointcloud
        		,OGMData<unsigned char>& ogm_data
				,double resolution,double heightdiffthreshold,int countthreshold=0);

        static bool DoorDetection(Cloud& pointcloud,int pointnum_threshold,double xmin,double xmax,double ymin,double ymax,double zmin=0.5,double zmax=2.0);
        static void border_detection(const Cloud& pointcloud,Cloud& borderpointcloud,
        		OGMData<unsigned char>& ogm_data,int LASER_LAYER,double x_offset,double y_offset);
        static void cloud2OGM(const Cloud& pointcloud,OGMData<unsigned char>& ogm_data,int countthredshold=5);
        void ogmDetection();
        void heightdiffDetection();
        void circletangentialDetection();
        void circleradiusDetection();


        int process ();
        int computepointcloud();
        void setpointcloud(const CloudConstPtr& cloud);
        const Cloud& getpointcloudaftercompute ();
        int pointCloudProcess ();
        void getRoadEdgePoints(Cloud& ,Cloud& ,int mode=2/*获取边界特征*/);
        void buildIntegral();
        void getCloudSections();
        void getRoadPath();
        void DrawBoundary();
//        float getSectionOverlap(const CloudPointSection& section1 , const CloudPointSection& section2 ,float& beginangle ,float& endangle);
        float getAngleOverlap(float s1anglebegin ,float s1angleend ,float s2anglebegin,float s2angleend ,float& beginangle ,float& endangle);
		std::vector<float> getAngleOverlap(float s1anglebegin ,float s1angleend ,float s2anglebegin,float s2angleend ,std::vector<float>& beginangle_vec ,std::vector<float>& endangle_vec);

	template<class T>
	  static void OGM2Mat(const OGMData<T>& ogmdata,cv::Mat& img)
	{

	  img=cv::Mat(ogmdata.ogmheight_cell,ogmdata.ogmwidth_cell,CV_32FC1);
	  for(int j=0;j<ogmdata.ogmheight_cell;j++)
	  {
	      float* pdata = img.ptr<float>(j);
	      for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
	      {
	          float val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];

	          pdata[i] = val;
	      }

	  }

	  cv::normalize(img,img,0,1,cv::NORM_MINMAX);//归一化
	  cv::flip(img,img,0);

	}

	template<class T>
	static void showOGM(const char* windowname , const OGMData<T>& ogmdata);
        template<class T>
            void showHaarOGM(const char* windowname , const OGMData<T>& ogmdata);

        void haarGradient(const OGMData<float>& minzogm,OGMData<float>& slopexogm, OGMData<float>& slopeyogm , int window_halfnum, float slopeminthreshold, float heightdiffminthreshold =0.3);
        void haarProcess();
        void ogmProcess ();
        void splitPointCloud();
        void initHDL();

        void displayPointCloud();


        pcl::VelodyneGrabber& grabber_H_;
        boost::mutex cloud_mutex_H_;
        CloudConstPtr cloud_H_;


        UCHAR freeze_;

        bool replay_;
        bool display_;
        bool flag_close;
        float rigid_heightdiffthreshold_;


        bool flag_stopprocess;

        float costtime;

        XmlConf& xmlconfig_;

        OGMData<unsigned char> hdl_ogm_data;
        OGMData<unsigned char> laneogm;
    private:
        boost::signals2::connection cloud_connection_H;
        boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)>* fpointcloud_cb_;
        double alfa_h ;
        double beta_h ;
        double gama_h ;
	int LASER_LAYER;
        double x_offset_h ;
        double y_offset_h ;
        double z_offset_h ;
        pcl::CalibrationValue calibvalue_;
        Eigen::Matrix4f transform_matrix_calibration_H2car_angle ;

        OGMData<unsigned char> refineogm;

        OGMData<float> minzogm;
        OGMData<float> minzrefineogm;
        OGMData<float> minzslopeogm;

        OGMData<unsigned char> farrefinedogm;
        //OGMData<unsigned char> farrefinedogm;

        OGMData<float> farminzogm;

        //lane ogm

        OGMData<unsigned char> intensityogm;
        OGMData<unsigned int> slope_ogm;
        OGMData<int> circle_slope_ogm;
        OGMData<unsigned int> stiff_ogm;

        OGMData<unsigned int> passible_point_count_ogm;
        OGMData<unsigned int> nopassible_point_count_ogm;
        OGMData<int> slope_point_count_ogm;
        OGMData<int> circle_slope_point_count_ogm;
        OGMData<unsigned int> stiff_point_count_ogm;
        OGMData<float> slope5xogm;
        OGMData<float> slope5yogm;
        OGMData<float> slope2xogm;
        OGMData<float> slope2yogm;



        float* laneminintensity_ogm ;
        float* lanemaxintensity_ogm ;
        unsigned char* maxintensity_ogm ;
        unsigned char* minintensity_ogm ;
        bool isinited;
        IplImage *intensityogm_img ;
        Cloud::Ptr  velodyne_pointcloud;//筛选后的激光点
        Cloud::Ptr passablecloud;//
        Cloud::Ptr rigid_nopassablecloud;//
        Cloud::Ptr positiveslopecloud;//
        Cloud::Ptr negativeslopecloud;//
        Cloud::Ptr boundarycloud;
        Cloud::Ptr roadorientationcloud;


        //CloudPointSections* cloudsections;
        std::vector<CloudPointSections> cloudsections;
        float ogm_y_offset ;//向y轴负方向移动了20
        float farogm_y_offset ;//向y轴负方向移动了20 - ogmheightm
        float laneogm_y_offset ;//向y轴负方向移动了10m
        bool flag_ogmdetection;
        bool flag_diffdetection;
        bool flag_circletangential;
        bool flag_circleradius;
        double  theorydis[MAX_LAYER];
        double actualdis[MAX_LAYER];
        double anglerange[MAX_LAYER];
        RoadBoundaries roadboundaries;
//        std::vector<std::vector<int> > roadpath;
        RoadPaths roadpaths;
//        st_tree::tree<int> sectiontree;

        boost::thread* thread_displayPointCloud;
        bool pointcloud_updateflag;
        PolarPointDI polaraxismat_[MAX_LAYER][3601];
        int passable_count_integral[MAX_LAYER][3601];
        double passable_height_integral[MAX_LAYER][3601];
        double passable_distance_integral[MAX_LAYER][3601];
//        int nopassable_integral[LASER_LAYER][3601];
//        int unknown_integral[LASER_LAYER][3601];
        fstream fs_boundary;
};


#endif

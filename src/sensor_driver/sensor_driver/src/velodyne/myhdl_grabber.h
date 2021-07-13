/*!
* \file myhdl_grabber.h
* \brief velodyne雷达数据获取及解析驱动
*
*该文件是单个velodyne雷达数据获取及解析驱动
*
* \author
* \version v1.2.1
* \date 2018/11/23
*/
#define PCL_NO_PRECOMPILE

#include "pcl/pcl_config.h"

#ifndef PCL_IO_HDL_GRABBER_H_
#define PCL_IO_HDL_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <glog/logging.h>
#include "io/iv_grabber.h"
#include "mytime.h"



    namespace pcl
{

    class /*PCL_EXPORTS*/ VelodyneGrabber : public IVGrabber
    {
        typedef unsigned char uchar;
        public:
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyz) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
        //    float, float);
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
        //    float, float);
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzi) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
        //    float startAngle, float);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0"
         */
        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyz) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
         *         This signal is sent when the Velodyne passes angle "0"
         */
        typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzi) (
                const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& , int id);

        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzil) (
        //	const boost::shared_ptr<const pcl::PointCloud<hdlPoint> >&);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
         */
        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);

        /** \brief Constructor taking an optional path to an HDL corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
         * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This parameter is mandatory for the HDL-64, optional for the HDL-32
         * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
         *///new
        VelodyneGrabber (const std::string& correctionsFile = "",
                const std::string& pcapFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
         * \param[in] ipAddress IP Address that should be used to listen for HDL packets
         * \param[in] port UDP Port that should be used to listen for HDL packets
         * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
         *///new
        VelodyneGrabber (const boost::asio::ip::address& ipAddress,
                const unsigned short port, const std::string& correctionsFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
        virtual ~VelodyneGrabber () throw ();

        /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
        virtual void start ();

        /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
        virtual void stop ();

        void pause();

        void resume();

        bool getpausestate();

        /** \brief Obtains the name of this I/O Grabber
         *  \return The name of the grabber
         */
        virtual std::string getName () const;

        /** \brief Check if the grabber is still running.
         *  \return TRUE if the grabber is running, FALSE otherwise
         */
        virtual bool isRunning () const;

        /** \brief Returns the number of frames per second.
        */
        virtual float getFramesPerSecond () const;

        /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
         *         This can be used, for instance, if multiple HDL LIDARs are on the same network
         */
        void filterPackets (const boost::asio::ip::address& ipAddress,
                const unsigned short port = 443);

        /** \brief Allows one to customize the colors used for each of the lasers.
        */
        void setLaserColorRGB (const pcl::RGB& color, unsigned int laserNumber);

        /** \brief Any returns from the HDL with a distance less than this are discarded.
         *         This value is in meters
         *         Default: 0.0
         */
        void setMinimumDistanceThreshold(float & minThreshold);

        /** \brief Any returns from the HDL with a distance greater than this are discarded.
         *         This value is in meters
         *         Default: 10000.0
         */
        void setMaximumDistanceThreshold(float & maxThreshold);

        /** \brief Returns the current minimum distance threshold, in meters
        */

        float getMinimumDistanceThreshold();

        /** \brief Returns the current maximum distance threshold, in meters
        */
        float getMaximumDistanceThreshold();

        long getframecount();

        void setCalibration(const CalibrationValue& value);
        void setDataFromExtern()
        {
        	externdatamode_ = true;
        }

        inline bool externenqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived)
        {
        	if(externdatamode_)
        		return enqueueHDLPacket(data,bytesReceived);
        	else
        		return false;
        }

        protected:
        static const int HDL_DATA_PORT = 2368;
        static const int HDL_NUM_ROT_ANGLES = 36001;
        static const int HDL_LASER_PER_FIRING = 32;
        static const int HDL_FIRING_PER_PKT = 12;
        static const boost::asio::ip::address HDL_DEFAULT_NETWORK_ADDRESS;

        enum HDLBlock
        {
            BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
        };

#pragma pack(push, 1)
        typedef struct HDLLaserReturn
        {
            unsigned short distance;
            unsigned char intensity;
        } HDLLaserReturn;
#pragma pack(pop)

        struct HDLFiringData
        {
            unsigned short blockIdentifier;
            unsigned short rotationalPosition;
            HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
        };

        struct HDLDataPacket
        {
            HDLFiringData firingData[HDL_FIRING_PER_PKT];
            unsigned int gpsTimestamp;
            unsigned char blank1;
            unsigned char blank2;
        };

        struct HDLLaserCorrection
        {
            double azimuthCorrection;
            double verticalCorrection;
            double distanceCorrection;
            double distCorrectionX;//两点矫正
            double distCorrectionY;//两点矫正
            double verticalOffsetCorrection;
            double horizontalOffsetCorrection;
            double sinVertCorrection;
            double cosVertCorrection;
            double sinVertOffsetCorrection;
            double cosVertOffsetCorrection;

            ////xiao add 0105
            uchar minIntensity;
            uchar maxIntensity;
            double focalSlope;
            int focalDistance;
        };

        private:
        static double *cos_lookup_table_;//!<余弦值查询表
        static double *sin_lookup_table_;//!<正弦值查询表
        pcl::SynchronizedQueue<unsigned char *> hdl_data_;//!<雷达数据
        boost::asio::ip::udp::endpoint udp_listener_endpoint_;//!<udp接受
        boost::asio::ip::address source_address_filter_;//!<雷达源地址滤波器
        unsigned short source_port_filter_;//!<源端口
        boost::asio::io_service hdl_read_socket_service_;//!<ｓｏｃｋｅｔ服务
        boost::asio::ip::udp::socket *hdl_read_socket_;//!<ｓｏｃｋｅｔ服务
        std::string pcap_file_name_;//!<ｐｃａｐ文件名
        boost::thread *queue_consumer_thread_;//!<数据解析线程
        boost::thread *hdl_read_packet_thread_;//!<数据接受线程
        HDLLaserCorrection laser_corrections_[MAX_NUM_LASERS];//!<雷达内参
        bool terminate_read_packet_thread_;//!<暂定读包
        bool read_packet_thread_paused_;//!<暂定读包线程
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_sweep_xyzi_;//!<原始数据一整帧
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_calibrated_xyzi_;//!<矫正数据一整帧
        unsigned int last_azimuth_;//!<上一次的雷达角度
        unsigned int last_azimuth_real_;//!<上一帧真实雷达角度
        boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;//!<一帧完成触发信号
        int framecount;//!<帧数

        CalibrationValue calibrationvalue_;//!<外参
        Eigen::Matrix4f calibration_transform_matrix_;//!<外参矩阵
        bool calibrated_;//!<是否矫正
        pcl::RGB laser_rgb_mapping_[MAX_NUM_LASERS];//!<雷达线与ｒｇｂ的映射
        float min_distance_threshold_;//!<最小距离阈值
        float max_distance_threshold_;//!<最大距离阈值
        int LASER_LAYER;//!<雷达线数
        boost::shared_ptr<std::vector<int> > synchornizationcontainor_;//!<多雷达同步器
        int id_;//!<雷达ｉｄ
        bool externdatamode_;//!<外部数据模式
        void processVelodynePackets ();
        bool enqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived);
        void initialize (const std::string& correctionsFile);
        void loadCorrectionsFile (const std::string& correctionsFile);

        void loadVLP16Corrections ();
        void loadHDL32Corrections ();
        void readPacketsFromSocket ();
#ifdef HAVE_PCAP
        MyTime timecounter;
        void readPacketsFromPcap();
        void readPacketsFromPcapold();
        void readPacketsFromPcapwithins ();
#endif //#ifdef HAVE_PCAP
        struct struct_FONSData
       	  {
       		  double dHeading;
       		  double dPitch;
       		  double dRoll;

       		  double dAccx;
       		  double dAccy;
       		  double dAccz;

       		  double dLat;
       		  double dLng;
       		  double dAltitude;

       		  double dState;
       		  char GPSState;

       		  UCHAR				ucStarNum;
       		  UCHAR				ucFONSState;

       		  UCHAR				ucStaThreshold;		//ECU通信状态阈值
       		  UCHAR				ucStaTimes;			//ECU通信状态计数
       		  UCHAR				ucStaOK;			//ECU通信状态

       		  double				dTimeStamp;			//IMU时间戳

       		  double				m_AHRSHeading;			//IMU发来的Heading
       		  double				dIPitch;			//IMU发来的Pitch
       		  double				dIRoll;				//IMU发来的Roll


       		  double				dGyroX;				//IMU发来的绕X向Angular rate
       		  double				dGyroY;				//IMU发来的绕Y向Angular rate
       		  double				dGyroZ;				//IMU发来的绕Z向Angular rate


       		  double				dVdd;				//IMU发来的电压
       		  double				dTemp;				//IMU发来的平均温度



       		  UCHAR				ucOStaThreshold;		//IMU通信状态阈值
       		  UCHAR				ucOStaTimes;			//IMU通信状态计数
       		  UCHAR				ucOStaOK;			//IMU通信状态


       		  //*****新加  fons ASCII 码协议部分
       		  double              dRunTime;           //FONS 上电运行时间
       		  double              dUTC;                //FONS utc世界协调时间
       		  double              dEastV;             // FONS  东向速度
       		  double              dNorthV;            // FONS  北向速度
       		  double              dAltitudeV;         // 天向速度

       	  };
       	struct_FONSData m_sFONSData;//!<惯导数据
       	unsigned int last_time_;//!<上一帧时间
        uint32_t scanCounter_;//!<扫描次数
        uint32_t sweepCounter_;//!<帧数
        void OctansData(const unsigned char * OctansData,std::size_t n);//!<
        void toPointClouds (HDLDataPacket *dataPacket);//!<
        void fireCurrentSweep ();//!<
        void computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth,
                HDLLaserReturn laserReturn, HDLLaserCorrection correction);//!<
        void calibratePoint (pcl::PointXYZI& point);//!<
        bool isAddressUnspecified (const boost::asio::ip::address& ip_address);//!<

        public:
        std::map<double,int> map_tanangle_index;//!<角度与线编号映射　sort by angle
    };
}

#endif /* PCL_IO_HDL_GRABBER_H_ */

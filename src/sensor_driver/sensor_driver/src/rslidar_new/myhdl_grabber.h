
/*!
* \file myhdl_grabber.h
* \brief velodyne雷达数据获取及解析驱动
*
*该文件是单个velodyne雷达数据获取及解析驱动
*各函数及变量功能请参照velodyne/myhdl_grabber.h 文件
* \author
* \version v1.2.1
* \date 2018/11/23
*/
#define PCL_NO_PRECOMPILE

#include "pcl/pcl_config.h"

#ifndef PCL_IO_RSLIDAR_GRABBER_H_
#define PCL_IO_RSLIDAR_GRABBER_H_

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

#include "io/iv_grabber.h"
#include <glog/logging.h>

    namespace pcl
{

    class /*PCL_EXPORTS*/ RslidarGrabber : public IVGrabber
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
        RslidarGrabber (const std::string& correctionsFile = "",
                const std::string& pcapFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
         * \param[in] ipAddress IP Address that should be used to listen for HDL packets
         * \param[in] port UDP Port that should be used to listen for HDL packets
         * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
         *///new
        RslidarGrabber (const boost::asio::ip::address& ipAddress,
                const unsigned short port, const std::string& correctionsFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
        virtual ~RslidarGrabber () throw ();

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

        bool externenqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived);

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
      	    uint8_t distance_1;//这里也是根据RS提供的程序做出的修改
      	    uint8_t distance_2;//这个和用户手册有些出入，用户手册上上面这两个是放在一起的，是一个函数
            unsigned char intensity;
        } HDLLaserReturn;
#pragma pack(pop)

        struct HDLFiringData
        {
            unsigned short blockIdentifier;
            uint8_t rotation_1;//2这里和那个数据手册上的内容有所出入，这里是根据RS提供的程序做的修改
            uint8_t rotation_2;//2
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
            double hori_angle;
            double g_ChannelNum[51];
            ////xiao add 0105
            uchar minIntensity;
            uchar maxIntensity;
            double focalSlope;
            int focalDistance;
        };

        private:
        static const int TEMPERATURE_MIN = 31;
        static const int TEMPERATURE_RANGE = 40;

        static double *cos_lookup_table_;
        static double *sin_lookup_table_;
        int temper_index_;
        pcl::SynchronizedQueue<unsigned char *> hdl_data_;
        boost::asio::ip::udp::endpoint udp_listener_endpoint_;
        boost::asio::ip::address source_address_filter_;
        unsigned short source_port_filter_;
        boost::asio::io_service hdl_read_socket_service_;
        boost::asio::ip::udp::socket *hdl_read_socket_;
        std::string pcap_file_name_;
        boost::thread *queue_consumer_thread_;
        boost::thread *hdl_read_packet_thread_;
        HDLLaserCorrection laser_corrections_[MAX_NUM_LASERS];
        bool terminate_read_packet_thread_;
        bool read_packet_thread_paused_;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_sweep_xyzi_;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_calibrated_xyzi_;
        unsigned int last_azimuth_;
        unsigned int last_azimuth_real_;
        boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
        int framecount;

        CalibrationValue calibrationvalue_;
        Eigen::Matrix4f calibration_transform_matrix_;
        bool calibrated_;
        pcl::RGB laser_rgb_mapping_[MAX_NUM_LASERS];
        float min_distance_threshold_;
        float max_distance_threshold_;
        int LASER_LAYER;
        boost::shared_ptr<std::vector<int> > synchornizationcontainor_;
        int id_;
        int temp_count;
        bool externdatamode_;
        void processVelodynePackets ();
        bool enqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived);
        void initialize (const std::string& correctionsFile);
        void loadCorrectionsFile (const std::string& correctionsFile);

        void loadrslidar16Corrections (std::string config_dir);
        void loadConfigFile(std::string config_dir);
        void readPacketsFromSocket ();

       	unsigned int last_time_;
        uint32_t scanCounter_;
        uint32_t sweepCounter_;
        void OctansData(const unsigned char * OctansData,std::size_t n);
        void toPointClouds (HDLDataPacket *dataPacket);
        void fireCurrentSweep ();
        void computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth,
                HDLLaserReturn laserReturn, HDLLaserCorrection correction);
        int correctAzimuth(float azimuth_f, float hori_correction);
        void calibratePoint (pcl::PointXYZI& point);
        bool isAddressUnspecified (const boost::asio::ip::address& ip_address);
        void settemperindex(unsigned char bit1,unsigned char bit2);

    };
}

#endif /* PCL_IO_RSLIDAR_GRABBER_H_ */

/*!
* \file myhdl_grabber.cpp
* \brief velodyne雷达数据获取及解析驱动
*
*该文件是单个velodyne雷达数据获取及解析驱动
*
* \author
* \version v1.2.1
* \date 2018/11/23
*/
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include "myhdl_grabber.h"
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/ip.h>
#include <netinet/udp.h>

#ifdef HAVE_PCAP
#include <pcap.h>
#include <pcap/sll.h>
#endif // #ifdef HAVE_PCAP

const boost::asio::ip::address pcl::VelodyneGrabber::HDL_DEFAULT_NETWORK_ADDRESS = boost::asio::ip::address::from_string ("192.168.3.255");
double *pcl::VelodyneGrabber::cos_lookup_table_ = NULL;
double *pcl::VelodyneGrabber::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;


///////////////////////////////////////////////////////////////////////////////new
pcl::VelodyneGrabber::VelodyneGrabber (const std::string& correctionsFile,
        const std::string& pcapFile, int laser_layer
		,int id,boost::shared_ptr<std::vector<int> > synchornizationcontainor)
    : hdl_data_ ()
    , udp_listener_endpoint_ (HDL_DEFAULT_NETWORK_ADDRESS, HDL_DATA_PORT)
    , source_address_filter_ ()
    , source_port_filter_ (443)
    , hdl_read_socket_service_ ()
    , hdl_read_socket_ (NULL)
    , pcap_file_name_ (pcapFile)
    , queue_consumer_thread_ (NULL)
    , hdl_read_packet_thread_ (NULL)
    , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
    , last_azimuth_ (0)
	, last_azimuth_real_(0)
    , sweep_xyzi_signal_ ()
    , min_distance_threshold_(0.0)
    , max_distance_threshold_(10000.0)
    , read_packet_thread_paused_(false)
	, synchornizationcontainor_(synchornizationcontainor)
	, id_(id)
{

    //	if(laser_layer == 16)
    //		LASER_LAYER = 16;
    //	else
    LASER_LAYER = laser_layer;
    initialize (correctionsFile);

}

///////////////////////////////////////////////////////////////////////////////new
pcl::VelodyneGrabber::VelodyneGrabber (const boost::asio::ip::address& ipAddress,
        const unsigned short int port,
        const std::string& correctionsFile, int laser_layer
		,int id,boost::shared_ptr<std::vector<int> > synchornizationcontainor)
    : hdl_data_ ()
    , udp_listener_endpoint_ (ipAddress, port)
    , source_address_filter_ ()
    , source_port_filter_ (443)
    , hdl_read_socket_service_ ()
    , hdl_read_socket_ (NULL)
    , pcap_file_name_ ()
    , queue_consumer_thread_ (NULL)
    , hdl_read_packet_thread_ (NULL)
    , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
    , last_azimuth_ (0)
	, last_azimuth_real_(0)
    , sweep_xyzi_signal_ ()
    , min_distance_threshold_(0.0)
    , max_distance_threshold_(10000.0)
    , read_packet_thread_paused_(false)
	, synchornizationcontainor_(synchornizationcontainor)
	, id_(id)
{

    //  if(laser_layer == 16)
    //	  LASER_LAYER = 16;
    //  else
    LASER_LAYER = laser_layer;

    initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::VelodyneGrabber::~VelodyneGrabber () throw ()
{

    stop ();

    free(cos_lookup_table_);
    free(sin_lookup_table_);
    disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();

    std::cout<<"~Grabber"<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////
/// \brief 初始化
///
/// 读取标定文件初始化
/// \param correctionsFile　标定文件名
    void
pcl::VelodyneGrabber::initialize (const std::string& correctionsFile)
{
    	last_time_ = 0;
        scanCounter_ = 0;
        sweepCounter_ = 0;
    	externdatamode_ = false;
    	if(synchornizationcontainor_->size()==0)
    	{
//    		synchornizationcontainor_ = boost::make_shared<std::vector<int> >(std::vector<int>());
    		synchornizationcontainor_->push_back(1);
    	}

    if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
    {
        cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
        sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
        for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
        {
            double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
            cos_lookup_table_[i] = std::cos (rad);
            sin_lookup_table_[i] = std::sin (rad);
        }
    }

    loadCorrectionsFile (correctionsFile);

    calibrated_ = false;
    //jkj polar detect 2016/10/22


    for(int index=0;index<MAX_NUM_LASERS;index++)
    {
        if(index<LASER_LAYER)
        {
            double angle=laser_corrections_[index].verticalCorrection;
            //double tan_angle=tan(HDL_Grabber_toRadians(angle));
            map_tanangle_index[angle]=index;     //build mapping
        }
    }
    std::cout<<"lasernum="<<map_tanangle_index.size()<<std::endl;

    int index=0;
    for(std::map<double,int>::iterator iter=map_tanangle_index.begin() ; iter!=map_tanangle_index.end(); iter++)
    {

        indexmaptable[index].number=iter->second;
        indexmaptable[index].angle=iter->first;
        // cout<<index<<"\tlaserindex="<<iter->second<<"\tangle="<<iter->first<<"\ttanangle="<<tan(HDL_Grabber_toRadians(iter->first))<<endl;
        index++;
    }
    //jkj end

    for (int i = 0; i < MAX_NUM_LASERS; i++)
    {
        HDLLaserCorrection correction = laser_corrections_[i];
        laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
            * correction.sinVertCorrection;
        laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
            * correction.cosVertCorrection;
    }
    sweep_xyzi_signal_ =createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
    current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
    current_calibrated_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < MAX_NUM_LASERS; i++)
        laser_rgb_mapping_[i].r = laser_rgb_mapping_[i].g = laser_rgb_mapping_[i].b = 0;

    if (laser_corrections_[32].distanceCorrection == 0.0)
    {
        for (int i = 0; i < 16; i++)
        {
            laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
            laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 6 + 64);
        }
    }
    else
    {
        for (int i = 0; i < 16; i++)
        {
            laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
            laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 3 + 64);
        }
        for (int i = 0; i < 16; i++)
        {
            laser_rgb_mapping_[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
            laser_rgb_mapping_[i * 2 + 33].b = static_cast<uint8_t> ( (i + 16) * 3 + 160);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////new
/// \brief 加载标定文件
///
/// \param correctionsFile　标定文件名
void
pcl::VelodyneGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{

    if (correctionsFile.empty ())
    {
        if(LASER_LAYER == 16)
            loadVLP16Corrections();
        else
            loadHDL32Corrections();
        return;
    }



    boost::property_tree::ptree pt;
    try
    {
        read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    }
    catch (boost::exception const&)
    {
        PCL_ERROR ("[pcl::VelodyneGrabber::loadCorrectionsFile] Error reading calibration file %s!\n", correctionsFile.c_str ());
        return;
    }

    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child ("boost_serialization.DB.points_"))
    {
        if (v.first == "item")
        {
            boost::property_tree::ptree points = v.second;
            BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
            {
                if (px.first == "px")
                {
                    boost::property_tree::ptree calibrationData = px.second;
                    int index = -1;
                    double azimuth = 0, vertCorrection = 0, distCorrection = 0,
                           vertOffsetCorrection = 0, horizOffsetCorrection = 0,focalDistance = 0 , focalSlope = 0;

                    BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
                    {
                        if (item.first == "id_")
                            index = atoi (item.second.data ().c_str ());
                        if (item.first == "rotCorrection_")
                            azimuth = atof (item.second.data ().c_str ());
                        if (item.first == "vertCorrection_")
                            vertCorrection = atof (item.second.data ().c_str ());
                        if (item.first == "distCorrection_")
                            distCorrection = atof (item.second.data ().c_str ());
                        if (item.first == "vertOffsetCorrection_")
                            vertOffsetCorrection = atof (item.second.data ().c_str ());
                        if (item.first == "horizOffsetCorrection_")
                            horizOffsetCorrection = atof (item.second.data ().c_str ());
                        if (item.first == "focalDistance_")
                            focalDistance = atof (item.second.data ().c_str ());
                        if (item.first == "focalSlope_")
                            focalSlope = atof (item.second.data ().c_str ());
                    }
                    if (index != -1)
                    {
                        laser_corrections_[index].azimuthCorrection = azimuth;
                        laser_corrections_[index].verticalCorrection = vertCorrection;
                        laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
                        laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
                        laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;
                        laser_corrections_[index].focalDistance= focalDistance / 100.0;
                        laser_corrections_[index].focalSlope= focalSlope ;
                        laser_corrections_[index].minIntensity= 0 ;
                        laser_corrections_[index].maxIntensity= 255 ;

                        laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
                        laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));

                    }
                }
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////new
/// \brief 加载１６线标定文件
///
    void
pcl::VelodyneGrabber::loadVLP16Corrections ()
{
    double vlp16VerticalCorrections[] = {
        -15, 1, -13, 3, -11,
        5, -9, 7, -7, 9, -5, 11, -3,
        13, -1, 15};
    for (int i = 0; i < 16; i++)
    {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = vlp16VerticalCorrections[i];
        laser_corrections_[i].focalDistance= 0 ;
        laser_corrections_[i].focalSlope= 0 ;
        laser_corrections_[i].minIntensity= 0 ;
        laser_corrections_[i].maxIntensity= 255 ;

        laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
    }
    for (int i = 16; i < MAX_NUM_LASERS; i++)
    {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = 0.0;
        laser_corrections_[i].focalDistance= 0 ;
        laser_corrections_[i].focalSlope= 0 ;
        laser_corrections_[i].minIntensity= 0 ;
        laser_corrections_[i].maxIntensity= 255 ;

        laser_corrections_[i].sinVertCorrection = 0.0;
        laser_corrections_[i].cosVertCorrection = 1.0;
    }
}

//    void
//    pcl::VelodyneGrabber::loadVLP16Corrections ()
//    {
//
//      double vlp16VerticalCorrections48[] = {-15.2763,  -13.1227,  -11.1032,  -9.0204,  -7.0192,  -4.9651,  -3.0410,  -1.0026,  15.0094,  12.8847,  10.9307,  8.8807,  6.8428,  4.9651,  3.0053,  0.9990};//对于角度的确切值
//      double distancecorrection48[]={210,215,210,204,209,201,212,216,216,212,219,227,216,222,214,200};//对于位置的修正值
//      double vlp16VerticalCorrections42[] = {-14.9782,  -13.0116,  -11.0513,  -9.0190,  -7.0059,  -5.0393,  -2.9714,  -0.9674,  15.0518,  12.9946,  11.0306,  9.0365,  6.9847,  5.0144,  2.9964,  1.0211};
//      double distancecorrection42[]={209,  217,  208,  217,  209,  202,  212,  213,  214,  214,  217,  229,  211,  230,  216,  205};
//      double * vlp16VerticalCorrections;
//      double * distancecorrection;
//      if(1)
//      {
//    	  vlp16VerticalCorrections = vlp16VerticalCorrections48;
//    	  distancecorrection = distancecorrection48;
//      }
//      else
//      {
//    	  vlp16VerticalCorrections = vlp16VerticalCorrections42;
//    	  distancecorrection = distancecorrection42;
//      }
//      for (int i = 0; i < 16; i++)
//      {
//        laser_corrections_[i].azimuthCorrection = 0.0;
//        laser_corrections_[i].distanceCorrection =distancecorrection[i];
//        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
//        laser_corrections_[i].verticalOffsetCorrection = 0.0;
//        laser_corrections_[i].verticalCorrection = vlp16VerticalCorrections[i];
//        laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
//        laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
//      }
//      for (int i = 16; i < MAX_NUM_LASERS; i++)
//      {//根据协议可以知道每一个block发送两组数据
//        laser_corrections_[i].azimuthCorrection = 0.0;
//        laser_corrections_[i].distanceCorrection = 0.0;
//        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
//        laser_corrections_[i].verticalOffsetCorrection = 0.0;
//        laser_corrections_[i].verticalCorrection = 0.0;
//        laser_corrections_[i].sinVertCorrection = 0.0;
//        laser_corrections_[i].cosVertCorrection = 1.0;
//      }
//    }

/////////////////////////////////////////////////////////////////////////////
/// \brief 加载３２线标定文件
///
    void
pcl::VelodyneGrabber::loadHDL32Corrections ()
{
    double hdl32VerticalCorrections[] = {
        -30.67, -9.3299999, -29.33, -8, -28,
        -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
        -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
        -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
    for (int i = 0; i < HDL_LASER_PER_FIRING; i++)
    {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
        laser_corrections_[i].focalDistance= 0 ;
        laser_corrections_[i].focalSlope= 0 ;
        laser_corrections_[i].minIntensity= 0 ;
        laser_corrections_[i].maxIntensity= 255 ;

        laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    }
    for (int i = HDL_LASER_PER_FIRING; i < MAX_NUM_LASERS; i++)
    {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].focalDistance= 0 ;
        laser_corrections_[i].focalSlope= 0 ;
        laser_corrections_[i].minIntensity= 0 ;
        laser_corrections_[i].maxIntensity= 255 ;

        laser_corrections_[i].verticalCorrection = 0.0;
        laser_corrections_[i].sinVertCorrection = 0.0;
        laser_corrections_[i].cosVertCorrection = 1.0;
    }
}


/////////////////////////////////////////////////////////////////////////////
/// \brief 处理每个数据包，点云解析
///
    void
pcl::VelodyneGrabber::processVelodynePackets ()
{
    while (true)
    {
        unsigned char *data;
//        LOG(INFO)<<"processVelodynePackets";
//#ifndef _WINDOWS
//        usleep(1);
//#endif
        if(!read_packet_thread_paused_)
        {
            if (!hdl_data_.dequeue (data))
                return;

            toPointClouds (reinterpret_cast<HDLDataPacket *> (data));

            free (data);

        }
        else
        	usleep(1000);
    }
}

///////////////////////////////////////////////////////////////////////////////new
/// \brief 处理每个数据包，点云解析
///
    void
pcl::VelodyneGrabber::toPointClouds (HDLDataPacket *dataPacket)
{

//    if (sizeof (HDLLaserReturn) != 3)
//        return;

    time_t  time_;
    time(&time_);
    time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
//    if(scanCounter_%100==0&&id_==0)
//    LOG(INFO)<<"gpsTimestamp:"<<dataPacket->gpsTimestamp;
    scanCounter_++;
    if(externdatamode_&&last_time_ >= dataPacket->gpsTimestamp&&!(last_time_>18000000&&dataPacket->gpsTimestamp<18000000))
    	return;
    last_time_ = dataPacket->gpsTimestamp;

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
    {
    	if(LASER_LAYER == 64&&i/2%2==1)
    		continue;
        HDLFiringData firingData = dataPacket->firingData[i];
        unsigned int temprotation = firingData.rotationalPosition;
//        temprotation += 9000;
        if(temprotation>=36000)
        	temprotation -=36000;
//        if(scanCounter_%100==0)
//        	LOG(INFO)<<"id:"<<id_<<" rotation:"<<temprotation;
        if(LASER_LAYER == 16)
        {
            //������16����azimuth
            unsigned short interpolated_rotationalPosition = firingData.rotationalPosition;
            if(firingData.rotationalPosition < last_azimuth_real_)
                interpolated_rotationalPosition += (firingData.rotationalPosition + 36000 - last_azimuth_real_) / 2;
            else
                interpolated_rotationalPosition += (firingData.rotationalPosition - last_azimuth_real_) / 2;

            //��һ��16��
            //�µ�һȦ������reset
            if (firingData.rotationalPosition < last_azimuth_||synchornizationcontainor_->at(id_)==2)
            {
            	bool ismaster = (synchornizationcontainor_->at(id_) < 2);
            	synchornizationcontainor_->at(id_)++;
            	if(synchornizationcontainor_->at(id_)>=2)
            	{
					if(current_sweep_xyzi_->size() > 0 )
					{
						current_sweep_xyzi_->is_dense = false;
						current_sweep_xyzi_->header.stamp = velodyneTime;
						current_sweep_xyzi_->header.seq = sweepCounter_;
						if(calibrated_)
						{
							current_calibrated_xyzi_->is_dense = false;
							current_calibrated_xyzi_->header.stamp = velodyneTime;
							current_calibrated_xyzi_->header.seq = sweepCounter_;
						}
						sweepCounter_++;

						fireCurrentSweep ();
					}
					current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
					if(calibrated_)
						current_calibrated_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
					if(ismaster)
					{
						for(int i=0;i<synchornizationcontainor_->size();i++)
							synchornizationcontainor_->at(i)=2;
						synchornizationcontainor_->at(id_) = 1;
					}
					else
						synchornizationcontainor_->at(id_) = 0;
            	}
            }

            for (int j = 0; j < 16; j++)
            {
                PointXYZI xyzi;
                computeXYZI (xyzi, firingData.rotationalPosition, firingData.laserReturns[j], laser_corrections_[j]);
                xyzi.passibility = 1.0;
                last_azimuth_ = temprotation;
                last_azimuth_real_ = firingData.rotationalPosition;
                current_sweep_xyzi_->push_back (xyzi);
                if(calibrated_)
                {
                    calibratePoint(xyzi);
                    current_calibrated_xyzi_->push_back (xyzi);
                }


            }

            {
                //�ڶ���16�㣬ֻ��interpolated_azimuth��Чʱ�ż���
                if(interpolated_rotationalPosition < 36000)
                {
                    for (int j = 16; j<32; j++)
                    {
                        PointXYZI xyzi;
                        computeXYZI (xyzi, interpolated_rotationalPosition, firingData.laserReturns[j], laser_corrections_[j - 16]);
                        xyzi.passibility = 1.0;

                        current_sweep_xyzi_->push_back (xyzi);
                        if(calibrated_)
                        {
                            calibratePoint(xyzi);
                            current_calibrated_xyzi_->push_back (xyzi);
                        }
                    }
                }
            }

        }
        else
        {
            //�ж�һȦ������reset
            int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

            if (offset==0&&((temprotation < last_azimuth_)||synchornizationcontainor_->at(id_)==2))
            {
//            	bool ismaster = (synchornizationcontainor_->at(id_) < 2);
            	bool ismaster = (id_==0);
            	LOG(INFO)<<"id:"<<id_;
            	synchornizationcontainor_->at(id_)++;
            	if(synchornizationcontainor_->at(id_)>=2)
            	{
					if(current_sweep_xyzi_->size() > 0 )
					{
						current_sweep_xyzi_->is_dense = false;
						current_sweep_xyzi_->header.stamp = velodyneTime;
						current_sweep_xyzi_->header.seq = sweepCounter_;
						if(calibrated_)
						{
							current_calibrated_xyzi_->is_dense = false;
							current_calibrated_xyzi_->header.stamp = velodyneTime;
							current_calibrated_xyzi_->header.seq = sweepCounter_;
						}
						sweepCounter_++;

						fireCurrentSweep ();
					}
					current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
					if(calibrated_)
						current_calibrated_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
					if(ismaster)
					{
						for(int i=0;i<synchornizationcontainor_->size();i++)
							synchornizationcontainor_->at(i)=2;
						synchornizationcontainor_->at(id_) = 1;
					}
					else
						synchornizationcontainor_->at(id_) = 0;
            	}
            }

            for (int j = 0; j < 32; j++)
            {
                PointXYZI xyzi;

                computeXYZI (xyzi, firingData.rotationalPosition, firingData.laserReturns[j], laser_corrections_[j + offset]);
                xyzi.passibility = 1.0;


                current_sweep_xyzi_->push_back (xyzi);
                if(calibrated_)
                {
                    calibratePoint(xyzi);
                    current_calibrated_xyzi_->push_back (xyzi);
                }
            }

            if(offset==0)
                last_azimuth_ = temprotation;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////new
/// \brief 点云解析
///
    void
pcl::VelodyneGrabber::computeXYZI (pcl::PointXYZI& point, int azimuth,
        HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
    double cosAzimuth, sinAzimuth;

    double distanceM = laserReturn.distance * 0.002;
    float intensityScale = (float)(correction.maxIntensity - correction.minIntensity);
    float focalSlope = correction.focalSlope;
    float focaloffset = 256*(1-correction.focalDistance/13100)*(1-correction.focalDistance/13100);
    point.intensity = 0;

    if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) {
        //point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
//    	std::cout<<distanceM<<std::endl;
        point.intensity = -1;// static_cast<float> (laserReturn.intensity);
        point.azimuth = static_cast<float> (azimuth) / 100.0;
        point.range = -0.1;

    	if((distanceM < 0.00001 || distanceM > max_distance_threshold_)&&correction.sinVertCorrection>-0.1)
    		distanceM = max_distance_threshold_;
    	else
    	{
            point.x = point.y = point.z = 0.1;

            return;
    	}

    }

    if (correction.azimuthCorrection == 0)
    {
        cosAzimuth = cos_lookup_table_[azimuth];
        sinAzimuth = sin_lookup_table_[azimuth];
    }
    else
    {
        double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
        cosAzimuth = std::cos (azimuthInRadians);
        sinAzimuth = std::sin (azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;

    double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;
    //  float intensityVal = laserReturn.intensity;

    float intensityVal = laserReturn.intensity  + focalSlope*abs((focaloffset - 256*((1-laserReturn.distance)/65535)*((1-laserReturn.distance)/65535)));
    if (intensityVal < correction.minIntensity)
        intensityVal = correction.minIntensity;
    if (intensityVal > correction.maxIntensity)
        intensityVal = correction.maxIntensity;
	point.x = static_cast<float> (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
	point.y = static_cast<float> (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
	point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);

    if(point.intensity > -0.5)
    {
    	point.intensity = static_cast<float> (intensityVal);
    	point.azimuth = static_cast<float> (azimuth) / 100.0;
    	point.range = static_cast<float>(distanceM);
    }


}

/// \brief 矫正数据
///
    void
pcl::VelodyneGrabber::calibratePoint (pcl::PointXYZI& point)
{
	if(point.range > 0)
	{
		float azimuth = 90 - point.azimuth  + calibrationvalue_.gama;
		while(azimuth > 360.0) azimuth -=360.0;
		while(azimuth <=0) azimuth +=360.0;
		point.azimuth=azimuth;
		point.passibility = 1.0;

		point.getVector3fMap () = calibration_transform_matrix_.block<3,3>(0,0) * point.getVector3fMap ()
										+ calibration_transform_matrix_.block<3,1>(0,3);
		if((point.x > -fabs(1.5) && point.x < fabs(1.5) &&
			  point.y < 4 && point.y > -2))
		{
		  point.range =- 0.01;
		}
	}
	else
	{
		float azimuth = 90 - point.azimuth  + calibrationvalue_.gama;
		while(azimuth > 360.0) azimuth -=360.0;
		while(azimuth <=0) azimuth +=360.0;
		point.azimuth=azimuth;
		point.passibility = 1.0;
	}


}
/////////////////////////////////////////////////////////////////////////////
    /// \brief 雷达扫描完一圈发送一次
    ///
    void
pcl::VelodyneGrabber::fireCurrentSweep ()
{
    pcl_conversions::toPCL(ros::Time::now(), current_sweep_xyzi_->header.stamp);//us
    LOG(INFO)<<"signal operator start";
    if (sweep_xyzi_signal_->num_slots () > 0)
        sweep_xyzi_signal_->operator() (current_sweep_xyzi_,2*id_);
    LOG(INFO)<<"signal operator end";
    if(calibrated_)
    {
    	current_calibrated_xyzi_->header = current_sweep_xyzi_->header;

//    	if(LASER_LAYER == 64)
//    	{
//			for(int j=0;j<64;j++)
//			{
//				int index_j = indexmaptable[j].number;
//				for(int i=0;i<current_calibrated_xyzi_->size()/64;i++)
//				{
//					if(j%4!=2)
//					{
//						current_calibrated_xyzi_->at(i*64+index_j).x = 0;
//						current_calibrated_xyzi_->at(i*64+index_j).y = 0;
//						current_calibrated_xyzi_->at(i*64+index_j).z = 0;
//						current_calibrated_xyzi_->at(i*64+index_j).range = -1;
//						current_calibrated_xyzi_->at(i*64+index_j).intensity = -1;
//					}
//				}
//			}
//    	}

        if (sweep_xyzi_signal_->num_slots () > 0)
            sweep_xyzi_signal_->operator() (current_calibrated_xyzi_,2*id_+1);
    }
//    static pcl::uint32_t counter = 0;
//    counter ++;
//    sensor_msgs::PointCloud2 PointCloudMsg;
//    pcl_conversions::toPCL(ros::Time::now(), current_sweep_xyzi_->header.stamp);//us
//    current_sweep_xyzi_->header.frame_id="32E1";
//    current_sweep_xyzi_->header.seq = counter;
//    pcl::toROSMsg(*current_sweep_xyzi_, PointCloudMsg);
//
//    PointCloudMsg.header.frame_id = "/camera";
//    pubPointCloud.publish(PointCloudMsg);
}

/////////////////////////////////////////////////////////////////////////////
    /// \brief 将收到的数据推入队列中
    ///
    bool
pcl::VelodyneGrabber::enqueueHDLPacket (const unsigned char *data,
        std::size_t bytesReceived) //jkj 2017/1/2
{
    if(bytesReceived ==1208)
    	bytesReceived = 1206;
    if (bytesReceived == 1206)
    {
        unsigned char *dup = static_cast<unsigned char *> (malloc (bytesReceived * sizeof(unsigned char)));
        memcpy (dup, data, bytesReceived * sizeof(unsigned char));

        hdl_data_.enqueue (dup);
        return true;
    }
    return false;
}


/////////////////////////////////////////////////////////////////////////////
    /// \brief 建立解析点云和接受数据的线程
    ///
    void
pcl::VelodyneGrabber::start ()
{
    terminate_read_packet_thread_ = false;


    if (isRunning ())
        return;


    queue_consumer_thread_ = new boost::thread (boost::bind (&VelodyneGrabber::processVelodynePackets, this));

    if(externdatamode_)
    	return;
    if (pcap_file_name_.empty ())
    {
//    	std::cout<<"socket"<<std::endl;
        try
        {
            try {
                hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp_listener_endpoint_);
            }
            catch (std::exception bind) {
                delete hdl_read_socket_;
                hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp::endpoint(boost::asio::ip::address_v4::any(), udp_listener_endpoint_.port()));
            }
            hdl_read_socket_service_.run ();
        }
        catch (std::exception &e)
        {
            PCL_ERROR ("[pcl::VelodyneGrabber::start] Unable to bind to socket! %s\n", e.what());
            return;
        }
//        std::cout<<"socket2"<<std::endl;
        hdl_read_packet_thread_ = new boost::thread (boost::bind (&VelodyneGrabber::readPacketsFromSocket, this));
    }
    else
    {
#ifdef HAVE_PCAP
        hdl_read_packet_thread_ = new boost::thread(boost::bind(&VelodyneGrabber::readPacketsFromPcap, this));
#endif // #ifdef HAVE_PCAP
    }
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::VelodyneGrabber::stop ()
{

    resume();
    terminate_read_packet_thread_ = true;
    hdl_data_.stopQueue ();

    if (queue_consumer_thread_ != NULL)
    {
        queue_consumer_thread_->join ();
        delete queue_consumer_thread_;
        queue_consumer_thread_ = NULL;
    }

    if (hdl_read_packet_thread_ != NULL)
    {
        //    hdl_read_packet_thread_->interrupt ();
        hdl_read_packet_thread_->join ();
        delete hdl_read_packet_thread_;
        hdl_read_packet_thread_ = NULL;
    }

    if (hdl_read_socket_ != NULL)
    {
        delete hdl_read_socket_;
        hdl_read_socket_ = NULL;
    }
}

void pcl::VelodyneGrabber::pause()
{
    read_packet_thread_paused_ = true;
}

void pcl::VelodyneGrabber::resume()
{
    read_packet_thread_paused_ = false;
}

bool pcl::VelodyneGrabber::getpausestate()
{
    return read_packet_thread_paused_;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::VelodyneGrabber::isRunning () const
{
    return (!hdl_data_.isEmpty() || (hdl_read_packet_thread_ != NULL &&
                !hdl_read_packet_thread_->timed_join (boost::posix_time::milliseconds (10))));
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::VelodyneGrabber::getName () const
{
    return (std::string ("Velodyne High Definition Laser (HDL) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::VelodyneGrabber::getFramesPerSecond () const
{
    return (0.0f);
}

long pcl::VelodyneGrabber::getframecount()
{
	return framecount;
}

void pcl::VelodyneGrabber::setCalibration(const pcl::CalibrationValue& value)
{
	calibrationvalue_ = value;
	pcl::get_transform_matrix(calibrationvalue_ , calibration_transform_matrix_);
	calibrated_ = true;
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::VelodyneGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
        const unsigned short port)
{
    source_address_filter_ = ipAddress;
    source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::VelodyneGrabber::setLaserColorRGB (const pcl::RGB& color,
        unsigned int laserNumber)
{
    if (laserNumber >= MAX_NUM_LASERS)
        return;

    laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
    bool
pcl::VelodyneGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
#if BOOST_VERSION>=104700
    return (ipAddress.is_unspecified ());
#else
    if (ipAddress.is_v4 ())
        return (ipAddress.to_v4 ().to_ulong() == 0);

    return (false);
#endif
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VelodyneGrabber::setMaximumDistanceThreshold(float &maxThreshold) {
    max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VelodyneGrabber::setMinimumDistanceThreshold(float &minThreshold) {
    min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::VelodyneGrabber::getMaximumDistanceThreshold() {
    return(max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::VelodyneGrabber::getMinimumDistanceThreshold() {
    return(min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
/// \brief 从ｓｏｃｋｅｔ接受数据
///
    void
pcl::VelodyneGrabber::readPacketsFromSocket ()
{
    unsigned char data[1500];
    udp::endpoint sender_endpoint;

    while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open())
    {
//    	std::cout<<"receive"<<std::endl;
//    	try{
			size_t length = hdl_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);

			if (isAddressUnspecified (source_address_filter_) )//||
	//                (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
			{
				enqueueHDLPacket (data, length);
			}
//    	}
//        catch (boost::system::system_error& err)
//        {
//          std::cerr << "Error: " << err.what() << std::endl;
//        }
    }
}

/////////////////////////////////////////////////////////////////////////////
#ifdef HAVE_PCAP
    void
pcl::VelodyneGrabber::readPacketsFromPcapold ()//jkj 2017/1/2
{
    struct pcap_pkthdr *header;
    const unsigned char *data;
    char errbuff[PCAP_ERRBUF_SIZE];

    //std::cout<<"readPacketsFromPcap\t"<<pcap_file_name_.c_str ()<<std::endl;
    pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

    struct bpf_program filter;
    std::ostringstream stringStream;

    stringStream << "udp ";
    if (!isAddressUnspecified(source_address_filter_))
    {
        stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
    }

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
    long long lastactualtime;

    struct timeval lasttime;
    long long uSecDelay;

    long long autualtime=0;

    lasttime.tv_sec = 0;
    timecounter.start();
    int packetcounter=0;
    int returnValue=-1;
    returnValue = pcap_next_ex(pcap, &header, &data);
    totaltime.start();
    long long start_tv_sec =header->ts.tv_sec;
    long long start_tv_usec =header->ts.tv_usec;

    while (returnValue >= 0 && !terminate_read_packet_thread_)
    {
        if (lasttime.tv_sec == 0)
        {
            lasttime.tv_sec = header->ts.tv_sec;
            lasttime.tv_usec = header->ts.tv_usec;
        }
        if (lasttime.tv_usec > header->ts.tv_usec)
        {
            lasttime.tv_usec -= 1000000;
            lasttime.tv_sec++;
        }
        packetcounter++;

        //    uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
        //                (header->ts.tv_usec - lasttime.tv_usec);
        totaltime.stop();
        //    timecounter.stop();

        autualtime=static_cast<long long>(totaltime.gettime_s()*1000000);
        uSecDelay=(header->ts.tv_sec-start_tv_sec)*1000000+header->ts.tv_usec-start_tv_usec-autualtime;

        //    autualtime+=timecounter.gettime_s()*1000000;
        // timecounter.start();
        //    file<<"packetcounter="<<packetcounter<<"\ttotaltime="<<totaltime.gettime_s()<<"\tuSecDelay="<<uSecDelay<<"\tautualtime="<<autualtime<<std::endl;
        //    if(uSecDelay>autualtime)
        //    {
        //        uSecDelay=uSecDelay-autualtime;
        //        autualtime=0;
        //    }
        //    else
        //    {
        //    	autualtime=autualtime-uSecDelay;
        //    	uSecDelay=0;
        //    }

        if(uSecDelay>0)
            boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));


        //    lasttime.tv_sec = header->ts.tv_sec;
        //    lasttime.tv_usec = header->ts.tv_usec;

        // The ETHERNET header is 42 bytes long; unnecessary
//        do{
//            returnValue = pcap_next_ex(pcap, &header, &data);
//        }while(	!enqueueHDLPacket(data + 42, header->len - 42));


	if (returnValue>0)
	{

		if (header->len==1248)
		{

			unsigned char *dup = static_cast<unsigned char *> (malloc ((header->len - 42) * sizeof(unsigned char)));
			memcpy (dup, data+42, (header->len-42) * sizeof(unsigned char));
			toPointClouds (reinterpret_cast<HDLDataPacket *> (dup));
			// The ETHERNET header is 42 bytes long; unnecessary
			framecount++;
		}
		if (header->len>180&&header->len<200)
		{
			//std::cout<<"IMU"<<std::endl;
			OctansData(data+42,header->len-42);

			//IMU_DataProcFromFOSN_ASCII(data+42,header->len-42);
		}
		/*
		if (header->len==109)
		{
			ECU_DataProcFromBYD(data+42,header->len-42);
			if (lasttime.tv_sec == 0)
		 {
			 lasttime.tv_sec = header->ts.tv_sec;
			 lasttime.tv_usec = header->ts.tv_usec;
			}
			if (lasttime.tv_usec > header->ts.tv_usec)
			{
				lasttime.tv_usec -= 1000000;
				lasttime.tv_sec++;
			}
			uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
				(header->ts.tv_usec - lasttime.tv_usec);

			boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay+20));

			lasttime.tv_sec = header->ts.tv_sec;
			lasttime.tv_usec = header->ts.tv_usec;
		}*/

	}
	returnValue = pcap_next_ex(pcap, &header, &data);

    }

    pcap_close(pcap);
}
    /// \brief 从ｐｃａｐ中解析数据
    ///
	void pcl::VelodyneGrabber::readPacketsFromPcap()
	{
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

		while (!terminate_read_packet_thread_)
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

		    const ether_header * eth = reinterpret_cast<const ether_header *>(data);
		    u_int16_t ether_type = eth->ether_type;
//		    std::cout<<"ether_type:"<<ntohs(ether_type)<<std::endl;
		    if (ntohs(ether_type) != ETHERTYPE_IP) {
		    	const sll_header * sll = reinterpret_cast<const sll_header *>(data);
		    	ether_type = sll->sll_protocol;
		    	offset += sizeof(sll_header);
		     }
		    else
		    	offset += sizeof(ether_header);
//		    std::cout<<"ether_type:"<<ntohs(ether_type)<<std::endl;
		    if (ntohs(ether_type) != ETHERTYPE_IP) {
		       continue;
		     }


		    const iphdr * ip = reinterpret_cast<const iphdr *>(data + offset);
//		    std::cout<<"ip->version:"<<(int)ip->version<<std::endl;
		    if (ip->version != 4) {
		      continue;
		    }
		    offset += ip->ihl * 4;
		    const udphdr * udp = reinterpret_cast<const udphdr *>(data + offset);
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
			if (port==9903)
			{
				enqueueHDLPacket(data + offset , header->len - offset);
			}


		}
//		std::cout<<"pcap end"<<std::endl;
		pcap_close(pcap);
//		LOG(INFO)<<"pcap end";
	}
//    void
//pcl::readPacketsFromPcap ()
//{
//	struct pcap_pkthdr *header;
//	const unsigned char *data;
//	char errbuff[PCAP_ERRBUF_SIZE];
//
//	//std::cout<<"readPacketsFromPcap\t"<<pcap_file_name_.c_str ()<<std::endl;
//	pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);
//
//	struct bpf_program filter;
//	std::ostringstream stringStream;
//
//	stringStream << "udp ";
//	if (!isAddressUnspecified(source_address_filter_))
//	{
//		stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
//	}
//
//	// PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
//	if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
//	{
//		PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
//	}
//	else if (pcap_setfilter(pcap, &filter) == -1)
//	{
//		PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
//	}
//
//	MyTime totaltime;
//	long long lastactualtime;
//
//	struct timeval lasttime;
//	long long uSecDelay;
//
//	long long autualtime=0;
//
//	lasttime.tv_sec = 0;
//	timecounter.start();
//	int packetcounter=0;
//	int returnValue=-1;
//	returnValue = pcap_next_ex(pcap, &header, &data);
//	totaltime.start();
//	long long start_tv_sec =header->ts.tv_sec;
//	long long start_tv_usec =header->ts.tv_usec;
//	LOG(INFO)<<"readPacketsFromPcap";
//	while (returnValue >= 0 && !terminate_read_packet_thread_)
//	{
//
//		if (lasttime.tv_sec == 0)
//		{
//			lasttime.tv_sec = header->ts.tv_sec;
//			lasttime.tv_usec = header->ts.tv_usec;
//		}
//		if (lasttime.tv_usec > header->ts.tv_usec)
//		{
//			lasttime.tv_usec -= 1000000;
//			lasttime.tv_sec++;
//		}
//		packetcounter++;
//
//		//    uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
//		//                (header->ts.tv_usec - lasttime.tv_usec);
//		totaltime.stop();
//		//    timecounter.stop();
//
//		autualtime=static_cast<long long>(totaltime.gettime_s()*1000000);
//		uSecDelay=(header->ts.tv_sec-start_tv_sec)*1000000+header->ts.tv_usec-start_tv_usec-autualtime;
//
//		//    autualtime+=timecounter.gettime_s()*1000000;
//		// timecounter.start();
//		//    file<<"packetcounter="<<packetcounter<<"\ttotaltime="<<totaltime.gettime_s()<<"\tuSecDelay="<<uSecDelay<<"\tautualtime="<<autualtime<<std::endl;
//		//    if(uSecDelay>autualtime)
//		//    {
//		//        uSecDelay=uSecDelay-autualtime;
//		//        autualtime=0;
//		//    }
//		//    else
//		//    {
//		//    	autualtime=autualtime-uSecDelay;
//		//    	uSecDelay=0;
//		//    }
//
//		if(uSecDelay>0)
//			boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));
//
//
//		//    lasttime.tv_sec = header->ts.tv_sec;
//		//    lasttime.tv_usec = header->ts.tv_usec;
//
//		// The ETHERNET header is 42 bytes long; unnecessary
//	//        do{
//	//            returnValue = pcap_next_ex(pcap, &header, &data);
//	//        }while(	!enqueueHDLPacket(data + 42, header->len - 42));
//
//
//	if (returnValue>0)
//	{
//
//		if (header->len==1248)
//		{
//
//			enqueueHDLPacket(data + 42, header->len - 42);
//		}
//	//    			if (header->len>180&&header->len<200)
//	//    			{
//	//    				//std::cout<<"IMU"<<std::endl;
//	//    				OctansData(data+42,header->len-42);
//	//
//	//    				//IMU_DataProcFromFOSN_ASCII(data+42,header->len-42);
//	//    			}
//		/*
//		if (header->len==109)
//		{
//			ECU_DataProcFromBYD(data+42,header->len-42);
//			if (lasttime.tv_sec == 0)
//		 {
//			 lasttime.tv_sec = header->ts.tv_sec;
//			 lasttime.tv_usec = header->ts.tv_usec;
//			}
//			if (lasttime.tv_usec > header->ts.tv_usec)
//			{
//				lasttime.tv_usec -= 1000000;
//				lasttime.tv_sec++;
//			}
//			uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
//				(header->ts.tv_usec - lasttime.tv_usec);
//
//			boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay+20));
//
//			lasttime.tv_sec = header->ts.tv_sec;
//			lasttime.tv_usec = header->ts.tv_usec;
//		}*/
//
//	}
//	returnValue = pcap_next_ex(pcap, &header, &data);
//
//	}
//	std::cout<<"pcap end"<<std::endl;
//	pcap_close(pcap);
//}

    void
    pcl::VelodyneGrabber::readPacketsFromPcapwithins ()
    {
    	framecount = 0;
      struct pcap_pkthdr *header;
      const unsigned char *data;
      char errbuff[PCAP_ERRBUF_SIZE];

      pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

      struct bpf_program filter;
      std::ostringstream stringStream;

      stringStream << "udp ";
      if (!isAddressUnspecified(source_address_filter_))
      {
        stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
      }

      // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
      if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
      {
        PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
      }
      else if (pcap_setfilter(pcap, &filter) == -1)
      {
        PCL_WARN ("[pcl::VelodyneGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
      }

      struct timeval lasttime;
      unsigned long long uSecDelay;

      lasttime.tv_sec = 0;
      while(!terminate_read_packet_thread_)
      {
    #ifndef _WINDOWS
    	usleep(1);
    #endif
    		if (!read_packet_thread_paused_)
    		{
    			int returnValue = pcap_next_ex(pcap, &header, &data);
    			if (returnValue>0)
    			{

    				if (header->len==1248)
    				{

    					unsigned char *dup = static_cast<unsigned char *> (malloc ((header->len - 42) * sizeof(unsigned char)));
    					memcpy (dup, data+42, (header->len-42) * sizeof(unsigned char));
    					toPointClouds (reinterpret_cast<HDLDataPacket *> (dup));
    					if (lasttime.tv_sec == 0)
    				 {
    					 lasttime.tv_sec = header->ts.tv_sec;
    					 lasttime.tv_usec = header->ts.tv_usec;
    					}
    					if (lasttime.tv_usec > header->ts.tv_usec)
    					{
    						lasttime.tv_usec -= 1000000;
    						lasttime.tv_sec++;
    					}
    					uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
    						(header->ts.tv_usec - lasttime.tv_usec);
    					boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay+300));

    					lasttime.tv_sec = header->ts.tv_sec;
    					lasttime.tv_usec = header->ts.tv_usec;

    					// The ETHERNET header is 42 bytes long; unnecessary
    					framecount++;
    				}
    				if (header->len>180&&header->len<200)
    				{
    					//std::cout<<"IMU"<<std::endl;
    					OctansData(data+42,header->len-42);
    					//IMU_DataProcFromFOSN_ASCII(data+42,header->len-42);
    					if (lasttime.tv_sec == 0)
    					{
    					 lasttime.tv_sec = header->ts.tv_sec;
    					 lasttime.tv_usec = header->ts.tv_usec;
    					}
    					if (lasttime.tv_usec > header->ts.tv_usec)
    					{
    						lasttime.tv_usec -= 1000000;
    						lasttime.tv_sec++;
    					}
    					uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
    						(header->ts.tv_usec - lasttime.tv_usec);

    					boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay+20));

    					lasttime.tv_sec = header->ts.tv_sec;
    					lasttime.tv_usec = header->ts.tv_usec;
    				}
    				/*
    				if (header->len==109)
    				{
    					ECU_DataProcFromBYD(data+42,header->len-42);
    					if (lasttime.tv_sec == 0)
    				 {
    					 lasttime.tv_sec = header->ts.tv_sec;
    					 lasttime.tv_usec = header->ts.tv_usec;
    					}
    					if (lasttime.tv_usec > header->ts.tv_usec)
    					{
    						lasttime.tv_usec -= 1000000;
    						lasttime.tv_sec++;
    					}
    					uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
    						(header->ts.tv_usec - lasttime.tv_usec);

    					boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay+20));

    					lasttime.tv_sec = header->ts.tv_sec;
    					lasttime.tv_usec = header->ts.tv_usec;
    				}*/

    			}
    			else
    				break;
    		}


      }
    }


#endif //#ifdef HAVE_PCAP
void pcl::VelodyneGrabber::OctansData(const unsigned char * OctansData,std::size_t n)
{

  double dTemp = 0.0;
  double dTemp1 = 0.0,dTemp2 = 0.0;
  UINT  unTemp = 0;
  long  lTemp = 0;
  //	CString strTemp;

  int i = 0;
  int j = 0;
  int OctansCommaNumber = 0;
  char OctansDataTemp[150];

  int ccounter=0;//20170314

  memset(OctansDataTemp,0,30);

  while(i++ <= n)
  {

	  if(OctansData[i] == ',')		//找到逗号
	  {
		  j = 0;
		  OctansCommaNumber++;
		  switch(OctansCommaNumber)
		  {
		  case 1:	// $HEHDT
			  memset(OctansDataTemp,0,30);
			  break;
		  case 2: // Course over ground

			  dTemp = atof(OctansDataTemp);

			  m_sFONSData.dHeading=dTemp;
  //				strTemp.Format("   %f",dTemp);
			  memset(OctansDataTemp,0,30);
			  break;
		  case 3: // Magnetic course over ground
			  memset(OctansDataTemp,0,30);
			  break;
		  case 4: //
			  dTemp=atof(OctansDataTemp);
			  m_sFONSData.dPitch=dTemp;//20170222 ltf

			  memset(OctansDataTemp,0,30);
			  break;

		  case 5://+ -
			  if (OctansData[i-1]=='M')
				  m_sFONSData.dPitch=m_sFONSData.dPitch;
			  else
				  m_sFONSData.dPitch=-1*m_sFONSData.dPitch;
			  memset(OctansDataTemp,0,30);
//  std::cout<<"dPitch:"<<m_sFONSData.dPitch <<std::endl;
			  break;
		  case 6: //
			  dTemp=atof(OctansDataTemp);
			  m_sFONSData.dRoll=dTemp;//20170222 ltf

			  memset(OctansDataTemp,0,30);
			  break;

		  case 7://+ -
			  if (OctansData[i-1]=='T')
				  m_sFONSData.dRoll=m_sFONSData.dRoll;
			  else
				  m_sFONSData.dRoll=-1*m_sFONSData.dRoll;
			  memset(OctansDataTemp,0,30);
//  std::cout<<"dRoll:"<<m_sFONSData.dRoll <<std::endl;
			  break;

		  case 8://v
			  dTemp=atof(OctansDataTemp);
			  //m_sFONSData.dLng=dTemp;
			  memset(OctansDataTemp,0,30);
			  break;

		  case 9://v
			  memset(OctansDataTemp,0,30);
			  break;

		  case 10://v
			  memset(OctansDataTemp,0,30);
			  break;

		  case 11://acc
			  dTemp=atof(OctansDataTemp);
			  m_sFONSData.dAccx=dTemp;
			  memset(OctansDataTemp,0,30);
			  break;

		  case 12://acc
			  dTemp=atof(OctansDataTemp);
			  m_sFONSData.dAccy=dTemp;
			  memset(OctansDataTemp,0,30);
			  break;

		  case 13://acc
			  dTemp=atof(OctansDataTemp);
			  m_sFONSData.dAccz=dTemp;
			  memset(OctansDataTemp,0,30);
			  break;

		  case 14://v
			  memset(OctansDataTemp,0,30);
			  break;

		  case 15://v
			  memset(OctansDataTemp,0,30);
			  break;

		  default: break;
		  }
	  }
	  else
	  {
		  if(OctansCommaNumber <15 ) {
				  OctansDataTemp[j++] = OctansData[i];
		  }
	  }
	//memset(OctansDataTemp,0,15);
  }
  m_sFONSData.dTimeStamp = MyTime::now();

  //std::cout<<m_sFONSData.dRoll<<" "<<m_sFONSData.dPitch<<" "<<m_sFONSData.dHeading<<std::endl;
  //std::cout<<m_sFONSData.dAccx<<" "<<m_sFONSData.dAccy<<" "<<m_sFONSData.dAccz<<std::endl;
  float roll = m_sFONSData.dRoll*M_PI/180;
  float pitch = m_sFONSData.dPitch*M_PI/180;
  float heading = m_sFONSData.dHeading*M_PI/180;
  tf::Quaternion orientation=tf::createQuaternionFromRPY(roll, pitch, heading);
  sensor_msgs::Imu imuout;
  tf::quaternionTFToMsg(orientation,imuout.orientation);
  imuout.header.stamp = ros::Time::now();
  //imuout.linear_acceleration.x = m_sFONSData.dAccx;
  //imuout.linear_acceleration.y = m_sFONSData.dAccy;
  //imuout.linear_acceleration.z = m_sFONSData.dAccz;
//  pubImudata.publish(imuout);
  //printf("%lf\n",m_sFONSData.dTimeStamp);
}

/*!
* \file myhdl_grabber.cpp
* \brief rslidar雷达数据获取及解析驱动
*
*该文件是单个rslidar雷达数据获取及解析驱动
*各函数及变量功能请参照velodyne/myhdl_grabber.cpp 文件
* \author
* \version v1.2.1
* \date 2018/11/23
*/
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
// #include <pcl/filters/filter.h>
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
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef HAVE_PCAP
#include <pcap.h>
#include <pcap/sll.h>
#endif // #ifdef HAVE_PCAP

const boost::asio::ip::address pcl::RslidarGrabber::HDL_DEFAULT_NETWORK_ADDRESS = boost::asio::ip::address::from_string ("192.168.3.255");
double *pcl::RslidarGrabber::cos_lookup_table_ = NULL;
double *pcl::RslidarGrabber::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;
///////////////////////////////////////////////////////////////////////////////new
pcl::RslidarGrabber::RslidarGrabber (const std::string& correctionsFile,
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
	, temper_index_(0)
{

    //	if(laser_layer == 16)
    //		LASER_LAYER = 16;
    //	else
    LASER_LAYER = laser_layer;
    initialize (correctionsFile);

}

///////////////////////////////////////////////////////////////////////////////new
pcl::RslidarGrabber::RslidarGrabber (const boost::asio::ip::address& ipAddress,
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
	, temper_index_(0)
{

    //  if(laser_layer == 16)
    //	  LASER_LAYER = 16;
    //  else
    LASER_LAYER = laser_layer;

    initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::RslidarGrabber::~RslidarGrabber () throw ()
{

    stop ();

    free(cos_lookup_table_);
    free(sin_lookup_table_);
    disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();

    std::cout<<"~Grabber"<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::initialize (const std::string& correctionsFile)
{
    	last_time_ = 0;
        scanCounter_ = 0;
        sweepCounter_ = 0;
    	externdatamode_ = false;
    	temp_count = 0;
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

//    loadCorrectionsFile (correctionsFile);
    loadConfigFile(correctionsFile);
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
        // std::cout<<index<<"\tlaserindex="<<iter->second<<"\tangle="<<iter->first<<"\ttanangle="<<tan(HDL_Grabber_toRadians(iter->first))<<std::endl;
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
    void
pcl::RslidarGrabber::loadConfigFile(std::string config_dir) {

            std::string anglePath, curvesPath, channelPath, curvesRatePath;
            std::string model;
            std::string anglefile = config_dir + "/angle.csv";
            std::string channelnumfile = config_dir + "/ChannelNum.csv";
            anglePath = config_dir  + "/angle.csv";
            curvesPath = config_dir  + "/curves.csv";
            channelPath = config_dir  + "/ChannelNum.csv";
            curvesRatePath = config_dir  + "/CurveRate.csv";


            /// 读参数文件 2017-02-27
            FILE *f_inten = fopen(curvesPath.c_str(), "r");
            int loopi = 0;
            int loopj = 0;

//            if (!f_inten) {
//                ROS_ERROR_STREAM(curvesPath << " does not exist");
//            } else {
//                while (!feof(f_inten)) {
//                    float a[32];
//                    loopi++;
//                    if (loopi > 7)
//                        break;
//                    if (LASER_LAYER == 16) {
//                        fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//                               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
//                               &a[13],
//                               &a[14], &a[15]);
//                    } else if (LASER_LAYER == 32) {
//                        fscanf(f_inten,
//                               "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//                               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
//                               &a[13],
//                               &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
//                               &a[25], &a[26], &a[27],
//                               &a[28], &a[29], &a[30], &a[31]);
//                    }
//                    for (loopj = 0; loopj < LASER_LAYER; loopj++) {
//                        aIntensityCal[loopi - 1][loopj] = a[loopj];
//                    }
//                }
//                fclose(f_inten);
//            }
            //=============================================================
            LOG(INFO)<<"test1";
            FILE *f_angle = fopen(anglePath.c_str(), "r");
            if (!f_angle) {
                ROS_ERROR_STREAM(anglePath << " does not exist");
            } else {
                float b[32], d[32];
                int loopk = 0;
                int loopn = 0;
                while (!feof(f_angle)) {
                    fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
                    if(LASER_LAYER<32)
                    	d[loopk] = 0;
                    loopk++;
                    if (loopk > (LASER_LAYER - 1)) break;
                }
                for (loopn = 0; loopn < LASER_LAYER; loopn++) {
                    laser_corrections_[loopn].verticalCorrection = b[loopn];
                    laser_corrections_[loopn].sinVertCorrection = std::sin (HDL_Grabber_toRadians(b[loopn]));
                    laser_corrections_[loopn].cosVertCorrection = std::cos (HDL_Grabber_toRadians(b[loopn]));

                    laser_corrections_[loopn].hori_angle = d[loopn] * 100;
                }
                fclose(f_angle);
            }
            LOG(INFO)<<"test2";
            //=============================================================
            FILE *f_channel = fopen(channelPath.c_str(), "r");
            if (!f_channel) {
                ROS_ERROR_STREAM(channelPath << " does not exist");
            } else {
                int loopl = 0;
                int loopm = 0;
                int c[51];
                int tempMode = 1;
                while (!feof(f_channel)) {
                    if(LASER_LAYER == 16)
                    {
                        fscanf(f_channel,
                               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                               &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                               &c[13], &c[14], &c[15],
                               &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26], &c[27],
                               &c[28], &c[29], &c[30],
                               &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40]);
                    }
                    else
                    {
                        fscanf(f_channel,
                               "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                               &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                               &c[13], &c[14], &c[15],
                               &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26], &c[27],
                               &c[28], &c[29], &c[30],
                               &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40],
                               &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48], &c[49], &c[50]);
                    }
                    if (c[1] < 100 || c[1] > 3000)
                    {
                        tempMode = 0;
                    }
                    for (loopl = 0; loopl < 51; loopl++) {
                    	laser_corrections_[loopm].g_ChannelNum[loopl] = c[tempMode * loopl];
                    }
                    loopm++;
                    if (loopm > (LASER_LAYER - 1)) {
                        break;
                    }
                }
                fclose(f_channel);
            }

//            if(LASER_LAYER == 32){
//                FILE *f_curvesRate = fopen(curvesRatePath.c_str(), "r");
//                if(!f_curvesRate)
//                {
//                    ROS_ERROR_STREAM(curvesRatePath << " does not exist");
//                }
//                else
//                {
//                    int loopk = 0;
//                    while(!feof(f_curvesRate))
//                    {
//                        fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
//                        loopk++;
//                        if(loopk > (LASER_LAYER-1)) break;
//                    }
//                    fclose(f_curvesRate);
//                }
//            }


        }


    void
pcl::RslidarGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{

    if(LASER_LAYER == 16)
    	loadrslidar16Corrections(correctionsFile);

}

    void
    pcl::RslidarGrabber::loadrslidar16Corrections (std::string config_dir)
    {

      std::string anglefile = config_dir + "/angle.csv";
      std::string channelnumfile = config_dir + "/ChannelNum.csv";
      std::cout<<anglefile<<std::endl;
      std::cout<<channelnumfile<<std::endl;
      std::fstream angle(anglefile.c_str(),std::ios::in);
      std::fstream channelnum(channelnumfile.c_str(),std::ios::in);

      double vlp16VerticalCorrections[16]={0};
      double distancecorrection[16]={0};
      for(int i=0;i<16;i++)
    	angle>>vlp16VerticalCorrections[i];

//      for(int i=0;i<16;i++)
//    	channelnum>>distancecorrection[i];

      for (int i = 0; i < 16; i++)
      {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection =distancecorrection[i];
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = vlp16VerticalCorrections[i];
        laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
      }
      for (int i = 16; i < MAX_NUM_LASERS; i++)
      {//根据协议可以知道每一个block发送两组数据
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = 0.0;
        laser_corrections_[i].sinVertCorrection = 0.0;
        laser_corrections_[i].cosVertCorrection = 1.0;
      }
    }


/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::processVelodynePackets ()
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
    void
pcl::RslidarGrabber::toPointClouds (HDLDataPacket *dataPacket)
{

//    if (sizeof (HDLLaserReturn) != 3)
//        return;

    time_t  time_;
    time(&time_);
    time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
//    if(scanCounter_%100==0&&id_==0)
//    LOG(INFO)<<"gpsTimestamp:"<<dataPacket->gpsTimestamp;
    scanCounter_++;
//    if(externdatamode_&&last_time_ >= dataPacket->gpsTimestamp&&!(last_time_>18000000&&dataPacket->gpsTimestamp<18000000))
//    	return;
    last_time_ = dataPacket->gpsTimestamp;

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
    {
    	if(LASER_LAYER == 64&&i/2%2==1)
    		continue;
        HDLFiringData firingData = dataPacket->firingData[i];
        unsigned int temprotation = firingData.rotation_1*256+firingData.rotation_2;//本时刻的水平角度;
        temprotation += 9000;
        if(temprotation>=36000)
        	temprotation -=36000;
//        if(scanCounter_%100==0)
//        	LOG(INFO)<<"id:"<<id_<<" rotation:"<<temprotation;
        if(LASER_LAYER == 16)
        {
            //������16����azimuth
        	unsigned short this_azimuth = firingData.rotation_1*256+firingData.rotation_2;//本时刻的水平角度;

            unsigned short interpolated_rotationalPosition = this_azimuth;//本时刻的水平角度;
            if(interpolated_rotationalPosition < last_azimuth_real_)
                interpolated_rotationalPosition += (this_azimuth + 36000 - last_azimuth_real_) / 2;
            else
                interpolated_rotationalPosition += (this_azimuth - last_azimuth_real_) / 2;

            //��һ��16��
            //�µ�һȦ������reset
            if ((current_sweep_xyzi_->size()/LASER_LAYER>1000&&(int)temprotation < (int)last_azimuth_-18000)
            		||synchornizationcontainor_->at(id_)==2)
            {
            	bool ismaster = (id_==0);

            	if(ismaster)
            	{
            		if((current_calibrated_xyzi_->size() - temp_count)/LASER_LAYER>1000)
						synchornizationcontainor_->at(id_)++;
					temp_count = current_sweep_xyzi_->size();
            	}
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
					temp_count = 0;
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
                computeXYZI (xyzi, this_azimuth, firingData.laserReturns[j], laser_corrections_[j]);
                xyzi.passibility = 1.0;
                last_azimuth_ = temprotation;
                last_azimuth_real_ = this_azimuth;
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

            if(LASER_LAYER == 32)
            	offset = 0;

            if (offset==0&&((current_sweep_xyzi_->size()/LASER_LAYER>1000&&(int)temprotation < (int)last_azimuth_-18000)
            		||synchornizationcontainor_->at(id_)==2))
            {
//            	bool ismaster = (synchornizationcontainor_->at(id_) < 2);
            	bool ismaster = (id_==0);
            	if(ismaster)
            	{
					if((current_sweep_xyzi_->size() - temp_count)/LASER_LAYER>1000)
						synchornizationcontainor_->at(id_)++;
					temp_count = current_sweep_xyzi_->size();
            	}
            	LOG(INFO)<<"id:"<<id_<<" temprotation:"<<temprotation<<" last_azimuth_"<<last_azimuth_;
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
					temp_count = 0;
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

        	unsigned short this_azimuth = firingData.rotation_1*256+firingData.rotation_2;//本时刻的水平角度;
//        	LOG(INFO)<<"id_:"<<id_<<" this_azimuth:"<<this_azimuth;
        	bool ABflag = firingData.laserReturns[0].distance_1 & 0x80;
            for (int j = 0; j < 32; j++)
            {
            	int dsr = j;
            	int index_j = dsr;  //rslidar32
                // SJY
                if (ABflag == 1 && dsr < 16) {
                	index_j = dsr + 16;
                } else if (ABflag == 1 && dsr >= 16) {
                	index_j = dsr - 16;
                } else {
                	index_j = dsr;
                }
                // SJY
                PointXYZI xyzi;

                computeXYZI (xyzi, this_azimuth, firingData.laserReturns[index_j], laser_corrections_[j + offset]);
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
    void
pcl::RslidarGrabber::computeXYZI (pcl::PointXYZI& point, int azimuth,
        HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
    double cosAzimuth, sinAzimuth;
    double disresolution;//距离的单位是cm
    // SJY
    //if(LASER_LAYER == 16)
        disresolution = 0.01;
    //else if(LASER_LAYER == 32)
    //    disresolution = 0.005;
    //if(LASER_LAYER == 16)
    // SJY
        laserReturn.distance_1 = laserReturn.distance_1 & 0x7f;
    double distanceM = (laserReturn.distance_1*256+laserReturn.distance_2 - correction.g_ChannelNum[temper_index_]) * disresolution;

    float intensityScale = (float)(correction.maxIntensity - correction.minIntensity);
    float focalSlope = correction.focalSlope;
    float focaloffset = 256*(1-correction.focalDistance/13100)*(1-correction.focalDistance/13100);
    point.intensity = 0;
    if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) {
        //point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
//    	std::cout<<distanceM<<std::endl;
        point.intensity = -1;// static_cast<float> (laserReturn.intensity);
        point.azimuth = static_cast<float> (azimuth + correction.hori_angle) / 100.0;
        point.range = -0.1;

        // HSH
        point.x = point.y = point.z = NAN;
        return;
    	// if((distanceM < 0.00001 || distanceM > max_distance_threshold_)&&correction.sinVertCorrection>-0.1)
    	// 	distanceM = max_distance_threshold_;
    	// else
    	// {
        //     point.x = point.y = point.z = 0.1;
        //     return;
    	// }
    }

    if (correction.hori_angle == 0)
    {
        cosAzimuth = cos_lookup_table_[azimuth];
        sinAzimuth = sin_lookup_table_[azimuth];
    }
    else
    {
    	azimuth += correction.hori_angle;
    	azimuth = (azimuth+36000)%36000;
        double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0));
        cosAzimuth = std::cos (azimuthInRadians);
        sinAzimuth = std::sin (azimuthInRadians);
    }

    double xyDistance = distanceM * correction.cosVertCorrection;
    //  float intensityVal = laserReturn.intensity;

    float intensityVal = laserReturn.intensity ;
    // HSH
    // if (intensityVal < correction.minIntensity)
    //     intensityVal = correction.minIntensity;
    // if (intensityVal > correction.maxIntensity)
    //     intensityVal = correction.maxIntensity;
    if (intensityVal < 0)
        intensityVal = 0;
    else if (intensityVal > 255)
        intensityVal = 255;
	point.x = static_cast<float> (xyDistance * sinAzimuth);
	point.y = static_cast<float> (xyDistance * cosAzimuth);
	point.z = static_cast<float> (distanceM * correction.sinVertCorrection);

    if(point.intensity > -0.5)
    {
    	point.intensity = static_cast<float> (intensityVal);
    	point.azimuth = static_cast<float> (azimuth) / 100.0;
    	point.range = static_cast<float>(distanceM);
    }
}

int pcl::RslidarGrabber::correctAzimuth(float azimuth_f, float hori_correction) {
        int azimuth;
        if (azimuth_f > 0.0 && azimuth_f < 3000.0) {
            azimuth_f = azimuth_f + hori_correction + 36000.0f;
        } else {
            azimuth_f = azimuth_f + hori_correction;
        }
        azimuth = (int)azimuth_f;
        azimuth %= 36000;

        return azimuth;
    }
void
pcl::RslidarGrabber::calibratePoint (pcl::PointXYZI& point)
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
		// filter vehicle body points
        if((fabs(point.x) < 1.5 && point.y < 4 && point.y > -2))
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
    void
pcl::RslidarGrabber::fireCurrentSweep ()
{
    pcl_conversions::toPCL(ros::Time::now(), current_sweep_xyzi_->header.stamp);//us
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*current_sweep_xyzi_, *current_sweep_xyzi_, indices);
    LOG(INFO)<<"signal operator start";
    if (sweep_xyzi_signal_->num_slots () > 0)
        sweep_xyzi_signal_->operator() (current_sweep_xyzi_,2*id_);
    LOG(INFO)<<"signal operator end";
    LOG(INFO)<<"temper_index_="<<temper_index_;
    // printf("points size: %d\n", current_sweep_xyzi_->points.size());
    if(calibrated_)
    {
        // pcl::removeNaNFromPointCloud(*current_calibrated_xyzi_, *current_calibrated_xyzi_, indices);
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
    bool
pcl::RslidarGrabber::enqueueHDLPacket (const unsigned char *data,
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

void pcl::RslidarGrabber::settemperindex(unsigned char bit1,unsigned char bit2)
{
	float Temp;
	float bitneg = bit2 & 128;//10000000
	float highbit = bit2 & 127;//01111111
	float lowbit = bit1 >> 3;
	if (bitneg == 128) {
		Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
	} else {
		Temp = (highbit * 32 + lowbit) * 0.0625f;
	}
	int temp = (int)std::floor(Temp + 0.5);
	if (temp < TEMPERATURE_MIN) {
		temp = TEMPERATURE_MIN;
	} else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
		temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
	}
	temper_index_ = temp - TEMPERATURE_MIN;
}

bool pcl::RslidarGrabber::externenqueueHDLPacket (const unsigned char *data,
		std::size_t bytesReceived)
{
	if(externdatamode_)
	{
		size_t length = bytesReceived;

		for(int i = 0; i + 8 < length; i++)
	   {

			if ((data[i] == 0x55) && (data[i + 1] == 0xAA) && (data[i + 2] == 0x05) && (data[i + 3] == 0x0A) && (data[i + 4] == 0x5A) && (data[i + 5] == 0xA5) && (data[i + 6] == 0x50) && (data[i + 7] == 0xA0))//对于数据的一个校验，根据通信手册，数据包前8位是固定的
			{
	            if (framecount % 20000 == 0)//update temperature information per 20000 packets
	            {
	            	settemperindex(data[38],data[39]);
	            }
				return enqueueHDLPacket (data+i+42, 1206);
			}
		}
	}

	return false;
}
/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::start ()
{
    terminate_read_packet_thread_ = false;


    if (isRunning ())
        return;


    queue_consumer_thread_ = new boost::thread (boost::bind (&RslidarGrabber::processVelodynePackets, this));

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
            PCL_ERROR ("[pcl::RslidarGrabber::start] Unable to bind to socket! %s\n", e.what());
            return;
        }
//        std::cout<<"socket2"<<std::endl;
        hdl_read_packet_thread_ = new boost::thread (boost::bind (&RslidarGrabber::readPacketsFromSocket, this));
    }
    else
    {
    	std::cout<<"can't support inter pcap parse, please set externdatamode"<<std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::stop ()
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

void pcl::RslidarGrabber::pause()
{
    read_packet_thread_paused_ = true;
}

void pcl::RslidarGrabber::resume()
{
    read_packet_thread_paused_ = false;
}

bool pcl::RslidarGrabber::getpausestate()
{
    return read_packet_thread_paused_;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::RslidarGrabber::isRunning () const
{
    return (!hdl_data_.isEmpty() || (hdl_read_packet_thread_ != NULL &&
                !hdl_read_packet_thread_->timed_join (boost::posix_time::milliseconds (10))));
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::RslidarGrabber::getName () const
{
    return (std::string ("Velodyne High Definition Laser (HDL) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::RslidarGrabber::getFramesPerSecond () const
{
    return (0.0f);
}

long pcl::RslidarGrabber::getframecount()
{
	return framecount;
}

void pcl::RslidarGrabber::setCalibration(const pcl::CalibrationValue& value)
{
	calibrationvalue_ = value;
	pcl::get_transform_matrix(calibrationvalue_ , calibration_transform_matrix_);
	calibrated_ = true;
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
        const unsigned short port)
{
    source_address_filter_ = ipAddress;
    source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::setLaserColorRGB (const pcl::RGB& color,
        unsigned int laserNumber)
{
    if (laserNumber >= MAX_NUM_LASERS)
        return;

    laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
    bool
pcl::RslidarGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
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
pcl::RslidarGrabber::setMaximumDistanceThreshold(float &maxThreshold) {
    max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RslidarGrabber::setMinimumDistanceThreshold(float &minThreshold) {
    min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::RslidarGrabber::getMaximumDistanceThreshold() {
    return(max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::RslidarGrabber::getMinimumDistanceThreshold() {
    return(min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
    void
pcl::RslidarGrabber::readPacketsFromSocket ()
{
  unsigned char data[1500];//实际上每次收到的数据都是1248
  unsigned char msg_Lidar[1248];//发送给雷达的数据
  udp::endpoint sender_endpoint;

  while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open())
  {
	size_t length = hdl_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);
	for(int i = 0; i + 8 < length; i++)
   {
		if ((data[i] == 0x55) && (data[i + 1] == 0xAA) && (data[i + 2] == 0x05) && (data[i + 3] == 0x0A) && (data[i + 4] == 0x5A) && (data[i + 5] == 0xA5) && (data[i + 6] == 0x50) && (data[i + 7] == 0xA0))//对于数据的一个校验，根据通信手册，数据包前8位是固定的
		{
            if (framecount % 20000 == 0)//update temperature information per 20000 packets
            {
            	settemperindex(data[38],data[39]);
            }
			memcpy(msg_Lidar,data+i+42,1206);//其实i是等于0的
			if (isAddressUnspecified (source_address_filter_) ||  (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
			{
			    enqueueHDLPacket (msg_Lidar, 1206);
			}
		}
	}
  }
}



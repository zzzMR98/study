#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <iostream>
#include <string>
#include <ctime>
#include <glog/logging.h>
#include "include/AnalysisGPS.h"
//#include <control_msgs/GetECUReport.h>
#include <sensor_driver_msgs/GpswithHeading.h>
#include <sensor_driver_msgs/OdometrywithGps.h>
#include <covgrid_slam_msgs/OdometrywithGps.h>
#include "covgrid_slam_msgs/GpsByLidarOdometry.h"
#include <sensor_driver_msgs/InsVelocity.h>
#include <sensor_driver_msgs/startconfig.h>
#include <sensor_driver_msgs/ECUData.h>
#include "sensor_driver_msgs/VehicleState.h"
//#include <control_msgs/GetECUReport.h>
#include <boost/bind.hpp>
#include <deque>
#include "ivcommon/transform/rigid_transform.h"
#include "include/util/xmlconf/xmlconf.h"

/**\brief
 * the golbal para used for transform between UTM coor and Ltitude&Longitude;
 *
 */
double a=6378137;//earth radius
double e2= 0.0818192*0.0818192;//e的平方
GridZone zone =UTM_ZONE_AUTO;// auto compute UTM zone acoording to input longitude.
Hemisphere hemi = HEMI_NORTH;

/**
 * \brief
 * store  ECU message
 */
struct struct_ECU_Mes
{
    double steer;
    double vel;
};


/**
 * \brief
 * store  GPS message
 *
 */
struct struct_GPS_Mes
{
    double	TimeStamp;
    double dLongitude;///< in degree
    double dLatitude;
    double altitude;
    double HDOP; ///<indecate GPS signal quality. only related to the geometry between satelites and our position.
    double  PHDT_heading; // /<Heading of the double antenna GPS reciever.
    int DGPSState;
    int Satnum;
    char GPSMissflag;///< 1:error 0:correct
};

/**
 * \brief
 * store GPS error compared with Odometry output
 */
struct struct_GPS_Error
{
    double north;///< unit: m
    double east;
};


/**
 * \brief
 * store  IMU message
 */
struct struct_IMU_Mes
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double TimeStamp;
    uint32 GPSweek;
    uint64   GPSms;
    double dLongitude; ///< in degree
    double dLatitude;
    double longitude_rad; ///<in rad
    double latitude_rad;
    double altitude;
    double yaw;///< in degree
    double pitch;
    double roll;
    Eigen::Vector3d linear_vel;//ENU
    Eigen::Vector3d angular_vel;
};

/**
 * \brief
 * store Lidarodometry msg
 */
struct struct_ODOMETER_Mes
{
    double TimeStamp;
    ::ivcommon::transform::Rigid3d pose_with_ins;
    ::ivcommon::transform::Rigid3d pose_without_ins;
    Eigen::Vector3d pose_diff_readmap;
    float matched_probability;///< if <0.4 ,the LidarOdometry output is invalid
};


/**
 * \brief
 * receive multisensor data in time order, processing and output the fused result
 */
class PostProcess
{
public:
    PostProcess(ros::NodeHandle& nodehandle, double time):m_nodehandle(nodehandle) , fusion(time)
    {
        init();
    }

    explicit PostProcess(double time):fusion(time)
    {
        init();
    }

    ~PostProcess(){};

    /**
     * prepare for incoming data
     */
    void init()
    {
        GPSreload_flag=false;///< if the GPS has been out of data for a while ,
                                ///< then the the next time GPS is valid ,filter will reload.
        //pause_flag=false;
        mileage_nogps=0;///< calculate the mileage during GPSoff time
//        readMap_error.north = 0;
//        readMap_error.east = 0;
//        readMap_error.mode =0;
        ros::param::get("~wheelbase_x",wheelbase_x);///<  ~ represent private space, where param can only be obtained by this node itself
        ros::param::get("~wheelbase_y",wheelbase_y);///< gps trans to INS coordinate
        ros::param::get("~disLimit",disLimit); ///< distance limit to decide whether GPS is valid
        ros::param::get("~timeLimit",timeLimit); ///< window size
        ros::param::get("~savedata_dir",savedata_dir_);
//        ros::param::get("~SatLimit_ma",SatLimit_ma);
//        ros::param::get("~SatLimit_fixed",SatLimit_fixed);

        subIMU=m_nodehandle.subscribe<sensor_driver_msgs::GpswithHeading>("gpsdata", 1, boost::bind(&PostProcess::IMU_1stMessageRecieved,this,_1));///< subscribe INS topic
        subInsVelocity = m_nodehandle.subscribe<sensor_driver_msgs::InsVelocity>("insvelocity", 1, boost::bind(&PostProcess::InsVelocityMessageRecieved,this,_1));
        //subIMU=m_nodehandle.subscribe("gpsdata", 1, &PostProcess::IMU_1stMessageRecieved,this);
        subGPS=m_nodehandle.subscribe<sensor_driver_msgs::GpswithHeading>("GPSmsg",1,boost::bind(&PostProcess::GPSMessageRecieved,this,_1));///< subscribe GPS topic
        subStates_ = m_nodehandle.subscribe<sensor_driver_msgs::VehicleState>("vehiclestate_GPS", 1, boost::bind(&PostProcess::vehiclestateMessageRecieved, this, _1));

        subOdo_with_ins.subscribe(m_nodehandle, "lidar_preciseodometry_to_earth", 1);
        //= m_nodehandle.subscribe<covgrid_slam_msgs::OdometrywithGps>
//                ("lidar_preciseodometry_to_earth",1,boost::bind(&PostProcess::OdometerMessageRecieved, this, _1));///< subscribe lidarodometry topic
        subOdo_without_ins.subscribe(m_nodehandle, "lidar_odometry_to_earth", 1);
        subOdo_readmap.subscribe(m_nodehandle, "gps_by_lidar_odometry", 1);
        //subECU=m_nodehandle.subscribe<control_msgs::GetECUReport>("ecudatareport",1,boost::bind(&PostProcess::ECUMessageRecieved,this,_1));
//        subFoggy=m_nodehandle.subscribe<std_msgs::String>("is_foggy",1,boost::bind(&PostProcess::FoggyMessageRecieved,this,_1));///< subscribe fog detection topic
        exact_sync_ = std::make_shared<ExactSync>(ExactPolicy(5),
                                        subOdo_readmap, subOdo_with_ins, subOdo_without_ins);
        exact_sync_->registerCallback(boost::bind(&PostProcess::Odometry_Related_Callback, this, _1, _2, _3));

        pub=m_nodehandle.advertise<sensor_driver_msgs::GpswithHeading>("sensor_fusion_output",1);///< advertise fused output
        pub_GPS_fixed=m_nodehandle.advertise<sensor_driver_msgs::GpswithHeading>("GPSmsg_fix",100);///< advertise GPS result transform to Rear axle
        pub_path_IMU=m_nodehandle.advertise<nav_msgs::Path>("IMU_rviz",100);
        pub_vehiclestate_ = m_nodehandle.advertise<sensor_driver_msgs::VehicleState>("vehiclestate", 10);


        ros::ServiceClient configclient = m_nodehandle.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");///< service client to receive config file
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
        sensor_driver_msgs::startconfig configsrv;

        while(!configclient.call(configsrv))
        {
            ros::Duration(0.01).sleep();
            std::cout<<"waiting for config file"<<std::endl;
        }

        /**
         * get the parameter of trans from INS coor to vehicle coor.
         */
        configstr_ = configsrv.response.configstr;
        log_on_ = false;
        xml_conf_.Parse(configstr_.c_str(),"GetGPSData");
        xml_conf_.GetSystemParam("log_path",log_path_);
        xml_conf_.GetSystemParam("log_on",log_on_);
        xml_conf_.GetSystemParam("system_start_time",log_start_time_);
        Eigen::Vector3d inspos;
        xml_conf_.GetModuleParam("pos_x",inspos[0]);
        xml_conf_.GetModuleParam("pos_y",inspos[1]);
        xml_conf_.GetModuleParam("pos_z",inspos[2]);
        Eigen::Vector3d insyawpitchroll;
        xml_conf_.GetModuleParam("yaw",insyawpitchroll[0]);
        xml_conf_.GetModuleParam("pitch",insyawpitchroll[1]);
        xml_conf_.GetModuleParam("roll",insyawpitchroll[2]);
        insyawpitchroll[0] = 0.0;

        Eigen::Quaterniond insquat = ::ivcommon::transform::RollPitchYaw(
                insyawpitchroll[2]*M_PI/180,insyawpitchroll[1]*M_PI/180,insyawpitchroll[0]*M_PI/180);

//    gpsquat.x() = tempquat.y();
//    gpsquat.y() = -tempquat.x();
        instrans_ = ::ivcommon::transform::Rigid3d(inspos,insquat);///< pose trans from INS to Rear axle
    }

    /**
     *
     * @param foggymsgs point to received foggy data
     *
     * this func is prepared for localization in foggy environment due to the mismatch of Lidarodometry
     */
//    void FoggyMessageRecieved(const std_msgs::StringConstPtr &foggymsgs)
//    {
//        foggy_mes=foggymsgs->data;
//    }

    /**
     *init filter in the 1st run
     * @param IMU_1stmsgs point to received INS data
     */
    void IMU_1stMessageRecieved(const sensor_driver_msgs::GpswithHeadingConstPtr& IMU_1stmsgs)//ConstPtr
    {
        imu_mes.TimeStamp=IMU_1stmsgs->gps.header.stamp.toSec();
        imu_mes.dLatitude=IMU_1stmsgs->gps.latitude;
        imu_mes.dLongitude=IMU_1stmsgs->gps.longitude;
        imu_mes.altitude=IMU_1stmsgs->gps.altitude;
//        imu_mes.yaw=IMU_1stmsgs->heading;//degree
//        imu_mes.pitch=IMU_1stmsgs->pitch;
//        imu_mes.roll=IMU_1stmsgs->roll;

        double N,E;
        geographic_to_grid(a, e2, imu_mes.dLatitude*M_PI/180, imu_mes.dLongitude*M_PI/180, &zone, &hemi, &N, &E);///< UTM trans
        //N-=pos_y*cos(-imu_mes.yaw*M_PI/180);
        //E-=pos_y*sin(-imu_mes.yaw*M_PI/180);
        Eigen::Vector3d temptranslation(E,N,IMU_1stmsgs->gps.altitude);
        Eigen::Quaterniond temprotation= ::ivcommon::transform::RollPitchYaw(IMU_1stmsgs->roll*M_PI/180,IMU_1stmsgs->pitch*M_PI/180,IMU_1stmsgs->heading*M_PI/180);
        ::ivcommon::transform::Rigid3d aftershaft= ::ivcommon::transform::Rigid3d(temptranslation,temprotation)*instrans_.inverse();
        temptranslation=aftershaft.translation();
        E=temptranslation(0);
        N=temptranslation(1);
        imu_mes.altitude=temptranslation(2);
        Eigen::Vector3d temprpy= ::ivcommon::transform::toRollPitchYaw(aftershaft.rotation());
        imu_mes.roll=temprpy(0)*180/M_PI;
        imu_mes.pitch=temprpy(1)*180/M_PI;
        imu_mes.yaw=temprpy(2)*180/M_PI;

        //std::cout<<"transformed RPY is : "<<temprpy*180/M_PI<<std::endl;

        grid_to_geographic(a,e2,zone,hemi,N,E,&imu_mes.latitude_rad,&imu_mes.longitude_rad);//zzh


        if(!fusion.m_Lflag)
        {
            fusion.LidarInit(imu_mes.latitude_rad,imu_mes.longitude_rad,imu_mes.altitude,imu_mes.roll*M_PI/180,imu_mes.pitch*M_PI/180,imu_mes.yaw*M_PI/180, imu_mes.TimeStamp);
            /// init filter.
        }


        //ROS_INFO_STREAM("IMU msgs update");

        ///< rviz
        geometry_msgs::PoseStamped path;
        path.header.stamp=IMU_1stmsgs->gps.header.stamp;
        path.header.frame_id="global_earth_frame";

        path.pose.position.x=E-500000;//east
        path.pose.position.y=N;
        path.pose.position.z=imu_mes.altitude;
        //path.pose.orientation.x=fusion.getLidarqx();
        //path.pose.orientation.y=fusion.getLidarqy();
        //path.pose.orientation.z=fusion.getLidarqz();
        //path.pose.orientation.w=fusion.getLidarqw();
        Path_IMU.header.frame_id="global_earth_frame";
        Path_IMU.poses.push_back(path);
        pub_path_IMU.publish(Path_IMU);
    }

    /**
     *  store INS velocity to make time alignment.
     * @param InsVelocity_msgs
     */
    void InsVelocityMessageRecieved(const sensor_driver_msgs::InsVelocityConstPtr& InsVelocity_msgs)
    {
        imu_mes.linear_vel = std::move(Eigen::Vector3d (InsVelocity_msgs->linear_velocity.x, InsVelocity_msgs->linear_velocity.y, InsVelocity_msgs->linear_velocity.z));// now body frame
        imu_mes.angular_vel = std::move(Eigen::Vector3d (InsVelocity_msgs->angular_velocity.x, InsVelocity_msgs->angular_velocity.y, InsVelocity_msgs->angular_velocity.z));
        double timestamp = InsVelocity_msgs->header.stamp.toSec();
        fusion.AddInsVelocityData(std::move(Time_Align::InsVelocityData(timestamp, imu_mes.linear_vel, imu_mes.angular_vel)));
        //ROS_INFO_STREAM("InsVelocity msgs update");
    }

    /**
     * recieve  odometry data when their Header.stamp are exactly the same.
     * make sure whether the localization info from map matching is valid, or do lidar odometry measurement update
     * @param Odomsg_ReadMap data from map matching
     * @param Odomsg_with_ins data from odometry with INS
     * @param Odomsg_without_ins data from odometry without ins
     */
    void Odometry_Related_Callback(const covgrid_slam_msgs::GpsByLidarOdometryConstPtr& Odomsg_ReadMap,
            const covgrid_slam_msgs::OdometrywithGpsConstPtr &Odomsg_with_ins, const sensor_driver_msgs::OdometrywithGpsConstPtr &Odomsg_without_ins)
    {
        static int count = 0;
        count ++;
        //LOG(ERROR) << "odometry callback count " << count;
        if(fusion.m_Lflag)
        {
            odo_msg_.TimeStamp = Odomsg_with_ins->odometry.header.stamp.toSec();
            odo_msg_.matched_probability = Odomsg_with_ins->matched_probability;
            Eigen::Vector3d temptrans(Odomsg_with_ins->odometry.pose.pose.position.x,
                                      Odomsg_with_ins->odometry.pose.pose.position.y,
                                      Odomsg_with_ins->odometry.pose.pose.position.z);
            Eigen::Quaterniond tempquat(Odomsg_with_ins->odometry.pose.pose.orientation.w,
                                        Odomsg_with_ins->odometry.pose.pose.orientation.x,
                                        Odomsg_with_ins->odometry.pose.pose.orientation.y,
                                        Odomsg_with_ins->odometry.pose.pose.orientation.z);
            odo_msg_.pose_with_ins = ::ivcommon::transform::Rigid3d(temptrans, tempquat);
            static auto odo_msg_last = odo_msg_;

            //first, check whether Odomsg_ReadMap is valid
            ::ivcommon::transform::Rigid3d pose_diff_without_ins, pose_diff_with_ins;
            if( Odomsg_ReadMap->mode == 2 )
            {
                temptrans = Eigen::Vector3d(Odomsg_ReadMap->odometry.pose.pose.position.x,
                                            Odomsg_ReadMap->odometry.pose.pose.position.y,
                                            Odomsg_ReadMap->odometry.pose.pose.position.z);
                tempquat = Eigen::Quaterniond(Odomsg_ReadMap->odometry.pose.pose.orientation.w,
                                              Odomsg_ReadMap->odometry.pose.pose.orientation.x,
                                              Odomsg_ReadMap->odometry.pose.pose.orientation.y,
                                              Odomsg_ReadMap->odometry.pose.pose.orientation.z);
                odo_msg_.pose_without_ins = ::ivcommon::transform::Rigid3d(temptrans, tempquat); // readmap_pose
                odo_msg_.pose_diff_readmap = (odo_msg_.pose_with_ins.translation() - odo_msg_.pose_without_ins.translation());
                deque_readMap_info_.push_back(odo_msg_);
                auto odo_msg_origin = deque_readMap_info_.front();

                while(!deque_readMap_info_.empty())
                {
                    if((odo_msg_.pose_with_ins.inverse() * odo_msg_origin.pose_with_ins).translation().norm() > 15
                        || deque_readMap_info_.size() > 600 )
                    {
                        deque_readMap_info_.pop_front();
                        odo_msg_origin = deque_readMap_info_.front();
                    }

                    else
                        break;
                }

                double diff_x_min = odo_msg_.pose_diff_readmap.x();
                double diff_x_max = odo_msg_.pose_diff_readmap.x();
                double diff_y_min = odo_msg_.pose_diff_readmap.y();
                double diff_y_max = odo_msg_.pose_diff_readmap.y();
                /**
                 * compute the dis error for GPS and store them in a sliding window.
                 */
                for(const auto& info:deque_readMap_info_)
                {
                    if(info.pose_diff_readmap.x() > diff_x_max)
                        diff_x_max = info.pose_diff_readmap.x();
                    if(info.pose_diff_readmap.x() < diff_x_min)
                        diff_x_min = info.pose_diff_readmap.x();

                    if(info.pose_diff_readmap.y() > diff_y_max)
                        diff_y_max = info.pose_diff_readmap.y();
                    if(info.pose_diff_readmap.y() < diff_y_min)
                        diff_y_min = info.pose_diff_readmap.y();
                }
                //LOG(ERROR) << "diff_x_max - diff_x_min " << diff_x_max - diff_x_min << " diff_y_max - diff_y_min " << diff_y_max - diff_y_min;
                if( (diff_x_max - diff_x_min) < 2 && (diff_y_max - diff_y_min) < 2 )///< if GPS error is out of threshold, filter will not do measure update any more until error reduce
                {
                    //LOG(WARNING) << " pose_with_ins mileage " << (odo_msg_.pose_with_ins.inverse() * odo_msg_origin.pose_with_ins).translation().norm();
                    if((odo_msg_.pose_with_ins.inverse() * odo_msg_origin.pose_with_ins).translation().norm() > 10)
                    {
                        Eigen::Quaterniond quat_readmap(Odomsg_ReadMap->odometry.pose.pose.orientation.w,
                                                        Odomsg_ReadMap->odometry.pose.pose.orientation.x,
                                                        Odomsg_ReadMap->odometry.pose.pose.orientation.y,
                                                        Odomsg_ReadMap->odometry.pose.pose.orientation.z);
                        auto euler_angle = ::ivcommon::transform::toRollPitchYaw(quat_readmap);
                        fusion.LidarInit(Odomsg_ReadMap->gps.latitude*M_PI/180, Odomsg_ReadMap->gps.longitude*M_PI/180,
                                                      Odomsg_ReadMap->gps.altitude,euler_angle[0], euler_angle[1], euler_angle[2], Odomsg_ReadMap->gps.header.stamp.toSec());
                        flag_readMap_valid = 1; //correct match
                    } else
                    {
                        flag_readMap_valid = 3; //unsure
                    }

                }
                else
                    flag_readMap_valid = 2; // incorrect match

            }
            else
            {
                LOG(WARNING)<< "odometry match offline map faild";
                flag_readMap_valid = 0;
                deque_readMap_info_.clear();
            }

            /// 统计gps丢失里程
            double Odo_duration = odo_msg_.TimeStamp - odo_msg_last.TimeStamp;
            Vector3d p_delta = odo_msg_.pose_with_ins.translation() - odo_msg_last.pose_with_ins.translation();
            odo_msg_last = odo_msg_;
/*
//
*/
            if(gps_mes.GPSMissflag == '1' && flag_readMap_valid == 0)
            {
                mileage_nogps += p_delta.norm();
                LOG(INFO)<<"missing GPS mileage is: "<<mileage_nogps<<endl;
            }
            else
            {
                if(mileage_nogps>3000)
                {
                    GPSreload_flag=true;
                }
                mileage_nogps=0;
            }

            /// 统计odo失效
//            if(odo_mes.matched_probability<0.25)
//            {
//                odobadmatch_count++;
//            }
//            else
//            {
//                odobadmatch_count=0;
//            }

            //fuse
            if( flag_readMap_valid != 1 )
            {
                bool timeUpdate_flag = (flag_readMap_valid != 3) && (gps_mes.GPSMissflag == '0');
                fusion.Lidar_time_update(Odo_duration,p_delta(0),p_delta(1),p_delta(2),Quaterniond::Identity(),timeUpdate_flag, odo_msg_.TimeStamp);
                LOG(INFO) << "Lidar_time_update , flag_readMap_valid is " << flag_readMap_valid;
            }

            fusion.Lidar_measure_update_resetangle(imu_mes.roll*M_PI/180,imu_mes.pitch*M_PI/180,imu_mes.yaw*M_PI/180);
            //ROS_INFO_STREAM("Odo msgs updated");

            /// 拿出计算后的经纬度
            sensor_driver_msgs::GpswithHeading after_fusion;

            after_fusion.header.stamp = Odomsg_with_ins->odometry.header.stamp;
            after_fusion.header.frame_id = "global_earth_frame";
            after_fusion.gps.header.frame_id = "global_earth_frame";
            //after_fusion.gps.header.stamp=Odometermsgs.odometry.header.stamp;
            after_fusion.gps.header.stamp = Odomsg_with_ins->odometry.header.stamp;
            after_fusion.gps.latitude = fusion.getLidarLa()*180/M_PI;
            after_fusion.gps.longitude = fusion.getLidarLo()*180/M_PI;
            after_fusion.gps.altitude = fusion.getaltitude();

            after_fusion.roll = fusion.getroll()*180/M_PI;
            after_fusion.pitch = fusion.getpitch()*180/M_PI;
            after_fusion.heading = fusion.getyaw()*180/M_PI;

            after_fusion.gps.status.status = gps_mes.GPSMissflag;
            after_fusion.mode = flag_readMap_valid;
            cout<<"after_fusion.gps.status.status is: "<<after_fusion.gps.status.status<<endl;
            gps_mes.GPSMissflag = '1';
            pub.publish(after_fusion);
            last_after_fusion_ = after_fusion;
            LogData();

            //ROS_INFO_STREAM("Lidar_filter_output advertised");
            sensor_driver_msgs::VehicleState vehicleState;
            vehicleState.header.stamp = Odomsg_with_ins->odometry.header.stamp;
            vehicleState.header.frame_id = "global_earth_frame";
            vehicleState.gps_week = imu_mes.GPSweek;
            vehicleState.gps_ms = imu_mes.GPSms;
            vehicleState.gps = after_fusion.gps;
            vehicleState.yaw = after_fusion.heading;
            vehicleState.roll = after_fusion.roll;
            vehicleState.pitch = after_fusion.pitch;

            Eigen::Quaterniond temp_rot = ::ivcommon::transform::RollPitchYaw(imu_mes.roll*M_PI/180, imu_mes.pitch*M_PI/180, imu_mes.yaw*M_PI/180);
            auto temp_vel = temp_rot * imu_mes.linear_vel;//ENU frame
            vehicleState.linear_velocity.x = temp_vel.x();
            vehicleState.linear_velocity.z = temp_vel.y();
            vehicleState.linear_velocity.y = temp_vel.z();
            vehicleState.angular_velocity.x = imu_mes.angular_vel.x();
            vehicleState.angular_velocity.y = imu_mes.angular_vel.y();
            vehicleState.angular_velocity.z = imu_mes.angular_vel.z();

            pub_vehiclestate_.publish(vehicleState);

            flag_Lidar_init = true;
        }




        LOG(INFO)<<" Odometry Msg Recieved ";
    }


/**
 * compute the diff between GPS and Lidarodometry localization result in North and East
 * filter measurement update
 * @param GPSmsgs point to received GPS data
 */
    void GPSMessageRecieved(const sensor_driver_msgs::GpswithHeadingConstPtr &GPSmsgs)
    {
        gps_mes.TimeStamp=GPSmsgs->gps.header.stamp.toSec();
        gps_mes.dLatitude=GPSmsgs->gps.latitude;
        gps_mes.dLongitude=GPSmsgs->gps.longitude;
        gps_mes.altitude=GPSmsgs->gps.altitude;
        gps_mes.PHDT_heading=GPSmsgs->heading;
        gps_mes.DGPSState=(int)GPSmsgs->roll;
        gps_mes.Satnum=(int)GPSmsgs->pitch;

        if(flag_Lidar_init)
        {
            //std::cout<<"*******satelites number is : "<<gps_mes.Satnum<<std::endl;
            //std::cout<<"*******DGPSState is : "<<gps_mes.DGPSState<<std::endl;
            double N,E;
            double lat_rad;
            double lon_rad;
            geographic_to_grid(a, e2, GPSmsgs->gps.latitude*M_PI/180, GPSmsgs->gps.longitude*M_PI/180, &zone, &hemi, &N, &E);
            double E_trans=E-500000; /// Odometry's UTM coor has a defferent defination with normal used one. we trans it back to normal

            struct_GPS_Error gps_error;
            gps_error.north=N-odo_msg_.pose_with_ins.translation()(1);
            gps_error.east=E_trans-odo_msg_.pose_with_ins.translation()(0);
            deque_gps_error_.push_back(gps_error);
            if(deque_gps_error_.size() > 10*timeLimit)///< GPS frequency: 10Hz
                deque_gps_error_.pop_front();

            double north_diff_max=gps_error.north;
            double north_diff_min=gps_error.north;
            double east_diff_max=gps_error.east;
            double east_diff_min=gps_error.east;
            /**
             * compute the dis error for GPS and store them in a sliding window.
             */
            for(struct_GPS_Error error:deque_gps_error_)
            {
                if(error.north>north_diff_max)
                    north_diff_max=error.north;
                if(error.north<north_diff_min)
                    north_diff_min=error.north;

                if(error.east>east_diff_max)
                    east_diff_max=error.east;
                if(error.east<east_diff_min)
                    east_diff_min=error.east;
            }

            if(north_diff_max-north_diff_min>disLimit||east_diff_max-east_diff_min>disLimit)///< if GPS error is out of threshold, filter will not do measure update any more until error reduce
                flag_GPS_valid=false;
            else
                flag_GPS_valid=true;


//        bool gps_valid=false;//局部flag
//        if(GPSmsgs->gps.latitude>0.00001&&GPSmsgs->gps.longitude>0.00001)//0经纬跑不了，防止GPS没信号
//        {
//            switch(gps_mes.DGPSState)
//            {
//                case 0:
//                {
//                    break;
//                }
//                case 1://单点定位，无差分
//                {
//                    if(gps_mes.Satnum>SatLimit_single)
//                        gps_valid=true;
//
//                    break;
//                }
//                case 2://码差分
//                {
//                    if(gps_mes.Satnum>SatLimit_ma)
//                        gps_valid=true;
//
//                    break;
//                }
//                case 3://无效PPS
//                {
//                    break;
//                }
//                case 4://RTK固定解
//                {
//                    if(gps_mes.Satnum>SatLimit_fixed)
//                        gps_valid=true;
//
//                    break;
//                }
//                default:break;
//            }
//
//        }


            if(flag_GPS_valid&&GPSmsgs->gps.latitude>0.00001&&GPSmsgs->gps.longitude>0.00001)
            {
                gps_mes.GPSMissflag='0';

                if( flag_readMap_valid != 1 && flag_readMap_valid != 3)
                {
                    Eigen::Vector3d gpspos(wheelbase_x,wheelbase_y,0);///< transform from GPS to Rear axle
                    Eigen::Vector3d temptranslation(E,N,0);
                    Eigen::Quaterniond temprotatation= ::ivcommon::transform::RollPitchYaw(0,0,imu_mes.yaw*M_PI/180);
                    Eigen::Vector3d aftershaft= ::ivcommon::transform::Rigid3d(temptranslation,temprotatation)*-gpspos;//gpscoor2UTM * aftershaft in gpscoor
                    E=aftershaft(0);
                    N=aftershaft(1);

                    grid_to_geographic(a,e2,zone,hemi,N,E,&lat_rad,&lon_rad);

                    fusion.Lidar_measure_update_GPS(lat_rad,lon_rad, gps_mes.TimeStamp);

                    if(GPSreload_flag)
                    {
                        fusion.LidarInit(lat_rad,lon_rad,imu_mes.altitude,imu_mes.roll*M_PI/180,imu_mes.pitch*M_PI/180,imu_mes.yaw*M_PI/180, imu_mes.TimeStamp);
                        GPSreload_flag=false;
                        //ROS_INFO_STREAM("GPS msgs recieved, Lidar init");
                    }
                }

            }
            else
            {
                //ROS_INFO_STREAM("GPS msgs invalid");
                gps_mes.GPSMissflag='1';
            }


            sensor_driver_msgs::GpswithHeading GPS_fix;
            GPS_fix.gps.header.stamp=GPSmsgs->gps.header.stamp;
            GPS_fix.gps.latitude=lat_rad*180/M_PI;
            GPS_fix.gps.longitude=lon_rad*180/M_PI;
            pub_GPS_fixed.publish(GPS_fix);
//            //可视化rviz
//            geometry_msgs::PoseStamped path;
//            path.header.stamp=GPSmsgs->gps.header.stamp;
//            path.header.frame_id="global_earth_frame";
//
//            path.pose.position.x=E-500000;//east
//            path.pose.position.y=N;
//            path.pose.position.z=imu_mes.altitude;
//            //path.pose.orientation.x=fusion.getLidarqx();
//            //path.pose.orientation.y=fusion.getLidarqy();
//            //path.pose.orientation.z=fusion.getLidarqz();
//            //path.pose.orientation.w=fusion.getLidarqw();
//            Path_GPS.header.frame_id="global_earth_frame";
//            Path_GPS.poses.push_back(path);
//            pub_path_GPS.publish(Path_GPS);
        }

    }

    void vehiclestateMessageRecieved(const sensor_driver_msgs::VehicleStateConstPtr& vehiclestatemsgs)
    {
        imu_mes.GPSweek = vehiclestatemsgs->gps_week;
        imu_mes.GPSms = vehiclestatemsgs->gps_ms;
    }

    bool makeStampedDir(std::string& folder)
    {
        char readable_start_time[100];
        time_t timel;
        time(&timel);
        tm* pTmp=localtime(&timel);

        memset(readable_start_time,0,21);
        sprintf(readable_start_time, "%04d-%02d-%02d_%02d-%02d-%02d",
                pTmp->tm_year + 1900,
                pTmp->tm_mon + 1,
                pTmp->tm_mday,
                pTmp->tm_hour,
                pTmp->tm_min,
                pTmp->tm_sec);

        folder += '/';
        folder += readable_start_time;
        int flag=mkdir(folder.c_str(), 0777);
        if (flag == 0)
        {
            LOG(INFO)<<"make dir successfully"<<std::endl;
            return true;
        } else {
            LOG(ERROR)<<"make dir errorly"<<std::endl;
            return false;
        }
    }

    void LogData()
    {
        static bool mkdir_success = makeStampedDir(savedata_dir_);
        CHECK(mkdir_success) << " sensor_fusion log data failed ";


        static std::ofstream log_gps_stream(savedata_dir_ + "/" + "GPS_point.txt", std::ios::out);
        log_gps_stream << std::fixed<<std::setprecision(7)
                       << last_after_fusion_.gps.latitude << " "
                       << last_after_fusion_.gps.longitude << " "
                       << last_after_fusion_.gps.altitude << " "
                       << last_after_fusion_.heading << " "
                       << last_after_fusion_.mode << " "
                       << imu_mes.dLatitude << " "
                       << imu_mes.dLongitude << " "
                       << imu_mes.altitude << " "
                       << imu_mes.yaw << std::endl;

    }

    ::ros::NodeHandle m_nodehandle;
    ros::Subscriber subIMU;//=nh.subscribe("gpsdata",1,&IMU_1stMessageRecieved);
    ros::Subscriber subInsVelocity;
    ros::Subscriber subGPS;//=nh.subscribe("GPSmsg",1,&GPSMessageRecieved);
    ros::Subscriber subStates_;
    message_filters::Subscriber<covgrid_slam_msgs::OdometrywithGps> subOdo_with_ins;//=nh.subscribe("lidar_preciseodometry_to_earth",1,&OdometerMessageRecieved);
    message_filters::Subscriber<sensor_driver_msgs::OdometrywithGps> subOdo_without_ins;
    message_filters::Subscriber<covgrid_slam_msgs::GpsByLidarOdometry> subOdo_readmap;
    ros::Subscriber subECU;
    typedef message_filters::sync_policies::ExactTime<covgrid_slam_msgs::GpsByLidarOdometry, covgrid_slam_msgs::OdometrywithGps, sensor_driver_msgs::OdometrywithGps> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    std::shared_ptr<ExactSync> exact_sync_;
//    ros::Subscriber subFoggy;
    ros::Publisher pub;
    ros::Publisher pub_GPS_fixed;
    ros::Publisher pub_path_IMU;
    ros::Publisher pub_vehiclestate_;

    struct_GPS_Mes gps_mes;
    struct_IMU_Mes imu_mes;
    struct_ODOMETER_Mes odo_msg_;
    struct_ECU_Mes ecu_mes;
    string foggy_mes;
    COdometry fusion;
    bool flag_GPS_valid = true;
    int flag_readMap_valid = 0;
    bool flag_Lidar_init = false;
    nav_msgs::Path Path_GPS;
    nav_msgs::Path Path_IMU;
    sensor_driver_msgs::GpswithHeading last_after_fusion_;
    //bool pause_flag;
    bool GPSreload_flag;
    double mileage_nogps;

    double wheelbase_x=0;
    double wheelbase_y=0;
    double disLimit;
    double timeLimit;
    std::string savedata_dir_;
    ::ivcommon::transform::Rigid3d instrans_;
    XmlConf xml_conf_;
    std::string configstr_;
    bool log_on_;
    std::string log_path_;
    double log_start_time_;

    std::deque<struct_GPS_Error> deque_gps_error_;
    std::deque<struct_ODOMETER_Mes> deque_readMap_info_;

};





int main(int argc,char **argv)
{
    ros::init(argc,argv,"lidar_filter");
    ros::NodeHandle nh;

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

    PostProcess postprocess(nh, ros::Time::now().toSec());

    /*
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
     */
    ros::spin();
    return 0;
}





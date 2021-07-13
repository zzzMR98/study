#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <iostream>
#include "include/AnalysisGPS.h"
//#include <control_msgs/GetECUReport.h>
#include <sensor_fusion/GpswithHeading.h>
#include <sensor_fusion/OdometrywithGps.h>
#include <sensor_fusion/gpsmsg.h>
#include <sensor_fusion/InsVelocity.h>
#include <sensor_fusion/ECUData.h>

#include <boost/bind.hpp>

#define PPI 3.14159265358979
#define pos_x 0
#define pos_y 1.2
#define pos_z 1.9

double a=6378137;
double e2= 0.0818192*0.0818192;//e的平方
GridZone zone =UTM_ZONE_AUTO;
Hemisphere hemi = HEMI_NORTH;

struct struct_ECU_Mes
{
    double steer;
    double vel;
};

struct struct_GPS_Mes
{
    double	TimeStamp;
    double dLongitude;
    double dLatitude;
    double altitude;
    double HDOP;
    double  PHDT_heading;
    char DGPSState;//1:error 0:correct
};

struct struct_IMU_Mes
{
    double TimeStamp;
    double dLongitude;
    double dLatitude;
    double altitude;
    double yaw;
    double pitch;
    double roll;
    double vbx;
    double vby;
    double vbz;
    double omegabx;//朝向right的轴旋转
    double omegaby;//forward
    double omegabz;//up
    double acc_x;//right
    double acc_y;//forward
    double acc_z;//up
};

struct struct_ODOMETER_Mes
{
    double TimeStamp;
    double quatx;
    double quaty;
    double quatz;
    double quatw;
    double positionx;
    double positiony;
    double positionz;
};

class PostProcess
{
public:
    PostProcess(ros::NodeHandle& nodehandle):m_nodehandle(nodehandle)
    {
        init();
    }
    ~PostProcess()
    {
    }

    void init()
    {
        flag_IMU_valid=false;
        flag_Lidar_valid=false;
        GPS_lost_count=0;
        ros::param::get("~wheelbase",wheelbase);//~表示私有空间的参数

        subIMU=m_nodehandle.subscribe<sensor_fusion::GpswithHeading>("gpsdata", 1, boost::bind(&PostProcess::IMU_1stMessageRecieved,this,_1));
        //subGPS=m_nodehandle.subscribe<sensor_fusion::GpswithHeading>("GPSmsg",1,boost::bind(&PostProcess::GPSMessageRecieved,this,_1));
        subOdo=m_nodehandle.subscribe<sensor_fusion::OdometrywithGps>
                ("lidar_preciseodometry_to_earth",1,boost::bind(&PostProcess::OdometerMessageRecieved,this,_1));
//        subECU=m_nodehandle.subscribe("ecudata",1,&ECUMessageRecieved);

        //processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
        //slopethread_ = new boost::thread(boost::bind(&PostProcess::pointCloudDisplay,this));//mdj 新开一个线程
        //file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);
//        pub=m_nodehandle.advertise<sensor_fusion::GpswithHeading>("lidar_filter_output",1);
        pub=m_nodehandle.advertise<sensor_fusion::GpswithHeading>("sensor_fusion_output",1);
        pub_path_GPS=m_nodehandle.advertise<nav_msgs::Path>("GPS_rviz",100);
        pub_path_IMU=m_nodehandle.advertise<nav_msgs::Path>("IMU_rviz",100);
    }

    void IMU_1stMessageRecieved(const sensor_fusion::GpswithHeadingConstPtr &IMU_1stmsgs)
    {
        imu_mes.TimeStamp=IMU_1stmsgs->gps.header.stamp.toSec();
        imu_mes.dLatitude=IMU_1stmsgs->gps.latitude;
        imu_mes.dLongitude=IMU_1stmsgs->gps.longitude;
        imu_mes.altitude=IMU_1stmsgs->gps.altitude-pos_z;
        imu_mes.yaw=IMU_1stmsgs->heading;//degree
        imu_mes.pitch=IMU_1stmsgs->pitch;
        imu_mes.roll=IMU_1stmsgs->roll;
        flag_IMU_valid=true;

        double N,E,lat_rad,lon_rad;
        geographic_to_grid(a, e2, imu_mes.dLatitude*PPI/180, imu_mes.dLongitude*PPI/180, &zone, &hemi, &N, &E);
        N-=pos_y*cos(-imu_mes.yaw*PPI/180);
        E-=pos_y*sin(-imu_mes.yaw*PPI/180);
        grid_to_geographic(a,e2,zone,hemi,N,E,&lat_rad,&lon_rad);//zzh

        if(!fusion.m_Lflag)
            fusion.LidarInit(lat_rad,lon_rad,imu_mes.altitude,imu_mes.roll*PPI/180,imu_mes.pitch*PPI/180,imu_mes.yaw*PPI/180);
//        else if(flag_Lidar_valid&&gps_mes.DGPSState=='0')
//        {
//            fusion.Lidar_measure_update_GPS(lat_rad,lon_rad);
//            flag_Lidar_valid=false;
//            ROS_INFO_STREAM("IMU msgs update");
//        }


        //可视化rviz
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

    void ECUMessageRecieved(const sensor_fusion::ECUDataConstPtr &ecumsg)
    {
        ecu_mes.vel=ecumsg->fForwardVel;//ms-1
        ecu_mes.steer=ecumsg->fFLRWheelAverAngle;//前轮偏角，需注意没有固定零位，每次上电时的转角为零位，左转为正，右为负 degree
        cout<<"ecu recieved v is :"<<ecu_mes.vel<<endl;
        if(flag_Lidar_valid&&fusion.m_Lflag)
            fusion.Lidar_measure_update_ECU(ecu_mes.vel);
        ROS_INFO_STREAM("ECU msgs update");
    }

    void GPSMessageRecieved(const sensor_fusion::GpswithHeadingConstPtr &GPSmsgs)
    {
        gps_mes.TimeStamp=GPSmsgs->gps.header.stamp.toSec();
        gps_mes.altitude=GPSmsgs->gps.altitude;
        gps_mes.PHDT_heading=GPSmsgs->heading;
        double N,E;
        double lat_rad;
        double lon_rad;
        geographic_to_grid(a, e2, GPSmsgs->gps.latitude*PPI/180, GPSmsgs->gps.longitude*PPI/180, &zone, &hemi, &N, &E);

        if(GPSmsgs->gps.latitude>0.001&&GPSmsgs->gps.longitude>0.001)//0经纬跑不了，防止GPS没信号
        {
            gps_mes.DGPSState='0';
            GPS_lost_count=0;
            N=N+wheelbase*cos(-imu_mes.yaw*PPI/180)-pos_y*cos(-imu_mes.yaw*PPI/180);
            E=E+wheelbase*sin(-imu_mes.yaw*PPI/180)-pos_y*sin(-imu_mes.yaw*PPI/180);
            grid_to_geographic(a,e2,zone,hemi,N,E,&lat_rad,&lon_rad);//zzh

            if(GPS_lost_count>600)
                fusion.m_Lflag=false;
            ROS_INFO_STREAM("GPS msgs update");
        }
        else
        {
            ROS_INFO_STREAM("GPS msgs error");
            gps_mes.DGPSState='1';
            GPS_lost_count++;
        }

        //可视化rviz
        geometry_msgs::PoseStamped path;
        path.header.stamp=GPSmsgs->gps.header.stamp;
        path.header.frame_id="global_earth_frame";

        path.pose.position.x=E-500000;//east
        path.pose.position.y=N;
        path.pose.position.z=imu_mes.altitude;
        //path.pose.orientation.x=fusion.getLidarqx();
        //path.pose.orientation.y=fusion.getLidarqy();
        //path.pose.orientation.z=fusion.getLidarqz();
        //path.pose.orientation.w=fusion.getLidarqw();
        Path_GPS.header.frame_id="global_earth_frame";
        Path_GPS.poses.push_back(path);
        pub_path_GPS.publish(Path_GPS);
    }

    void OdometerMessageRecieved(const sensor_fusion::OdometrywithGpsConstPtr &Odometermsgs)
    {
        if(fusion.m_Lflag)
        {
            odo_mes.TimeStamp=Odometermsgs->odometry.header.stamp.toSec();
            odo_mes.quatx=Odometermsgs->odometry.pose.pose.orientation.x;
            odo_mes.quaty=Odometermsgs->odometry.pose.pose.orientation.y;
            odo_mes.quatz=Odometermsgs->odometry.pose.pose.orientation.z;
            odo_mes.quatw=Odometermsgs->odometry.pose.pose.orientation.w;
            odo_mes.positionx=Odometermsgs->odometry.pose.pose.position.x;
            odo_mes.positiony=Odometermsgs->odometry.pose.pose.position.y;
            odo_mes.positionz=Odometermsgs->odometry.pose.pose.position.z;

            static double Odo_last_px=odo_mes.positionx;
            static double Odo_last_py=odo_mes.positiony;
            static double Odo_last_pz=odo_mes.positionz;
            static double Odo_last_qx=odo_mes.quatx;
            static double Odo_last_qy=odo_mes.quaty;
            static double Odo_last_qz=odo_mes.quatz;
            static double Odo_last_qw=odo_mes.quatw;
            static double Odo_last_TimeStamp=odo_mes.TimeStamp;

            double Odo_duration=odo_mes.TimeStamp-Odo_last_TimeStamp;

            Quaterniond q_last(Odo_last_qw,Odo_last_qx,Odo_last_qy,Odo_last_qz);//q为b_last2n,用时需共扼
            Quaterniond q_current(odo_mes.quatw,odo_mes.quatx,odo_mes.quaty,odo_mes.quatz);//q为b2n,用时需共扼
            //std::cout<<"q_current is: "<<q_current.w()<<q_current.x()<<q_current.y()<<q_current.z()<<std::endl;
            Quaterniond q_delta=q_current.conjugate()*q_last;//b_last2b
            double px_delta=odo_mes.positionx-Odo_last_px;
            double py_delta=odo_mes.positiony-Odo_last_py;
            double pz_delta=odo_mes.positionz-Odo_last_pz;
            Vector3d p_delta;
            p_delta<<px_delta,py_delta,pz_delta;//原始东北天位移
//		Matrix3d R_temp=q_current.matrix().transpose();
//		double lidaryaw=fusion.R2euler(R_temp)(2);
            std::cout<<"before p_delta is ："<<p_delta<<std::endl;
            p_delta=fusion.euler2R(0,0,-1*PPI/60)*p_delta;
//		std::cout<<"before lidaryaw is ："<<lidaryaw*180/PPI<<std::endl;

            if(Odo_duration>0.01)
            {
                //if(flag_IMU_valid)
                {
//				p_delta=q_current.conjugate()*p_delta;//车体系下位移
//				std::cout<<"body fix frame p_delta is ："<<p_delta<<std::endl;
//				Matrix3d Rn2b=fusion.euler2R(imu_mes.roll*PPI/180,imu_mes.pitch*PPI/180,imu_mes.yaw*PPI/180);
//				Matrix3d Rb2n=Rn2b.transpose();
//				p_delta=Rb2n*p_delta;//东北天位移
//				std::cout<<"after p_delta is ："<<p_delta<<std::endl;
//				std::cout<<"after yaw is ："<<imu_mes.yaw<<std::endl;
                    //fusion.Lidar_measure_update_resetangle(imu_mes.roll*PPI/180,imu_mes.pitch*PPI/180,imu_mes.yaw*PPI/180);
                    fusion.Lidar_time_update(Odo_duration,p_delta(0),p_delta(1),p_delta(2),Quaterniond::Identity(),gps_mes.DGPSState);
                    flag_IMU_valid=false;
                }
                //else
                  //  fusion.Lidar_time_update(Odo_duration,p_delta(0),p_delta(1),p_delta(2),q_delta,gps_mes.DGPSState);
            }

            Odo_last_px=odo_mes.positionx;
            Odo_last_py=odo_mes.positiony;
            Odo_last_pz=odo_mes.positionz;
            Odo_last_qx=odo_mes.quatx;
            Odo_last_qy=odo_mes.quaty;
            Odo_last_qz=odo_mes.quatz;
            Odo_last_qw=odo_mes.quatw;
            Odo_last_TimeStamp=odo_mes.TimeStamp;
            ROS_INFO_STREAM("Odo msgs updated");
            flag_Lidar_valid=true;

            //拿出计算后的经纬度
            sensor_fusion::GpswithHeading after_fusion;

            after_fusion.gps.header.frame_id="global_earth_frame";
            //after_fusion.gps.header.stamp=Odometermsgs.odometry.header.stamp;
            after_fusion.gps.header.stamp=Odometermsgs->odometry.header.stamp;
            after_fusion.gps.latitude=fusion.getLidarLa()*180/PPI;
            after_fusion.gps.longitude=fusion.getLidarLo()*180/PPI;
            after_fusion.gps.altitude=fusion.getaltitude();
            after_fusion.heading=fusion.getyaw()*180/PPI;
            after_fusion.roll=fusion.getroll()*180/PPI;
            after_fusion.pitch=fusion.getpitch()*180/PPI;
            after_fusion.gps.status.status=gps_mes.DGPSState;
            cout<<"after_fusion.gps.status.status is: "<<after_fusion.gps.status.status<<endl;
            gps_mes.DGPSState='1';
            pub.publish(after_fusion);
            ROS_INFO_STREAM("Lidar_filter_output advertised");
        }

    }

    ros::NodeHandle m_nodehandle;
    ros::Subscriber subIMU;//=nh.subscribe("gpsdata",1,&IMU_1stMessageRecieved);
    ros::Subscriber subGPS;//=nh.subscribe("GPSmsg",1,&GPSMessageRecieved);
    ros::Subscriber subOdo;//=nh.subscribe("lidar_preciseodometry_to_earth",1,&OdometerMessageRecieved);
    ros::Publisher pub;
    ros::Publisher pub_path_GPS;
    ros::Publisher pub_path_IMU;

    struct_GPS_Mes gps_mes;
    struct_IMU_Mes imu_mes;
    struct_ODOMETER_Mes odo_mes;
    struct_ECU_Mes ecu_mes;
    COdometry fusion;
    bool flag_IMU_valid;
    bool flag_Lidar_valid;
    nav_msgs::Path Path_GPS;
    nav_msgs::Path Path_IMU;
    int GPS_lost_count;
    double wheelbase;
};





int main(int argc,char **argv)
{
    ros::init(argc,argv,"lidar_filter");
    ros::NodeHandle nh;
    PostProcess postprocess(nh);

    /*
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
     */
    ros::spin();
    return 0;
}





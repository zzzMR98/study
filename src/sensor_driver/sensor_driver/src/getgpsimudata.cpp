#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "ins/AnalysisGPS_IMU.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "sensor_driver_msgs/InsVelocity.h"
//#include "ivcommon/transform/rigid_transform.h"

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"getgpsimudata");
  ros::NodeHandle nh;
  int UDPport;
  std::string comport;
  int baudrate;
  bool use_udp;
  bool use_serial;
  //nh.param<int>("UDPport",port,9001);

  ros::Publisher pub;
  pub=nh.advertise<sensor_driver_msgs::GpswithHeading>("gpsdata",50);
  
  ros::Publisher imu_pub;
  imu_pub=nh.advertise<sensor_msgs::Imu>("imudata", 50);
  
  ros::Publisher InsVelocity_pub;
  InsVelocity_pub= nh.advertise<sensor_driver_msgs::InsVelocity> ("insvelocity", 50);
  
  //ros::Rate loop_rate(1);
  CAnalysisGPS_IMU gpsreciever;
  //nh.getParam("UDPport",port);

  ros::param::get("~UDPport",UDPport);//~表示私有空间的参数
  ros::param::get("~comport",comport);
  ros::param::get("~baudrate",baudrate);
  ros::param::get("~use_serial",use_serial);
  ros::param::get("~use_udp",use_udp);
  
  if(use_udp)
  {
    bool creatlisten=gpsreciever.UDPInit(UDPport);//port 9003
    if(creatlisten)
	  ROS_INFO_STREAM("successfully init udp with port: "<<UDPport<<std::endl);
    else
    {
      ROS_INFO_STREAM("failed init udp");
      return 0;
      
    }
  }
  if(use_serial)
  {
    bool serial_request=gpsreciever.SerialInit(comport,baudrate,1000);
    if(serial_request)
	  ROS_INFO_STREAM("successfully init serial "<<comport<<"with baudrate: "<<baudrate);
    else
    {
      ROS_INFO_STREAM("failed init serial");
      return 0;
    }
  }
  
   ros::Rate loop_rate(100);
  bool status = ros::ok();
  while(status)
  {
	  gpsreciever.Update();
// 	  std::cout<<"update"<<std::endl;
// 	  std::cout<<"flag1"<<gpsreciever.Data_struct.GPFPDflag<<"  flag2"<<gpsreciever.Data_struct.GPFPDflag<<std::endl;
	  if(gpsreciever.Data_struct.GPFPDflag && gpsreciever.Data_struct.GTIMUflag)
	  {
// 	    if(gpsreciever.Data_struct.status[0]!=0){
// 	      ROS_INFO_STREAM("wait for initialization");
// 	      continue;
// 	    }
// 	      ROS_INFO_STREAM("ins status:"<<gpsreciever.Data_struct.statusstr.c_str());
	    gpsreciever.Data_struct.GPFPDflag=false;
	    gpsreciever.Data_struct.GTIMUflag=false;
	    sensor_driver_msgs::GpswithHeading gps;
	    gps.header.frame_id="gps_frame";
	    gps.header.stamp=ros::Time::now();
	    gps.gps.header=gps.header;

	    gps.gps.latitude=gpsreciever.Data_struct.dLatitude;
	    gps.gps.longitude=gpsreciever.Data_struct.dLongitude;
	    gps.gps.altitude=gpsreciever.Data_struct.daltitude;
	    gps.heading=gpsreciever.Data_struct.dheading;
	    
	    gps.roll=gpsreciever.Data_struct.droll;
	    gps.pitch=gpsreciever.Data_struct.dpitch;
	    
	    if(gps.heading<-180)
		gps.heading +=360;
	    if(gps.heading>180)
		gps.heading -=360;
	    gps.heading=-gps.heading;
	    
	    sensor_msgs::Imu imuout;
	    float roll = gpsreciever.Data_struct.droll*M_PI/180;
	    float pitch = gpsreciever.Data_struct.dpitch*M_PI/180;
	    float heading = gpsreciever.Data_struct.dheading*M_PI/180;
	    tf::Quaternion orientation=tf::createQuaternionFromRPY(roll, pitch, heading);
	    tf::quaternionTFToMsg(orientation,imuout.orientation);
	    imuout.header.stamp = gps.header.stamp;
	    imuout.header.frame_id="imu_frame";
	    double latitude=gpsreciever.Data_struct.dLatitude;
	    double temp_c=1 + 0.0052884*sin(latitude)*sin(latitude) - 0.0000059*sin(2*latitude)*sin(2*latitude);
	    gpsreciever.Data_struct.gravity=9.78046*temp_c - 0.000003086*gpsreciever.Data_struct.daltitude;
	    imuout.linear_acceleration.x = gpsreciever.Data_struct.AccX * gpsreciever.Data_struct.gravity;
	    imuout.linear_acceleration.y = gpsreciever.Data_struct.AccY * gpsreciever.Data_struct.gravity;
	    imuout.linear_acceleration.z = gpsreciever.Data_struct.AccZ * gpsreciever.Data_struct.gravity;
	    imuout.angular_velocity.x = gpsreciever.Data_struct.droX*M_PI/180;
	    imuout.angular_velocity.y = gpsreciever.Data_struct.droY*M_PI/180;
	    imuout.angular_velocity.z = gpsreciever.Data_struct.droZ*M_PI/180;	
	    
	    sensor_driver_msgs::InsVelocity insvelocity;
	    insvelocity.header.frame_id = "ins_frame";
	    insvelocity.header.stamp = imuout.header.stamp;
	    insvelocity.angular_velocity = imuout.angular_velocity;
	    Eigen::Vector3d globalvelocity;
	    globalvelocity.x() = gpsreciever.Data_struct.dVelE;
	    globalvelocity.y() = gpsreciever.Data_struct.dVelN;
	    globalvelocity.z() = gpsreciever.Data_struct.dVelSky;

	    
	    Eigen::Quaterniond inspose = RollPitchYaw(roll, pitch, heading);
	    Eigen::Vector3d velocity = inspose.inverse()*globalvelocity;
	    insvelocity.linear_velocity.x = velocity.x();
	    insvelocity.linear_velocity.y = velocity.y();
	    insvelocity.linear_velocity.z = velocity.z();
	    
	    
	    pub.publish(gps);
	    imu_pub.publish(imuout);
	    if(velocity.norm()<100)
	      InsVelocity_pub.publish(insvelocity);
	    
	    status = ros::ok();
	    
	  }
	  ros::spinOnce();
	loop_rate.sleep();
  }

}

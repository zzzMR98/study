#include <iostream>
#include "ros/ros.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/Image.h"

class Msg_Filter_Gps_Imu{
public:
    Msg_Filter_Gps_Imu(ros::NodeHandle& nh):_nh(nh){
        sub_gps.subscribe(_nh,"/GPSmsg",100);
        sub_imu.subscribe(_nh,"/gpsdata_imu",100);
        exact_sync_ = std::make_shared<ExactSync>(ExactPolicy(10),sub_gps,sub_imu);
        exact_sync_->registerCallback(boost::bind(&Msg_Filter_Gps_Imu::GPS_IMU_Callback, this, _1, _2));
        pub_gps_imu = _nh.advertise<sensor_driver_msgs::GpswithHeading>("/gpsdata_add",100);
    }

    void GPS_IMU_Callback(const sensor_driver_msgs::GpswithHeadingConstPtr& gpsIn,const sensor_driver_msgs::GpswithHeadingConstPtr& imuIn){
        sensor_driver_msgs::GpswithHeading gps_imu_msg;
        gps_imu_msg.header = gpsIn->header;
        gps_imu_msg.gps = gpsIn->gps;
        gps_imu_msg.heading = imuIn->heading;
        gps_imu_msg.pitch = imuIn->pitch;
        gps_imu_msg.roll = imuIn->roll;
        gps_imu_msg.mode = imuIn->mode;
        pub_gps_imu.publish(gps_imu_msg);
        ROS_INFO_ONCE("pub");
//        std::cout<<gpsIn->header.stamp.sec*1e-9<<std::endl<<imuIn->header.stamp.toSec()*1e-9<<std::endl<<std::endl;
//        ROS_INFO("GPS timestamp-IMU timestamp: %d",gpsIn->header.stamp.toSec()*1e-9-imuIn->header.stamp.toSec()*1e-9);
//        ROS_INFO("GPS timestamp: %d,IMU timestamp: %d",gpsIn->header.stamp.toSec()*1e-9,imuIn->header.stamp.toSec()*1e-9);
    }


    ::ros::NodeHandle& _nh;
    ros::Publisher pub_gps_imu;
    message_filters::Subscriber<sensor_driver_msgs::GpswithHeading> sub_gps;
    message_filters::Subscriber<sensor_driver_msgs::GpswithHeading> sub_imu;
    typedef message_filters::sync_policies::ApproximateTime<sensor_driver_msgs::GpswithHeading,sensor_driver_msgs::GpswithHeading> ExactPolicy;//ApproximateTime,ExactTime
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    std::shared_ptr<ExactSync> exact_sync_;

};

int main(int argc, char** argv){
    ros::init(argc,argv,"sync_gps_imu");
    ros::NodeHandle nh("~");
    Msg_Filter_Gps_Imu msg_get(nh);
    ros::spin();
}

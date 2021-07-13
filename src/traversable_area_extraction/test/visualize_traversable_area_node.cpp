// system
#include <string>
#include <memory>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// custom
#include "iv_slam_ros_msgs/TraversableArea.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/utm/utm.h"

using namespace std;

double a = 6378137;
double e2 = 0.0818192 * 0.0818192; //e的平方
::ivcommon::transform::GridZone gps_zone = ::ivcommon::transform::UTM_ZONE_AUTO;
::ivcommon::transform::Hemisphere hemi = ::ivcommon::transform::HEMI_NORTH;

typedef message_filters::sync_policies::ApproximateTime<
    iv_slam_ros_msgs::TraversableArea,
    nav_msgs::Odometry>
    ApproximateSyncPolicy;
typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
// clock_t cur_time;// = clock();
double cur_time;
long long int msg_count = 0;

// ROS_INFO("obstacle_detect process time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);

void message_filter_callback(
        const iv_slam_ros_msgs::TraversableArea::ConstPtr traversable_area_msg,
        const nav_msgs::Odometry::ConstPtr vehicle_pose_msg) {
    // ROS_INFO_ONCE("visualize_traversable_area_node callback running");
    // clock_t new_time = clock();
    msg_count += 1;
    double new_time = ros::Time::now().toSec();
    double fps = (double)msg_count / (new_time - cur_time);
    // double fps = (double)CLOCKS_PER_SEC / (double)(new_time - cur_time) * (double)msg_count;
    // std::cout << "new: " << new_time << ", old: " << cur_time << ", fps: " << fps << std::endl;
    bool traversable_area_empty = false;
    if (traversable_area_msg->width == 0 || traversable_area_msg->height == 0)
        traversable_area_empty = true;
    if (!traversable_area_empty) {
        traversable_area_empty = true;
        for (int i=0; i<traversable_area_msg->cells.size(); i++) {
            if (traversable_area_msg->cells[i] != 0) {
                traversable_area_empty = false;
                break;
            }
        }
    }
    if (traversable_area_empty) {
        ROS_WARN_THROTTLE(5, "[visualize_traversable_area_node] Empty map!");
        if (traversable_area_msg->width == 0 || traversable_area_msg->height == 0) {
            printf("map size zero\n");
            return;
        }
    }
    cv::Mat img = cv::Mat::zeros(cv::Size(traversable_area_msg->width, traversable_area_msg->height), CV_8UC3);
    for (int i=0; i<traversable_area_msg->height; i++) {
        for (int j=0; j<traversable_area_msg->width; j++) {
            switch (traversable_area_msg->cells[(traversable_area_msg->height-i)*traversable_area_msg->width+j])
            {
            case 1:
                img.at<cv::Vec3b>(i,j)[0] = 0;
                img.at<cv::Vec3b>(i,j)[1] = 255;
                img.at<cv::Vec3b>(i,j)[2] = 0;
                break;
            case 2:
                img.at<cv::Vec3b>(i,j)[0] = 255;
                img.at<cv::Vec3b>(i,j)[1] = 255;
                img.at<cv::Vec3b>(i,j)[2] = 255;
                break;
            case 4:
                img.at<cv::Vec3b>(i,j)[0] = 0;
                img.at<cv::Vec3b>(i,j)[1] = 0;
                img.at<cv::Vec3b>(i,j)[2] = 255;
                break;
            default:
                break;
            }
        }
    }
    double tem_submap_longitude = traversable_area_msg->triD_submap_pose.position.x * M_PI / 180.;
    double tem_submap_latitude = traversable_area_msg->triD_submap_pose.position.y * M_PI / 180.;
    double N, E;
    ::ivcommon::transform::geographic_to_grid(a, e2, tem_submap_latitude, tem_submap_longitude, &gps_zone, &hemi, &N, &E);
    ::ivcommon::transform::Rigid3d tem_map_pose(::ivcommon::transform::Rigid3d::Vector(E-50000, N, 
        traversable_area_msg->triD_submap_pose.position.z),
        ::ivcommon::transform::Rigid3d::Quaternion(
            traversable_area_msg->triD_submap_pose.orientation.w,
            traversable_area_msg->triD_submap_pose.orientation.x,
            traversable_area_msg->triD_submap_pose.orientation.y,
            traversable_area_msg->triD_submap_pose.orientation.z
        ));

    tem_submap_longitude = vehicle_pose_msg->pose.pose.position.x * M_PI / 180.;
    tem_submap_latitude = vehicle_pose_msg->pose.pose.position.y * M_PI / 180.;
    ::ivcommon::transform::geographic_to_grid(a, e2, tem_submap_latitude, tem_submap_longitude, &gps_zone, &hemi, &N, &E);
    ::ivcommon::transform::Rigid3d tem_latest_vechicle_pose(::ivcommon::transform::Rigid3d::Vector(E-50000, N, 
        vehicle_pose_msg->pose.pose.position.z),
        ::ivcommon::transform::Rigid3d::Quaternion(
            vehicle_pose_msg->pose.pose.orientation.w,
            vehicle_pose_msg->pose.pose.orientation.x,
            vehicle_pose_msg->pose.pose.orientation.y,
            vehicle_pose_msg->pose.pose.orientation.z
        ));
    ::ivcommon::transform::Rigid3d tem_relative_pose = tem_map_pose.inverse() * tem_latest_vechicle_pose;
    // cv::flip(img, img, 0);
    double resolution = traversable_area_msg->resolution;//0.2;
    int heightnum = traversable_area_msg->height / (10 / resolution);
    int widthnum = traversable_area_msg->width / (10 / resolution);
    for (int i = 0; i < heightnum;i++){
      cv::line(img, cv::Point(0, img.rows * i / heightnum),
        cv::Point(img.cols - 1, img.rows * i / heightnum), cvScalar(255, 0, 0));
    }
    for (int i = 1;i < widthnum;i++){
      cv::line(img, cv::Point(img.cols * i / widthnum, 0),
        cv::Point(img.cols * i / widthnum, img.rows - 1), cvScalar(255, 0, 0));
    }
    cv::circle(img, cv::Point(tem_relative_pose.translation().x() / traversable_area_msg->resolution
      + traversable_area_msg->triD_submap_pose_image_index_x,
      img.rows - 1 - (tem_relative_pose.translation().y() / traversable_area_msg->resolution
        + traversable_area_msg->triD_submap_pose_image_index_y)), 5,
      cv::Scalar(0, 255, 255), -1);

	//设置绘制文本的相关参数
	std::string text = std::to_string((int)fps) + " FPS";
	static int font_face = cv::FONT_HERSHEY_SIMPLEX; 
	static double font_scale = 0.4;
	static int thickness = 1;
	static int baseline;
	//获取文本框的长宽
	cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
	cv::putText(img, text, cv::Point(2, img.rows - text_size.height), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
    cv::imshow("visualize_traversable_area_node", img);
    cv::waitKey(1);
    // cur_time = new_time;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualize_traversable_area_node");
    ros::start();
    ROS_INFO("[visualize_traversable_area_node] launched.");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    string traversable_area_topic, vehicle_pose_topic;
    // pnh.param<string>("traversable_area_topic", traversable_area_topic, "final_traversable_area_topic");
    // ros::Subscriber traversable_area_sub = nh.subscribe<iv_slam_ros_msgs::TraversableArea>(traversable_area_topic, 1, traversable_area_callback);

    pnh.param<string>("traversable_area_topic", traversable_area_topic, "final_traversable_area_topic");
    pnh.param<string>("vehicle_pose_topic", vehicle_pose_topic, "final_vehicle_global_pose_topic");
    cur_time = ros::Time::now().toSec();
    message_filters::Subscriber<iv_slam_ros_msgs::TraversableArea> traversable_area_sub;
    message_filters::Subscriber<nav_msgs::Odometry> vehicle_pose_sub;
    traversable_area_sub.subscribe(nh, traversable_area_topic, 2);
    vehicle_pose_sub.subscribe(nh, vehicle_pose_topic, 2);
    std::shared_ptr<ApproximateSync> sync_;
    sync_.reset(new ApproximateSync(ApproximateSyncPolicy(300), traversable_area_sub, vehicle_pose_sub));
    sync_->registerCallback(boost::bind(&message_filter_callback, _1, _2));

    ROS_INFO("visualize_traversable_area_node start.");
    ros::spin();

}
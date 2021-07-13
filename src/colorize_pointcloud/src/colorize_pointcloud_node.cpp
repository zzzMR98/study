/**
 * @file colorize_pointcloud_node.cpp
 * @author sunsky (you@domain.com)
 * @brief assign color to pointcloud by corresponding image pixels
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

// C++
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <boost/thread.hpp>
#include <map>
#include <iomanip>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// #include"opencv2/opencv.hpp"

// Eigen
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>

using namespace std;

class ColorizePointCloudNode {
public:
    ColorizePointCloudNode(ros::NodeHandle& nh, string pointcloud_topic,
        string pub_pointcloud_topic, string front_camera_topic, 
        string front_camera_lidar_extrinsic_path, string left_camera_topic, 
        string left_camera_lidar_extrinsic_path, string right_camera_topic, 
        string right_camera_lidar_extrinsic_path):
            image_transport_(nh),
            pointcloud_topic_(pointcloud_topic),
            pub_pointcloud_topic_(pub_pointcloud_topic),
            front_camera_topic_(front_camera_topic),
            left_camera_topic_(left_camera_topic),
            right_camera_topic_(right_camera_topic) {
        if (front_camera_topic_ == "") front_camera_on = false;
        else front_camera_on = true;
        if (left_camera_topic_ == "") left_camera_on = false;
        else left_camera_on = true;
        if (right_camera_topic_ == "") right_camera_on = false;
        else right_camera_on = true;

        ros::NodeHandle pnh("~");
        pnh.getParam("display_image", display_image);
        if (front_camera_on) {
            front_project_matrix = readCalibFile(front_camera_lidar_extrinsic_path);
            vector<float> x_range, y_range, z_range;
            pnh.getParam("front_x_range", x_range);
            pnh.getParam("front_y_range", y_range);
            pnh.getParam("front_z_range", z_range);
            front_projection_range = {x_range, y_range, z_range};
        }
        if (left_camera_on) {
            left_project_matrix = readCalibFile(left_camera_lidar_extrinsic_path);
            vector<float> x_range, y_range, z_range;
            pnh.getParam("left_x_range", x_range);
            pnh.getParam("left_y_range", y_range);
            pnh.getParam("left_z_range", z_range);
            left_projection_range = {x_range, y_range, z_range};
        }
        if (right_camera_on) {
            right_project_matrix = readCalibFile(right_camera_lidar_extrinsic_path);
            vector<float> x_range, y_range, z_range;
            pnh.getParam("right_x_range", x_range);
            pnh.getParam("right_y_range", y_range);
            pnh.getParam("right_z_range", z_range);
            right_projection_range = {x_range, y_range, z_range};
        }
        color_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_pointcloud_topic_, 5);
        if (front_camera_on && left_camera_on && right_camera_on) {
            init_message_filter_subscriber(nh);
        }
        else {
            init_individual_subscribers(nh);
        }

    }

    void message_filter_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                                const sensor_msgs::CompressedImageConstPtr& front_img_msg,
                                const sensor_msgs::CompressedImageConstPtr& left_img_msg,
                                const sensor_msgs::CompressedImageConstPtr& right_img_msg) {
        cv::Mat front_img, left_img, right_img;
        try {
            cv::Mat left_img_bayer, right_img_bayer;
            front_img = cv_bridge::toCvCopy(front_img_msg, sensor_msgs::image_encodings::BGR8)->image;
            left_img_bayer = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::MONO8)->image;
            right_img_bayer = cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::MONO8)->image;
            // printf("left image bayer depth: %d\n", left_img_bayer.channels());
            // printf("right image bayer depth: %d\n", right_img_bayer.channels());
            cv::cvtColor(left_img_bayer, left_img, cv::COLOR_BayerRG2RGB);
            cv::cvtColor(right_img_bayer, right_img, cv::COLOR_BayerRG2RGB);
            // printf("left image depth: %d\n", left_img.channels());
            // printf("right image depth: %d\n", right_img.channels());
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        static int front_img_width=front_img.cols, front_img_height=front_img.rows, left_img_width=left_img.cols, left_img_height=left_img.rows, right_img_width=right_img.cols, right_img_height=right_img.rows;
        pcl::PointCloud<pcl::PointXYZ> pointcloud;
        pcl::PointCloud<pcl::PointXYZRGB> color_pointcloud;
        pcl::fromROSMsg(*pointcloud_msg, pointcloud);
        // vector<int> indices;
        // pcl::removeNaNFromPointCloud(pointcloud, pointcloud, indices);
        pcl::copyPointCloud(pointcloud, color_pointcloud);

        Eigen::MatrixXf pointcloud_mat;
        size_t points_count = color_pointcloud.points.size();
        pointcloud_mat.resize(4, points_count);
        for (size_t i = 0; i < points_count; i++)
        {
            pointcloud_mat(0, i) = color_pointcloud.points[i].x;
            pointcloud_mat(1, i) = color_pointcloud.points[i].y;
            pointcloud_mat(2, i) = color_pointcloud.points[i].z;
        }
        pointcloud_mat.block(3, 0, 1, points_count) = Eigen::MatrixXf::Constant(1, points_count, 1);
        Eigen::MatrixXf front_projected_points = front_project_matrix * pointcloud_mat;
        Eigen::MatrixXf left_projected_points = left_project_matrix * pointcloud_mat;
        Eigen::MatrixXf right_projected_points = right_project_matrix * pointcloud_mat;
        for (size_t i=0; i<points_count; i++) {
            float x=color_pointcloud.points[i].x, y=color_pointcloud.points[i].y, z=color_pointcloud.points[i].z;
            if ((front_projection_range[0][0]<x && x<front_projection_range[0][1]) &&
                    (front_projection_range[1][0]<y && y<front_projection_range[1][1]) &&
                    (front_projection_range[2][0]<z && z<front_projection_range[2][1])) {
                if (front_projected_points(2, i) > 0)
                {
                    // front_projected_points(0, i) /= front_projected_points(2, i);
                    // front_projected_points(1, i) /= front_projected_points(2, i);
                    float u=front_projected_points(0, i) / front_projected_points(2, i), v=front_projected_points(1, i) / front_projected_points(2, i);
                    if ((u >= 0) && 
                        (u <= front_img_width - 1) 
                        && (v >= 0) && 
                        (v <= front_img_height - 1))
                    {
                        color_pointcloud.points[i].r = front_img.at<cv::Vec3b>(v,u)[2];
                        color_pointcloud.points[i].g = front_img.at<cv::Vec3b>(v,u)[1];
                        color_pointcloud.points[i].b = front_img.at<cv::Vec3b>(v,u)[0];
                    }
                }
            }
            if ((left_projection_range[0][0]<x && x<left_projection_range[0][1]) &&
                    (left_projection_range[1][0]<y && y<left_projection_range[1][1]) &&
                    (left_projection_range[2][0]<z && z<left_projection_range[2][1])) {
                if (left_projected_points(2, i) > 0)
                {
                    // left_projected_points(0, i) /= left_projected_points(2, i);
                    // left_projected_points(1, i) /= left_projected_points(2, i);
                    float u=left_projected_points(0, i) / left_projected_points(2, i), v=left_projected_points(1, i) / left_projected_points(2, i);
                    if ((u >= 0) && 
                        (u <= left_img_width - 1) 
                        && (v >= 0) && 
                        (v <= left_img_height - 1))
                    {
                        color_pointcloud.points[i].r = left_img.at<cv::Vec3b>(v,u)[2];
                        color_pointcloud.points[i].g = left_img.at<cv::Vec3b>(v,u)[1];
                        color_pointcloud.points[i].b = left_img.at<cv::Vec3b>(v,u)[0];
                    }
                }
            }
            if ((right_projection_range[0][0]<x && x<right_projection_range[0][1]) &&
                    (right_projection_range[1][0]<y && y<right_projection_range[1][1]) &&
                    (right_projection_range[2][0]<z && z<right_projection_range[2][1])) {
                if (right_projected_points(2, i) > 0)
                {
                    // right_projected_points(0, i) /= right_projected_points(2, i);
                    // right_projected_points(1, i) /= right_projected_points(2, i);
                    float u=right_projected_points(0, i) / right_projected_points(2, i), v=right_projected_points(1, i) / right_projected_points(2, i);
                    if ((u >= 0) && 
                        (u <= right_img_width - 1) 
                        && (v >= 0) && 
                        (v <= right_img_height - 1))
                    {
                        color_pointcloud.points[i].r = right_img.at<cv::Vec3b>(v,u)[2];
                        color_pointcloud.points[i].g = right_img.at<cv::Vec3b>(v,u)[1];
                        color_pointcloud.points[i].b = right_img.at<cv::Vec3b>(v,u)[0];
                    }
                }
            }
        }
        if (color_pointcloud_pub.getNumSubscribers()) {
            sensor_msgs::PointCloud2 color_pointcloud_msg;
            color_pointcloud_msg.header = pointcloud_msg->header;
            pcl::toROSMsg(color_pointcloud, color_pointcloud_msg);
            color_pointcloud_pub.publish(color_pointcloud_msg);
            ROS_DEBUG("rgb pointcloud published");
        }
        if (display_image) {
            cv::Mat combined_img1, combined_img2;
            cv::Mat left_img_resized, right_img_resized;
            cv::resize(left_img, left_img_resized, cv::Size(front_img.cols, front_img.rows));
            cv::resize(right_img, right_img_resized, cv::Size(front_img.cols, front_img.rows));
            hconcat(left_img, front_img, combined_img1);
            hconcat(combined_img1, right_img, combined_img2);
            cv::namedWindow("img", 0);
            cv::imshow("img", combined_img2);
            cv::waitKey(1);
        }
    }

    void frontCameraCallback(const sensor_msgs::ImageConstPtr& _msg) {
        cv_bridge::CvImagePtr cam_image;

        try {
            cam_image = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BAYER_RGGB8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("[fontCameraCallback] cv_bridge exception: %s", e.what());
            return;
        }

        if (cam_image)
        {
            {
                boost::unique_lock<boost::shared_mutex> lockImageCallback(frontMutexImageCallback_);
                frontCamImageHeader_ = _msg->header;
                frontCamImageCopy_.release();
                frontCamImageCopy_ = cam_image->image.clone();
            }
            {
                boost::unique_lock<boost::shared_mutex> lockImageStatus(frontMutexImageCallback_);
            }
            // int frameWidth_ = cam_image->image.size().width;
            // int frameHeight_ = cam_image->image.size().height;
        }
        return;
    }
    void leftCameraCallback(const sensor_msgs::ImageConstPtr& _msg) {
        cv_bridge::CvImagePtr cam_image;

        try {
            cam_image = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BAYER_RGGB8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("[fontCameraCallback] cv_bridge exception: %s", e.what());
            return;
        }

        if (cam_image)
        {
            {
                boost::unique_lock<boost::shared_mutex> lockImageCallback(leftMutexImageCallback_);
                leftCamImageHeader_ = _msg->header;
                leftCamImageCopy_.release();
                leftCamImageCopy_ = cam_image->image.clone();
            }
            {
                boost::unique_lock<boost::shared_mutex> lockImageStatus(leftMutexImageCallback_);
            }
            // int frameWidth_ = cam_image->image.size().width;
            // int frameHeight_ = cam_image->image.size().height;
        }
        return;
    }
    void rightCameraCallback(const sensor_msgs::ImageConstPtr& _msg) {
        cv_bridge::CvImagePtr cam_image;

        try {
            cam_image = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BAYER_RGGB8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("[fontCameraCallback] cv_bridge exception: %s", e.what());
            return;
        }

        if (cam_image)
        {
            {
                boost::unique_lock<boost::shared_mutex> lockImageCallback(rightMutexImageCallback_);
                rightCamImageHeader_ = _msg->header;
                rightCamImageCopy_.release();
                rightCamImageCopy_ = cam_image->image.clone();
            }
            {
                boost::unique_lock<boost::shared_mutex> lockImageStatus(rightMutexImageCallback_);
            }
            // int frameWidth_ = cam_image->image.size().width;
            // int frameHeight_ = cam_image->image.size().height;
        }
        return;
    }
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& _msg) {
        // 
    }

private:
    bool display_image;
    image_transport::ImageTransport image_transport_;
    string pointcloud_topic_, pub_pointcloud_topic_,
        front_camera_topic_, front_camera_lidar_extrinsic_path,
        left_camera_topic_, left_camera_lidar_extrinsic_path, 
        right_camera_topic_, right_camera_lidar_extrinsic_path;
    Eigen::MatrixXf front_project_matrix, left_project_matrix, right_project_matrix;
    bool front_camera_on, left_camera_on, right_camera_on;
    std_msgs::Header frontCamImageHeader_, leftCamImageHeader_, rightCamImageHeader_;
    cv::Mat frontCamImageCopy_, leftCamImageCopy_, rightCamImageCopy_;
    boost::shared_mutex frontMutexImageCallback_, leftMutexImageCallback_, rightMutexImageCallback_;
    vector<vector<float>> front_projection_range, left_projection_range, right_projection_range;

    // subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> front_img_sub, left_img_sub, right_img_sub;
    message_filters::Subscriber<sensor_msgs::CompressedImage> front_img_compressed_sub, left_img_compressed_sub, right_img_compressed_sub;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::Image,
        sensor_msgs::Image,
        sensor_msgs::Image>
        ApproximateSyncPolicy2;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::CompressedImage,
        sensor_msgs::CompressedImage,
        sensor_msgs::CompressedImage>
        ApproximateSyncPolicy;
    typedef message_filters::Synchronizer<ApproximateSyncPolicy2> ApproximateSync2;
    typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> sync_;
    boost::shared_ptr<ApproximateSync2> sync2_;

    // publisher
    ros::Publisher color_pointcloud_pub;


    Eigen::MatrixXf readCalibFile(string calib_file_path) {
        cv::FileStorage calibParamFile(calib_file_path, cv::FileStorage::READ);
        if(!calibParamFile.isOpened())
        {
            std::cerr << "Can't Open File: " << calib_file_path << std::endl;
            // return Eigen::MatrixXf::Zero(3,4);
        }
        cv::Mat RT_base_to_cam; // 4*4
        cv::Mat Intrins_matrix; // 3*4
        calibParamFile["RT"] >> RT_base_to_cam;
        calibParamFile["INTRINS"] >> Intrins_matrix;
        cv::Mat temp = Intrins_matrix * RT_base_to_cam;
        Eigen::MatrixXf transform_matrix;
        cv::cv2eigen(temp, transform_matrix);
        ROS_INFO_STREAM("transform_matrix: " << transform_matrix << std::endl);
        return transform_matrix;
    }

    void init_individual_subscribers(ros::NodeHandle& nh) {
        std::string compressed_str = "compressed";
        if (front_camera_on) {
            image_transport::Subscriber front_camera_sub;// = image_transport_.subscribe(front_camera_topic_, 1, frontCameraCallback);
            std::string::size_type position = front_camera_topic_.find(compressed_str);
            if (position == front_camera_topic_.npos) {
                // compressed_image = false;
                front_camera_sub = image_transport_.subscribe
                        (front_camera_topic_, 1, &ColorizePointCloudNode::frontCameraCallback, this);
            }
            else {
                // compressed_image = true;
                image_transport::TransportHints hints(compressed_str);
                front_camera_sub = image_transport_.subscribe
                        (front_camera_topic_.substr(0, position-1), 1, &ColorizePointCloudNode::frontCameraCallback, this, hints);
            }
        }
        if (left_camera_on) {
            image_transport::Subscriber left_camera_sub;// = image_transport_.subscribe(left_camera_topic_, 1, leftCameraCallback);
            std::string::size_type position = left_camera_topic_.find(compressed_str);
            if (position == left_camera_topic_.npos) {
                // compressed_image = false;
                left_camera_sub = image_transport_.subscribe
                        (left_camera_topic_, 1, &ColorizePointCloudNode::leftCameraCallback, this);
            }
            else {
                // compressed_image = true;
                image_transport::TransportHints hints(compressed_str);
                left_camera_sub = image_transport_.subscribe
                        (left_camera_topic_.substr(0, position-1), 1, &ColorizePointCloudNode::leftCameraCallback, this, hints);
            }
        }
        if (right_camera_on) {
            image_transport::Subscriber right_camera_sub;// = image_transport_.subscribe(right_camera_topic_, 1, rightCameraCallback);
            std::string::size_type position = right_camera_topic_.find(compressed_str);
            if (position == right_camera_topic_.npos) {
                // compressed_image = false;
                right_camera_sub = image_transport_.subscribe
                        (right_camera_topic_, 1, &ColorizePointCloudNode::rightCameraCallback, this);
            }
            else {
                // compressed_image = true;
                image_transport::TransportHints hints(compressed_str);
                right_camera_sub = image_transport_.subscribe
                        (right_camera_topic_.substr(0, position-1), 1, &ColorizePointCloudNode::rightCameraCallback, this, hints);
            }
        }
        ros::Subscriber pointcloud_sub2 = nh.subscribe<sensor_msgs::PointCloud2>
            (pointcloud_topic_, 1, boost::bind(&ColorizePointCloudNode::pointcloudCallback, this, _1));//经过筛选且转换之后的点云
    }

    /**
     * @brief initialize message_filter subscriber of pointcloud and image
     * 
     * @param nh 
     */
    void init_message_filter_subscriber(ros::NodeHandle& nh) {
        // if (front_camera_topic_.find("compressed") == string::npos) {
        //     front_img_sub.subscribe(nh, front_camera_topic_, 2);
        //     left_img_sub.subscribe(nh, left_camera_topic_, 2);
        //     right_img_sub.subscribe(nh, right_camera_topic_, 2);
        //     sync2_.reset(new ApproximateSync2(ApproximateSyncPolicy2(300), pointcloud_sub, front_img_compressed_sub, left_img_compressed_sub, right_img_compressed_sub));
        //     sync2_->registerCallback(boost::bind(&ColorizePointCloudNode::message_filter_callback2,
        //                                         this, _1, _2, _3, _4));
        // }
        // else {
            pointcloud_sub.subscribe(nh, pointcloud_topic_, 2);
            front_img_compressed_sub.subscribe(nh, front_camera_topic_, 2);
            left_img_compressed_sub.subscribe(nh, left_camera_topic_, 2);
            right_img_compressed_sub.subscribe(nh, right_camera_topic_, 2);
            sync_.reset(new ApproximateSync(ApproximateSyncPolicy(300), pointcloud_sub, front_img_compressed_sub, left_img_compressed_sub, right_img_compressed_sub));
            sync_->registerCallback(boost::bind(&ColorizePointCloudNode::message_filter_callback,
                                                this, _1, _2, _3, _4));
        // }

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "colorize_pointcloud_node");
    ros::start();
    ros::NodeHandle nh;
    string pointcloud_topic, pub_pointcloud_topic, front_camera_topic, left_camera_topic, right_camera_topic;
    bool front_camera_on, left_camera_on, right_camera_on;
    string front_camera_lidar_extrinsic_path, left_camera_lidar_extrinsic_path, right_camera_lidar_extrinsic_path;
    ros::param::get("~pointcloud_topic", pointcloud_topic);
    ros::param::get("~pub_pointcloud_topic", pub_pointcloud_topic);
    ros::param::get("~front_camera_on", front_camera_on);
    ROS_INFO_STREAM("LiDAR pointcloud topic: " << pointcloud_topic);
    if (front_camera_on) {
        ros::param::get("~front_camera_topic", front_camera_topic);
        ros::param::get("~front_camera_lidar_extrinsic_path", front_camera_lidar_extrinsic_path);
        ROS_INFO_STREAM("front camera LiDAR calibration file: " << front_camera_lidar_extrinsic_path);
    }
    ros::param::get("~left_camera_on", left_camera_on);
    if (left_camera_on) {
        ros::param::get("~left_camera_topic", left_camera_topic);
        ros::param::get("~left_camera_lidar_extrinsic_path", left_camera_lidar_extrinsic_path);
        ROS_INFO_STREAM("left camera LiDAR calibration file: " << left_camera_lidar_extrinsic_path);
    }
    ros::param::get("~right_camera_on", right_camera_on);
    if (right_camera_on) {
        ros::param::get("~right_camera_topic", right_camera_topic);
        ros::param::get("~right_camera_lidar_extrinsic_path", right_camera_lidar_extrinsic_path);
        ROS_INFO_STREAM("right camera LiDAR calibration file: " << right_camera_lidar_extrinsic_path);
    }
    ColorizePointCloudNode node(nh, 
        pointcloud_topic, pub_pointcloud_topic,
        front_camera_topic, front_camera_lidar_extrinsic_path,
        left_camera_topic, left_camera_lidar_extrinsic_path,
        right_camera_topic, right_camera_lidar_extrinsic_path);
    
    ros::spin();
}
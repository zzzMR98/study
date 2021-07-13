// system
#include <string>
#include <memory>

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// custom
#include "covgrid_slam_msgs/LidarOdometryForMapping.h"
#include "iv_slam_ros_msgs/TraversableArea.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    iv_slam_ros_msgs::TraversableArea,
    covgrid_slam_msgs::LidarOdometryForMapping>
    ApproximateSyncPolicy;
typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;

void message_filter_callback(
        const iv_slam_ros_msgs::TraversableArea::ConstPtr traversable_area_msg,
        const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr lidar_odom_msg) {
    ROS_INFO_ONCE("test_traversable_area_node callback running");
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
        ROS_WARN("[test_traversable_area_node] Empty map!");
        printf("seq: %d, mode: %d, index: [", lidar_odom_msg->header.seq, lidar_odom_msg->mode);
        for (const int& x : lidar_odom_msg->indexs) printf("%d,", x);
        printf("]. stamp:");
        cout << lidar_odom_msg->header.stamp << "\n";
        if (traversable_area_msg->width == 0 || traversable_area_msg->height == 0)
            printf("map size zero\n");
        // cout << *lidar_odom_msg << endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_traversable_area_node");
    ros::start();
    ROS_INFO("entering test_traversable_area_node.");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    string traversable_area_topic, lidar_odom_topic;
    pnh.param<string>("traversable_area_topic", traversable_area_topic, "final_traversable_area_topic");
    pnh.param<string>("lidar_odom_topic", lidar_odom_topic, "lidar_odometry_for_mapping");
    message_filters::Subscriber<iv_slam_ros_msgs::TraversableArea> traversable_area_sub;
    message_filters::Subscriber<covgrid_slam_msgs::LidarOdometryForMapping> lidar_odom_sub;
    traversable_area_sub.subscribe(nh, traversable_area_topic, 2);
    lidar_odom_sub.subscribe(nh, lidar_odom_topic, 2);
    std::shared_ptr<ApproximateSync> sync_;
    sync_.reset(new ApproximateSync(ApproximateSyncPolicy(300), traversable_area_sub, lidar_odom_sub));
    sync_->registerCallback(boost::bind(&message_filter_callback, _1, _2));
    ROS_INFO("test_traversable_area_node start.");
    ros::spin();

}
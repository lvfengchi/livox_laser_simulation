/*
 * Created on Sun Aug 15 2021
 *
 * Author: EpsAvlc
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "livox_laser_simulation/livox_point_xyzrtl.h"

void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg, std::vector<ros::Publisher>& pc_line_publishers) {
  pcl::PointCloud<pcl::LivoxPointXyzrtl> pc;
  pcl::fromROSMsg(*pc_msg, pc);
  std::vector<pcl::PointCloud<pcl::LivoxPointXyzrtl>> pc_lines(6);
  for (int i = 0; i < 6; ++i) {
    pc_lines[i].reserve(pc.size() / 6);
  }
  for (int i = 0; i < pc.size(); ++i) {
    pc_lines[pc[i].line].push_back(pc[i]);
  }
  for (int i = 0; i < 6; ++i) {
    sensor_msgs::PointCloud2 pc_line_msg;
    pcl::toROSMsg(pc_lines[i], pc_line_msg);
    pc_line_msg.header = pc_msg->header;
    pc_line_publishers[i].publish(pc_line_msg);
  }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "point_line_test");
    ros::NodeHandle nh;
    std::vector<ros::Publisher> pc_line_publishers;
    for (int i = 0; i < 6; ++i) {
        ros::Publisher pc_line_pub = nh.advertise<sensor_msgs::PointCloud2>("scan_line_" + std::to_string(i), 1);
        pc_line_publishers.push_back(std::move(pc_line_pub));
    }
    ros::Subscriber pc_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("scan", 1, boost::bind(callback, _1, pc_line_publishers));
    ros::spin();
    return 0;
}

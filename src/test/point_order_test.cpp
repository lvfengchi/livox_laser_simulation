/*
 * Created on Sat Aug 14 2021
 *
 * Author: EpsAvlc
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <iostream>
#include <string>

#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_points_plugin.h"

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas,
                             std::vector<gazebo::AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (auto &data : datas) {
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_order_test");
    ros::NodeHandle nh;
    ros::Publisher points_pub;
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("livox_points_order", 1);

    std::string mid40_path = ros::package::getPath("livox_laser_simulation") + "/scan_mode/mid40.csv";
    CsvReader reader;
    std::vector<std::vector<double>> datas;
    if (!reader.ReadCsvFile(mid40_path, datas)) {
        ROS_ERROR("Can't read csv file.");
    }
    std::vector<gazebo::AviaRotateInfo> avia_data;
    convertDataToRotateInfo(datas, avia_data);

    int pt_num = datas.size();
    std::cout << pt_num << std::endl;
    int split_num = 96000;
    int split_size = pt_num / split_num;

    pcl::PointCloud<pcl::PointXYZ> points;
    ros::Rate loop(20);
    float r = 3;
    pcl::PointXYZ tmp;
    for (int i = 0; i < split_num && ros::ok(); ++i) {
        for (int j = 0; j < split_size; ++j) {
            int ind = i * split_size + j;
            gazebo::math::Quaternion ray;
            ray.SetFromEuler(gazebo::math::Vector3(0.0, avia_data[ind].zenith, avia_data[ind].azimuth));
            auto axis = ray * gazebo::math::Vector3(1.0, 0.0, 0.0);
            auto point = r * axis;
            tmp.x = point.x;
            tmp.y = point.y;
            tmp.z = point.z;
            points.push_back(tmp);
        }
        sensor_msgs::PointCloud2 points_msg;
        pcl::toROSMsg(points, points_msg);
        points_msg.header.frame_id = "world";
        points_msg.header.stamp = ros::Time::now();
        points_pub.publish(points_msg);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

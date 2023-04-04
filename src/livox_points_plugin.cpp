//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <livox_ros_driver/CustomMsg.h>
#include <limits>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/livox_point_xyzrtl.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (int i = 0; i < datas.size(); ++i) {
        auto &data = datas[i];
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
            avia_infos.back().line = i % 6;
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    sdfPtr = sdf;
    const auto rayElem = sdfPtr->GetElement("ray");
    const auto scanElem = rayElem->GetElement("scan");
    const auto rangeElem = rayElem->GetElement("range");

    int argc = 0;
    char **argv = nullptr;
    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
    ROS_INFO_STREAM("ros topic name: " << curr_scan_topic);

    raySensor = _parent;
    const auto sensor_pose = raySensor->Pose();
    SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 1);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    parentEntity = world->EntityByName(_parent->ParentName());
    const auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    downSample = std::max(downSample, static_cast<std::int64_t>(1));
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);

    namespace_ = sdfPtr->Get<std::string>("robotNamespace");
    ROS_INFO_STREAM("namespace: " << namespace_);
    publishPointCloudType = sdfPtr->Get<int>("publish_pointcloud_type");
    ROS_INFO_STREAM("publish_pointcloud_type: " << publishPointCloudType);
    ros::init(argc, argv, curr_scan_topic);
    rosNode = std::make_shared<ros::NodeHandle>(namespace_);
    switch (publishPointCloudType) {
        case SENSOR_MSG_POINT_CLOUD:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud>(curr_scan_topic, 1);
            break;
        case SENSOR_MSG_POINT_CLOUD2_POINTXYZI:
        case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTL:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 1);
            break;
        case LIVOX_ROS_DRIVER_CUSTOM_MSG:
            rosPointPub = rosNode->advertise<livox_ros_driver::CustomMsg>(curr_scan_topic, 1);
            break;
        default:
            break;
    }

    visualize = sdfPtr->Get<bool>("visualize");

    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    const auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        const int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaternion<double> ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        const auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
}

void LivoxPointsPlugin::OnNewLaserScans() {
    GZ_ASSERT(rayShape, "RayShape is null");
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());

        switch (publishPointCloudType) {
            case SENSOR_MSG_POINT_CLOUD:
                PublishPointCloud(points_pair);
                break;
            case SENSOR_MSG_POINT_CLOUD2_POINTXYZI:
                PublishPointCloud2XYZI(points_pair);
                break;
            case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTL:
                PublishPointCloud2XYZRTL(points_pair);
                break;
            case LIVOX_ROS_DRIVER_CUSTOM_MSG:
                PublishLivoxROSDriverCustomMsg(points_pair);
                break;
            default:
                break;
        }
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    const auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    const auto offset = laserCollision->RelativePose();
    const int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    const auto ray_size = rays.size();
    points_pair.clear();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex; k < end_index; k += downSample) {
        const auto index = k % maxPointSize;
        const auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        const auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
    if (currStartIndex > maxPointSize) {
        currStartIndex -= maxPointSize;
    }
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}
void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                  const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
}

void LivoxPointsPlugin::PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    const auto rayCount = RayCount();
    const auto verticalRayCount = VerticalRayCount();
    const auto angle_min = AngleMin().Radian();
    const auto angle_incre = AngleResolution();
    const auto verticle_min = VerticalAngleMin().Radian();
    const auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud scan_point;
    scan_point.header.stamp = ros::Time::now();
    scan_point.header.frame_id = "livox";
    auto &scan_points = scan_point.points;
    for (const auto &[p_index, rotate_info] : points_pair) {  // std::pair<int, gazebo::AviaRotateInfo>
        const int verticle_index = roundf((rotate_info.zenith - verticle_min) / verticle_incre);
        const int horizon_index = roundf((rotate_info.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            const auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            const auto range = rayShape->GetRange(p_index);
            const auto intensity = rayShape->GetRetro(p_index);
            if (range >= RangeMax() || range <= RangeMin()) continue;
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

            const auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            const auto point = range * axis;
            scan_points.emplace_back();
            scan_points.back().x = point.X();
            scan_points.back().y = point.Y();
            scan_points.back().z = point.Z();
        }
    }
    // Only publish scan point if there are subscribers AND if there are points to publish
    // This prevents the "empty scan" warning in the robot side from being printed
    if(rosPointPub.getNumSubscribers() > 0 && !scan_point.points.empty()) {
        rosPointPub.publish(scan_point);
    }
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZI(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    const auto rayCount = RayCount();
    const auto verticalRayCount = VerticalRayCount();
    const auto angle_min = AngleMin().Radian();
    const auto angle_incre = AngleResolution();
    const auto verticle_min = VerticalAngleMin().Radian();
    const auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;

    pcl::PointCloud<pcl::PointXYZI> pc;
    pc.points.resize(points_pair.size());
    const auto timestamp = ros::Time::now();
    int pt_count = 0;
#pragma omp parallel for
    for (const auto& [p_index, rotate_info] : points_pair) {  // std::pair<int, gazebo::AviaRotateInfo>
        const int verticle_index = roundf((rotate_info.zenith - verticle_min) / verticle_incre);
        const int horizon_index = roundf((rotate_info.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) continue;
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            const auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            const auto range = rayShape->GetRange(p_index);
            const auto intensity = rayShape->GetRetro(p_index);
            if (range >= RangeMax() || range <= RangeMin()) continue;

            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            const auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            const auto point = range * axis;

            pcl::PointXYZI pt;
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.intensity = static_cast<float>(intensity);
#pragma omp critical
            {
                pc[pt_count] = pt;
                ++pt_count;
            }
        }
    }
    if(rosPointPub.getNumSubscribers() > 0 && pt_count > 0) {
        pc.resize(pt_count);
        pcl::toROSMsg(pc, scan_point);
        scan_point.header.stamp = timestamp;
        scan_point.header.frame_id = "livox";
        rosPointPub.publish(scan_point);
    }
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZRTL(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    const auto rayCount = RayCount();
    const auto verticalRayCount = VerticalRayCount();
    const auto angle_min = AngleMin().Radian();
    const auto angle_incre = AngleResolution();
    const auto verticle_min = VerticalAngleMin().Radian();
    const auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;

    pcl::PointCloud<pcl::LivoxPointXyzrtl> pc;
    pc.points.reserve(points_pair.size());
    const auto timestamp = ros::Time::now();
    for (const auto& [p_index, rotate_info] : points_pair) {  // std::pair<int, gazebo::AviaRotateInfo>
        const int verticle_index = roundf((rotate_info.zenith - verticle_min) / verticle_incre);
        const int horizon_index = roundf((rotate_info.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            const auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            const auto range = rayShape->GetRange(p_index);
            const auto intensity = rayShape->GetRetro(p_index);
            if (range >= RangeMax() || range <= RangeMin() || abs(range) <= 1e-5) {
                continue;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

            const auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            const auto point = range * axis;
            pcl::LivoxPointXyzrtl pt;
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.reflectivity = static_cast<float>(intensity);
            pt.tag = 0;
            pt.line = rotate_info.line;
            pc.push_back(std::move(pt));
        }
    }
    pcl::toROSMsg(pc, scan_point);
    if(rosPointPub.getNumSubscribers() > 0 && !scan_point.data.empty()) {
        scan_point.header.stamp = timestamp;
        scan_point.header.frame_id = "livox";
        rosPointPub.publish(scan_point);
    }
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    const auto rayCount = RayCount();
    const auto verticalRayCount = VerticalRayCount();
    const auto angle_min = AngleMin().Radian();
    const auto angle_incre = AngleResolution();
    const auto verticle_min = VerticalAngleMin().Radian();
    const auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;
    livox_ros_driver::CustomMsg msg;
    msg.header.frame_id = "livox";

    struct timespec tn;
    clock_gettime(CLOCK_REALTIME, &tn);

    msg.timebase = tn.tv_nsec;
    msg.header.stamp = ros::Time::now();
    const auto timestamp = ros::Time::now();
    for (const auto& [p_index, rotate_info] : points_pair) {  // std::pair<int, gazebo::AviaRotateInfo>
        const int verticle_index = roundf((rotate_info.zenith - verticle_min) / verticle_incre);
        const int horizon_index = roundf((rotate_info.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            const auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            const auto range = rayShape->GetRange(p_index);
            const auto intensity = rayShape->GetRetro(p_index);
            if (range >= RangeMax() || range <= RangeMin() || abs(range) <= 1e-5) {
                continue;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

            const auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            const auto point = range * axis;
            livox_ros_driver::CustomPoint pt;
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.reflectivity = 100;
            pt.line = rotate_info.line;
            pt.tag = 0x10;
            msg.points.push_back(pt);
        }
    }
    clock_gettime(CLOCK_REALTIME, &tn);
    const uint64_t interval = tn.tv_nsec - msg.timebase;
    for (int i = 0; i < msg.points.size(); ++i) {
        msg.points[i].offset_time = (float)interval / msg.points.size() * i;
    }
    msg.point_num = msg.points.size();
    rosPointPub.publish(msg);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

}  // namespace gazebo

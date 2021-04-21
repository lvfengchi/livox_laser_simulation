# Livox Laser Simulation
A package to provide plug-in for [Livox Series LiDAR](https://www.livoxtech.com).

## Requirements
- ROS(=Kinectic)
- Gazebo (= 7.0, http://gazebosim.org/)
- Ubuntu(=16.04)

## Results
- avia

![](resources/avia.gif)
- mid40

![](resources/mid40.gif)
- mid70

![](resources/mid70.gif)
- tele

![](resources/tele.gif)
- horizon

![](resources/horizon.gif)

## Usage

> Note that the published point cloud message is sensor_msg/PointCloud in main branch. If you want to obtain snesor_msg/PointCloud2 message, you can checkout to "PointCloud2-ver" branch.
> If you use gazebo 9, checkout to "gazebo-9-ver" branch. The gazebo-9 version is maintained by [jp-ipu](https://github.com/jp-ipu).

Before you write your urdf file by using this plugin, catkin_make/catkin build is needed.

A simple demo is shown in livox_simulation.launch

Run 
```
    roslauch livox_laser_simulation livox_simulation.launch
```
to see.

We can choose the lidar model by selecting different CSV file in scan_mode dir from changing the launch file:
- avia.csv
- horizon.csv
- mid40.csv
- mid70.csv
- tele.csv

## Parameters(example by avia)

- laser_min_range: 0.1  // min detection range
- laser_max_range: 200.0  // max detection range
- horizontal_fov: 70.4   //°
- vertical_fov: 77.2    //°
- ros_topic: scan // topic in ros
- samples: 24000  // number of points in each scan loop
- downsample: 1 // we can increment this para to decrease the consumption

### This repository is now maintained by CaoMing(https://github.com/EpsAvlc) and LvFengchi. Its appreciate with the help of Caoming!

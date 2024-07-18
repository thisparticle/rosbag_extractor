# Bag Extractor

## Overview

This repository is a tailored adaptation of the [dataset_bag_extractor](https://github.com/Robotics-Mechatronics-UMA/dataset_bag_extractor) and [livox_to_pointcloud2](https://github.com/koide3/livox_to_pointcloud2) projects, specifically optimized for the GEODE dataset. It provides a comprehensive package designed to extract raw data from ROS (Robot Operating System) bags, facilitating the extraction of various data types such as images, IMU data, and point clouds from Velodyne, Ouster, and Livox sensors.

### Key Features

- **img2jpg**: Converts compressed image topics to JPG format, utilizing OpenCV and the [cv_bridge](https://wiki.ros.org/cv_bridge). Images are saved with a timestamp-based naming convention.
- **imu2txt**: Extracts IMU data into a text file, with each data point on a new line, including timestamp and IMU measurements.
- **pc2bin**: Saves PointCloud data from Velodyne or Ouster topics into binary files (`.bin`), using a [PointCloud iterator](https://wiki.ros.org/pcl/Overview). Each file is named with a timestamp.
- **livox2bin**: Similar to pc2bin, but specifically for Livox point clouds, extracting data from the `livox_ros_driver::CustomMsg` message type.

## Installation

### Dependencies

Before you begin, ensure you have the following dependencies installed:

- [Robot Operating System (ROS)](http://wiki.ros.org/)
- [OpenCV](https://opencv.org/)
- [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
- ROS dependencies: roscpp, std_msgs, sensor_msgs, rosbag, cv_bridge
- [fmt](https://github.com/fmtlib/fmt): For text formatting

### Building from Source

1. Clone the latest version of the repository into your catkin workspace:
   ```bash
   cd catkin_workspace/src
   git clone https://github.com/thisparticle/rosbag_extractor.git
   cd ../
   ```
2. Build the package using:
   ```bash
   catkin build
   ```

## Usage

To use the tool, first, ensure you have a launch file configured to meet your specific requirements in terms of structure, sensors, and topics. A minimal example is provided for testing, which can be executed as follows:

```bash
roslaunch bag_extractor <extract_dataset>.launch bag:=<bag_file>
```

## Launch Files

- `<extract_dataset>.launch`: Launches all sensor nodes for the specified ROS bag file. This file is provided as a template for extracting data from the GEODE dataset, but can be adapted for other use cases with minor modifications.

## Nodes

All nodes in this package share common parameters:

- `bag`: Name of the ROS bag file. Note that the path in the .launch file is relative to the `~/.ros` or `$ROS_HOME` directory.
- `topic`: Name of the topic from which to extract data. 
- `folder`: Name of the folder where the extracted data will be stored. This folder must exist before running the node. The .launch file includes a script to create directories if they do not exist.

Optional parameters:

- `start_time`: Start time for data extraction from the bag.
- `end_time`: End time for data extraction from the bag.

For LiDAR messages:

- `lidar_type`: Type of LiDAR (Livox, Ouster, Velodyne).

### Node Details

- **img2jpg**: Extracts images from compressed image topics and saves them as JPG files, using OpenCV and cv_bridge. Images are named with a timestamp.
- **imu2txt**: Extracts IMU data and saves it in a text file, with each message on a new line, formatted as follows:
  
  ```
  timestamp roll pitch yaw ang_vel_X ang_vel_Y ang_vel_Z lin_acc_X lin_acc_Y lin_acc_Z
  ```
- **pc2bin**: Extracts PointCloud data from Velodyne or Ouster topics and saves it in binary files. Each file is named with a timestamp and contains the following data:
  - Velodyne: `x y z intensity ring time`
  - Ouster: `x y z intensity t reflectivity ring ambient range`
- **livox2bin**: Extracts PointCloud data from a Livox Points topic and saves it in binary files, formatted as: `x y z intensity tag line`

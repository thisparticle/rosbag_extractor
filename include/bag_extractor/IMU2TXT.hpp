#pragma once

#include "bag_extractor/Utils.hpp"

// ROS
#include <ros/ros.h>

// Quaternion conversion
#include <tf/tf.h>

// Bag reading and processing
#include <rosbag/bag.h>
#include <rosbag/view.h>

// IMU specific includes
#include <sensor_msgs/Imu.h>

// fmt
#include <fmt/core.h>

// STD
#include <string>
#include <iostream>

namespace bag_extractor
{

/*!
*  Main class for the IMU bag extractor.
*/
class IMU2TXT
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    IMU2TXT(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    virtual ~IMU2TXT();

    /*!
     * Extract all data.
     */
    void extract();

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if succesful.
     */
    bool readParameters_();

    /*!
     * Set the time queries if the time limits have been set.
     * @param start the starting ros time for the query.
     * @param end the end ros time for the query.
     */
    void setTimeFilters_(ros::Time &start, ros::Time &end);

    /*!
     * Method to process the message obtained from the message instance.
     * @param msg the message to process.
     */
    void imuMsgProcess_(const sensor_msgs::ImuConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! Counter for the number of msgs (USED FOR SEQUENCING)
    //int counter_;

    //! Folder name for the saved IMU data
    std::string folder_;

    //! File to save the IMU data
    std::ofstream out_;

    //! IMU topic name
    std::string topic_;

    //! Bag name
    std::string bagName_;

    //! Rosbag
    rosbag::Bag bag_;

    double start_time_filter_;
    double end_time_filter_;

    //! Extension of the file to save the data.
    const std::string EXTENSION_ = ".txt";
};

} // namespace bag_extractor
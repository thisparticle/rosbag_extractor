#pragma once

#include "bag_extractor/Utils.hpp"

// ROS
#include <ros/ros.h>

// Quaternion conversion
#include <tf/tf.h>

// Bag reading and processing
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS Image specific includes
#include <sensor_msgs/CompressedImage.h>

// OpenCV
#include <opencv2/imgcodecs.hpp>

// CV bridge
#include <cv_bridge/cv_bridge.h>

// Image transport
#include <image_transport/image_transport.h>

// STD
#include <string>

namespace bag_extractor
{

/*!
*  Main class for the image bag extractor.
*/
class ComIMG2JPG
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    ComIMG2JPG(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    virtual ~ComIMG2JPG();

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
     * ROS topic callback method.
     * @param message the received message.
     */
    void imgMsgProcess_(const sensor_msgs::CompressedImageConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! Counter for the number of files (USED FOR SEQUENCING)
    //int counter_;

    //! Folder name for the saved images
    std::string folder_;

    //! File to save the image data
    std::ofstream out_;

    //! Image topic name
    std::string topic_;

    //! Bag name
    std::string bagName_;

    //! lidar type
    std::string lidar_type_;

    //! Rosbag
    rosbag::Bag bag_;

    //! Start time for the bag query.
    double start_time_filter_;

    //! End time for the bag query.
    double end_time_filter_;
    
    //! Part of the folder name used for the file name (Specific use case).
    std::string device_;

    //! Encoding of the compressed images
    // std::string encoding_ = "bgr8";

    //! Extension of the file to save the data.
    const std::string EXTENSION_ = ".jpg";
};

} // namespace bag_extractor
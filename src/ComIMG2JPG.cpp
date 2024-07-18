#include "bag_extractor/ComIMG2JPG.hpp"

namespace bag_extractor
{

    ComIMG2JPG::ComIMG2JPG(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        if (!readParameters_())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
    }

    ComIMG2JPG::~ComIMG2JPG()
    {
    }

    void ComIMG2JPG::extract()
    {

        // Init a counter (Not used right now)
        //counter_ = 0;

        ROS_INFO("Successfully launched node.");
        ROS_INFO("Topic: %s", topic_.c_str());

        // Get the device name for the file name.
        device_ = utils::get_device(folder_);

        // Open the bag using the bagname path.
        bag_.open(bagName_);

        // Declare the default time query to use if the time was not specified.

        /* A workaround to check if the time is valid for the 
        specified bag would be to create a preliminar ros::View without 
        the time param to extract the max and min times of the bag 
        and check that the selected ones are in between. */

        ros::Time ros_start_filter = ros::TIME_MIN;
        ros::Time ros_end_filter = ros::TIME_MAX;

        // If the time was specified, modify the time query.
        setTimeFilters_(ros_start_filter, ros_end_filter);

        // Create the topic queryto extract information about.
        rosbag::View view(bag_, rosbag::TopicQuery(topic_), ros_start_filter, ros_end_filter);

        // Initialize time data for progress indicator.
        uint64_t begin_time = view.getBeginTime().toNSec();
        uint64_t end_time = view.getEndTime().toNSec();
        uint64_t duration = end_time - begin_time;

        for (rosbag::MessageInstance const m : view)
        {
            // Get the message instance from the iterator.
            sensor_msgs::CompressedImageConstPtr msg = m.instantiate<sensor_msgs::CompressedImage>();

            // Compute and show the progress.
            uint64_t m_time = m.getTime().toNSec();
            float progress = (float)(m_time - begin_time) / (float)duration * 100;
            ROS_INFO("Processing image message ( %.2f%% )", progress);

            // Process the imu msg and write to file.
            imgMsgProcess_(msg);

            // Break the loop in case of shutdown.
            if (!ros::ok())
            {
                ROS_INFO("ROS shutdown!");
                bag_.close();
                return;
            }
        }

        bag_.close();
        ROS_INFO("Finished!");
    }

    void ComIMG2JPG::imgMsgProcess_(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        
        double timestamp = msg->header.stamp.toSec();

        std::string filename = utils::get_file_name(folder_, device_, timestamp, EXTENSION_);

        ROS_INFO("Saving image into: %s", filename.c_str());

        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg);

        cv::imwrite(filename, img->image);
    }

    void ComIMG2JPG::setTimeFilters_(ros::Time &start, ros::Time &end)
    {

        // Set the time filters if they were set.
        if (start_time_filter_ >= 0)
        {
            ROS_INFO("Start time set. Filter set at: %f", start_time_filter_);
            start = ros::Time(start_time_filter_);
        }

        if (end_time_filter_ >= 0)
        {
            ROS_INFO("End time set. Filter set at: %f", end_time_filter_);
            end = ros::Time(end_time_filter_);
        }

        ROS_INFO("Start filtering at: %f", start.toSec());

        ROS_INFO("End filtering at: %f", end.toSec());
    }

    bool ComIMG2JPG::readParameters_()
    {
        if (!nodeHandle_.getParam("folder", folder_))
            return false;
        if (!nodeHandle_.getParam("topic", topic_))
            return false;
        if (!nodeHandle_.getParam("bag", bagName_))
            return false;

        // Init the time filters if specified or -1 otherwise.
        if (!nodeHandle_.getParam("start_time", start_time_filter_))
            start_time_filter_ = -1;
        if (!nodeHandle_.getParam("end_time", end_time_filter_))
            end_time_filter_ = -1;

        return true;
    }

} // namespace bag_extractor
#include "bag_extractor/livox2BIN.hpp"

namespace bag_extractor
{

    livox2BIN::livox2BIN(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        if (!readParameters_())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        ROS_INFO("Successfully launched node.");
    }

    livox2BIN::~livox2BIN()
    {
    }

    void livox2BIN::extract()
    {
        // Init a counter for the file names.
        //counter_ = 0;

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
            livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            sensor_msgs::PointCloud2ConstPtr msg = converter.convert(*livox_msg);
            // sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

            // Compute and show the progress.
            uint64_t m_time = m.getTime().toNSec();
            float progress = (float)(m_time - begin_time) / (float)duration * 100;
            ROS_INFO("Processing PC message ( %.2f%% )", progress);

            // Process the imu msg and write to file.
            pcMsgProcess_(msg);

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

    void livox2BIN::pcMsgProcess_(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        double timestamp = msg->header.stamp.toSec();

        std::string filename = utils::get_file_name(folder_, device_, timestamp, EXTENSION_);

        ROS_INFO("Saving PointCloud into: %s", filename.c_str());

        std::ofstream out;

        out.open(filename.c_str(), std::ios::binary);

        // Check if the file was opened correctly.
        if (!out_)
        {
            ROS_ERROR("Problem opening the file: %s!!!", filename.c_str());
            ros::requestShutdown();
        }

        // Create iterator for the data
        // for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)
        // {
        //     out.write(reinterpret_cast<const char *>(&it[0]), sizeof(float) * 3);
        //     out.write(reinterpret_cast<const char *>(&it[4]), sizeof(float));
        // }
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        if (lidar_type_ == "Velodyne") {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
            sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring, ++iter_time) {
                if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                    out.write(reinterpret_cast<const char*>(&(*iter_x)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_y)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_z)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_intensity)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_ring)), sizeof(uint16_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_time)), sizeof(float));
                }
            }
        } else if (lidar_type_ == "Ouster") {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");  
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_t(*msg, "t");
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_reflectivity(*msg, "reflectivity");
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ambient(*msg, "ambient");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_range(*msg, "range");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_t, ++iter_reflectivity, ++iter_ring, ++iter_ambient, ++iter_range) {
                if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                    out.write(reinterpret_cast<const char*>(&(*iter_x)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_y)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_z)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_intensity)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_t)), sizeof(uint32_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_reflectivity)), sizeof(uint16_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_ring)), sizeof(uint16_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_ambient)), sizeof(uint16_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_range)), sizeof(uint32_t));
                }
            }
        } else if (lidar_type_ == "Livox") {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");  
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_tag(*msg, "tag");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(*msg, "line");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag, ++iter_line) {
                if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                    out.write(reinterpret_cast<const char*>(&(*iter_x)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_y)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_z)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_intensity)), sizeof(float));
                    out.write(reinterpret_cast<const char*>(&(*iter_tag)), sizeof(uint8_t));
                    out.write(reinterpret_cast<const char*>(&(*iter_line)), sizeof(uint8_t));
                }
            }
        } else {
            ROS_ERROR("Unsupported LiDAR type: %s", lidar_type_.c_str());
        }
        out.close();
    }

    void livox2BIN::setTimeFilters_(ros::Time &start, ros::Time &end)
    {

        // Set the time filters if they were set.
        if (start_time_filter_ >= 0)
        {
            start = ros::Time(start_time_filter_);
        }

        if (end_time_filter_ >= 0)
        {
            end = ros::Time(end_time_filter_);
        }

        ROS_INFO("Start filtering at: %f", start.toSec());

        ROS_INFO("End filtering at: %f", end.toSec());
    }

    bool livox2BIN::readParameters_()
    {
        if (!nodeHandle_.getParam("folder", folder_))
            return false;
        if (!nodeHandle_.getParam("topic", topic_))
            return false;
        if (!nodeHandle_.getParam("lidar_type", lidar_type_))
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
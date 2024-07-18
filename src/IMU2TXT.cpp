#include "bag_extractor/IMU2TXT.hpp"

namespace bag_extractor
{

    IMU2TXT::IMU2TXT(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        if (!readParameters_())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        ROS_INFO("Successfully launched node.");
    }

    IMU2TXT::~IMU2TXT()
    {
    }

    void IMU2TXT::extract()
    {
        // Create the file in the specified path.
        // std::string filename = "./" + folder_ + "imu.txt";
        std::string filename = folder_ + "imu.txt";

        // Open the file to store the data.
        out_.open(filename.c_str(), std::ios::out);

        // Check if the file was opened correctly.
        if (!out_)
        {
            ROS_ERROR("Problem opening the file: %s!!!", filename.c_str());
            ros::requestShutdown();
        }

        // Init a counter (Not used right now)
        //counter_ = 0;

        
        ROS_INFO("Topic: %s", topic_.c_str());
        ROS_INFO("Saving IMU data into: %s", filename.c_str());

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
            sensor_msgs::ImuConstPtr msg = m.instantiate<sensor_msgs::Imu>();

            // Compute and show the progress.
            uint64_t m_time = m.getTime().toNSec();
            float progress = (float)(m_time - begin_time) / (float)duration * 100;
            ROS_INFO("Processing IMU message ( %.2f%% )", progress);

            // Process the imu msg and write to file.
            imuMsgProcess_(msg);

            // Break the loop in case of shutdown.
            if (!ros::ok())
            {
                ROS_INFO("ROS shutdown!");
                bag_.close();
                out_.close();
                return;
            }
        }

        bag_.close();
        out_.close();
        ROS_INFO("Finished!");
    }

    void IMU2TXT::imuMsgProcess_(const sensor_msgs::ImuConstPtr &msg)
    {
        // Creates a quaternion with the message information.
        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // Generate a rotation matrix 3x3 from the quaternion.
        tf::Matrix3x3 m(q);

        double roll, pitch, yaw;

        m.getRPY(roll, pitch, yaw);

       // Create the container for the string.
        std::ostringstream str;

        str << fmt::format("{:0<10.9f}", msg->header.stamp.toSec()) << " ";

        str << roll << " ";
        str << pitch << " ";
        str << yaw << " ";

        str << msg->angular_velocity.x << " ";
        str << msg->angular_velocity.y << " ";
        str << msg->angular_velocity.z << " ";

        str << msg->linear_acceleration.x << " ";
        str << msg->linear_acceleration.y << " ";
        str << msg->linear_acceleration.z << "\n";

        std::string r = str.str();

        //ROS_INFO("Printing IMU sentence: \n%s", r.c_str());
        out_.write(r.c_str(), r.length());

        //counter_++;
    }

    void IMU2TXT::setTimeFilters_(ros::Time &start, ros::Time &end)
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

    bool IMU2TXT::readParameters_()
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
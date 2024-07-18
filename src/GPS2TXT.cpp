#include "bag_extractor/GPS2TXT.hpp"

namespace bag_extractor
{

    GPS2TXT::GPS2TXT(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        if (!readParameters_())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        ROS_INFO("Successfully launched node.");
    }

    GPS2TXT::~GPS2TXT()
    {
    }

    void GPS2TXT::extract()
    {
        // Create the file in the specified path.
        std::string filename = "./" + folder_ + "data.txt";

        // Open the file to store the data.
        out_.open(filename.c_str(), std::ios::out);

        // Check if the file was opened correctly.
        if (!out_)
        {
            ROS_ERROR("Problem opening the file: %s!!!", filename.c_str());
            ros::requestShutdown();
        }

        // Init a counter (USED FOR SEQUENCING)
        //counter_ = 0;

        ROS_INFO("Saving GPS data into: %s", filename.c_str());

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

        // This case covers several possible topics.
        // Therefore a coma separated list is created from them.
        std::vector<std::string> topics = utils::split_strings(topic_, ',');

        ROS_INFO("Topics: ");
        for (std::string const topic : topics)
        {
            ROS_INFO("%s", topic.c_str());
        }

        // Create the topic query to extract information.
        rosbag::View view(bag_, rosbag::TopicQuery(topics), ros_start_filter, ros_end_filter);

        // Initialize time data for progress indicator.
        uint64_t begin_time = view.getBeginTime().toNSec();
        uint64_t end_time = view.getEndTime().toNSec();
        uint64_t duration = end_time - begin_time;

        for (rosbag::MessageInstance const m : view)
        {
            // Try to get a GPGGA message instance from the iterator.
            novatel_gps_msgs::GpggaConstPtr gpggaMsg = m.instantiate<novatel_gps_msgs::Gpgga>();

            if (gpggaMsg != NULL)
            {
                // Compute and show the progress.
                uint64_t m_time = m.getTime().toNSec();
                float progress = (float)(m_time - begin_time) / (float)duration * 100;
                ROS_INFO("Processing GPGGA message ( %.2f%% )", progress);

                // Process the gpgga msg and write to file.
                gpggaMsgProcess_(gpggaMsg);
            }

            // Try to get a GPRMC message instance from the iterator.
            novatel_gps_msgs::GprmcConstPtr gprmcMsg = m.instantiate<novatel_gps_msgs::Gprmc>();

            if (gprmcMsg != NULL)
            {
                // Compute and show the progress.
                uint64_t m_time = m.getTime().toNSec();
                float progress = (float)(m_time - begin_time) / (float)duration * 100;
                ROS_INFO("Processing GPRMC message ( %.2f%% )", progress);

                // Process the gprmc msg and write to file.
                gprmcMsgProcess_(gprmcMsg);
            }

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

    void GPS2TXT::setTimeFilters_(ros::Time &start, ros::Time &end)
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

    void GPS2TXT::gpggaMsgProcess_(const novatel_gps_msgs::GpggaConstPtr &msg)
    {
        // Create the container for the string.
        std::ostringstream str;

        // Insert the timestamp with all the available decimals.
        str << fmt::format("{:0<10.9f}", msg->header.stamp.toSec()) << " $GPGGA,";

        // Get the utc seconds and convert it to NMEA utc time format: HHMMSS.SSS
        float time = msg->utc_seconds;
        int hour = time / 3600;
        time = fmod(time, 3600);
        int min = time / 60;
        float sec = fmod(time, 60);

        str << fmt::format("{:0>2}{:0>2}{:0^2.3f}", hour, min, sec) << ",";

        // Get latitude and convert it from decimal degrees to degrees and minutes.
        int deg = msg->lat;
        float deg_min = (msg->lat - deg) * 60;
        str << fmt::format("{}{:2.10f}", deg, deg_min) << ",";
        str << msg->lat_dir << ",";

        // Get longitude and convert it from decimal degrees to degrees and minutes.
        deg = msg->lon;
        deg_min = (msg->lon - deg) * 60;
        str << fmt::format("{}{:2.10f}", deg, deg_min) << ",";
        str << msg->lon_dir << ",";

        str << msg->gps_qual << ",";

        // Number of sats must be in NN format.
        str << fmt::format("{:0>2}", msg->num_sats) << ",";

        str << msg->hdop << ",";
        str << msg->alt << ",";
        str << msg->altitude_units << ",";
        str << msg->undulation << ",";
        str << msg->undulation_units << ",";

        // Add only if differential correction is available.
        if (msg->station_id != "")
        {
            str << msg->diff_age << ",";
        }
        else
        {
            str << ",";
        }
        // Empty if there is no differential correction.
        str << msg->station_id << "\n";

        std::string r = str.str();

        //ROS_INFO("Printing GPGGA sentence: \n%s", r.c_str());
        out_.write(r.c_str(), r.length());

        //counter_++;
    }

    void GPS2TXT::gprmcMsgProcess_(const novatel_gps_msgs::GprmcConstPtr &msg)
    {
        // Create the container for the string.
        std::ostringstream str;

        // Insert the timestamp with all the available decimals.
        str << fmt::format("{:0<10.9f}", msg->header.stamp.toSec()) << " $GPRMC,";

        // Get the utc seconds and convert it to NMEA utc time format: HHMMSS.SSS
        float time = msg->utc_seconds;
        int hour = time / 3600;
        time = fmod(time, 3600);
        int min = time / 60;
        float sec = fmod(time, 60);

        str << fmt::format("{:0>2}{:0>2}{:0^2.3f}", hour, min, sec) << ",";
        str << msg->position_status << ",";

        // Get latitude and convert it from decimal degrees to degrees and minutes.
        int deg = msg->lat;
        float deg_min = (msg->lat - deg) * 60;
        str << fmt::format("{}{:2.10f}", deg, deg_min) << ",";
        str << msg->lat_dir << ",";

        // Get longitude and convert it from decimal degrees to degrees and minutes.
        deg = msg->lon;
        deg_min = (msg->lon - deg) * 60;
        str << fmt::format("{}{:2.10f}", deg, deg_min) << ",";
        str << msg->lon_dir << ",";

        str << msg->speed << ",";
        str << msg->track << ",";

        // Convert date from the message to NMEA date format: DDMMYYYY.
        std::string date = msg->date;
        std::string delimiter = "-";

        size_t delimiter_len = delimiter.length();
        size_t year_pos = msg->date.find(delimiter, 0);
        size_t month_pos = msg->date.find(delimiter, year_pos + delimiter_len);
        std::string year = date.substr(0, year_pos);
        std::string month = msg->date.substr(year_pos + delimiter_len, month_pos - (year_pos + delimiter_len));
        std::string day = msg->date.substr(month_pos + delimiter_len, date.length() - (month_pos + delimiter_len));
        str << day << month << year << ",";

        str << msg->mag_var << ",";
        str << msg->mag_var_direction << ",";
        str << msg->mode_indicator << "\n";

        std::string r = str.str();

        //ROS_INFO("Printing GPRMC sentence: \n%s", r.c_str());
        out_.write(r.c_str(), r.length());

        //counter_++;
    }

    bool GPS2TXT::readParameters_()
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
#include <ros/ros.h>

#include <bag_extractor/IMU2TXT.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "imu2bin");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::IMU2TXT imu2txt(nodeHandle);
  imu2txt.extract();

  ros::spin ();

}

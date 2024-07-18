#include <ros/ros.h>

#include <bag_extractor/GPS2TXT.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "gps2bin");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::GPS2TXT gps2txt(nodeHandle);
  gps2txt.extract();

  ros::spin ();

}

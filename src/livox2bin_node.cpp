#include <ros/ros.h>

#include <bag_extractor/livox2BIN.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "livox2bin");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::livox2BIN livox2bin(nodeHandle);
  livox2bin.extract();
  
  ros::spin ();

}

#include <ros/ros.h>

#include <bag_extractor/PC2BIN.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc2bin");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::PC2BIN pc2bin(nodeHandle);
  pc2bin.extract();
  
  ros::spin ();

}

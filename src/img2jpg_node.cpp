#include <ros/ros.h>

#include <bag_extractor/IMG2JPG.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "img2jpg");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::IMG2JPG img2jpg(nodeHandle);
  img2jpg.extract();
  
  ros::spin ();

}

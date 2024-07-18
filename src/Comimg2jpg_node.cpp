#include <ros/ros.h>

#include <bag_extractor/ComIMG2JPG.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Comimg2jpg");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::ComIMG2JPG img2jpg(nodeHandle);
  img2jpg.extract();
  
  ros::spin ();

}

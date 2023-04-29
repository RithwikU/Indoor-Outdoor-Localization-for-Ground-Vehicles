#include "ros/ros.h"
#include "std_msgs/String.h"
#include <odometry_example.cpp>

#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  std::cout << "We are going to run Odom example!"<< std::endl;

  run_odom();


  return 0;
}
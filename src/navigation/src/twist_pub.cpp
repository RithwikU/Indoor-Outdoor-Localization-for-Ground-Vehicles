#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_pub");

  ros::NodeHandle n;

  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = 0.1;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    ROS_INFO("FRONT : %f", msg.linear.x);

    twist_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
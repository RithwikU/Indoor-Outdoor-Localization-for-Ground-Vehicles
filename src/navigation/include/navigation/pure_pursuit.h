#include <string>
#include <cmath>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"



struct Waypoint
{
    Waypoint()
        : Waypoint(0.0, 0.0) {}
    Waypoint(double x, double y)
        : x(x), y(y) {}
    double x;
    double y;
};


ros::Publisher twist_pub_;
ros::Publisher vis_waypoint_pub_;
ros::Subscriber pose_sub_;
tf::Transform transform_ego_T_map_;

double l_ = 2.0;  // Look ahead
double steering_gain_ = 1.2;
double speed_ = 0.69;
double marker_size = 1.0;

int idx_current_ = 0;

visualization_msgs::MarkerArray marker_array_;

std::vector<Waypoint> waypoints_;

Waypoint find_tracking_point(geometry_msgs::Pose::ConstPtr& pose);
Waypoint transformToEgoFrame(Waypoint wpt);

void csv_to_waypoints();
// void timer_callback(const ros::TimerEvent& event);

void pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

double calculate_steering(Waypoint wp);
void update_tracking_waypoint();




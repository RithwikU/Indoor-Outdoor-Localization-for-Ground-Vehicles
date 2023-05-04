#include <string>
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


struct Waypoint
{
    Waypoint()
        : Waypoint(0.0, 0.0) {}
    Waypoint(double x, double y)
        : x(x), y(y) {}
    Waypoint(std::vector<double> v)
        : x(v.at(0)), y(v.at(1)) {}
    double x;
    double y;
    bool has_visited;
};

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();
    ~PurePursuit() {}


    ros::NodeHandle n_;
    ros::Publisher twist_pub_;
    ros::Subscriber pose_sub_;
    tf::Transform transform_ego_T_map_

    double l_;  // Look ahead
    double steering_gain_;
    double speed_ = 0.5;

    int idx_current_;

    std::vector<Waypoint> waypoints_;

    Waypoint find_tracking_point(geometry_msgs::Pose::ConstPtr& pose);
    Waypoint transformToEgoFrame(Waypoint &wpt);

    void calculate_steering(Waypoint wp);
    void update_tracking_waypoint();
};



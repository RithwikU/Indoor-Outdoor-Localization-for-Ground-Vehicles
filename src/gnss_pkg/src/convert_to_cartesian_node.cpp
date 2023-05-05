#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <math.h>
#include <cmath>

#define radians(val) (val*M_PI/180.0)

ros::Publisher pose_pub;
double base_point_gps_[2] = {39.9521881666698, -75.1912719920683};

std::vector<double> get_earth_radius_at_latitude(double latitude)
{
    // constants for from GPS Hector plugin
    double EQUATORIAL_RADIUS = 6378137.0;
    double FLATTENING = 1.0 / 298.257223563;
    double ECCENTRICITY2 = 2.0 * FLATTENING - pow(FLATTENING, 2.0);

    // calculate earth radius from GPS Hector plugin
    double base_point_latitude_ = radians(latitude);
    double temp_ = 1.0 / (
        1.0 - ECCENTRICITY2 * pow(sin(base_point_latitude_), 2.0)
    );
    double prime_vertical_radius_ = EQUATORIAL_RADIUS * sqrt(temp_);
    double radius_north = prime_vertical_radius_ * (1.0 - ECCENTRICITY2) * temp_;
    double radius_east = prime_vertical_radius_ * cos(base_point_latitude_);

    std::vector<double> radii{radius_north, radius_east};
    return radii;
}


std::vector<double> gnss_to_cartesian(double point_lat, double point_lon)
{
    std::vector<double> radii = get_earth_radius_at_latitude(point_lat);
    double radius_north = radii[0];
    double radius_east = radii[1];

    double point_centered[2] = {point_lat - base_point_gps_[0], point_lon - base_point_gps_[1]};

    double x = radians(point_centered[0]) * radius_north;
    double y = radians(point_centered[1]) * radius_east;
    
    std::vector <double> cartesian_xy{y, x};
    return cartesian_xy;
}


void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // ROS_INFO("Lat: [%s]", msg->latitude.c_str());
    // ROS_INFO("Lon: [%s]", msg->longitude.c_str());
    if(msg->latitude != 0.0 && msg->longitude != 0.0)
    {
        std::vector<double> cartesian_xy = gnss_to_cartesian(msg->latitude, msg->longitude);
        geometry_msgs::PoseStamped cartesian_pose;
        cartesian_pose.header.stamp = ros::Time::now();
        cartesian_pose.pose.position.x = cartesian_xy[0];
        cartesian_pose.pose.position.y = cartesian_xy[1];
        cartesian_pose.pose.position.z = msg->altitude;
        cartesian_pose.pose.orientation.x = 0;
        cartesian_pose.pose.orientation.y = 0;
        cartesian_pose.pose.orientation.z = 0;
        cartesian_pose.pose.orientation.w = 1;
        ROS_INFO("x: %f", cartesian_pose.pose.position.x);
        ROS_INFO("y: %f", cartesian_pose.pose.position.y);
        pose_pub.publish(cartesian_pose);
    }
    else
    {
        ROS_INFO("No GPS data");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_to_cartesian_node");
    ros::NodeHandle n;

    ros::Subscriber gnss_sub = n.subscribe("fix", 1000, gnss_callback);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("cartesian_pose", 1000);

    ros::spin();

    return 0;
}

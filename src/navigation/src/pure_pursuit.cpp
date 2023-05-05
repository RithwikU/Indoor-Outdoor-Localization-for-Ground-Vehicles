#include "navigation/pure_pursuit.h"

void csv_to_waypoints()
{
    std::string relative_path = "/home/indro/ESE650/src/navigation/waypoints/waypoints_direct.csv";
    // std::string fname = "waypoints.csv";
    
    std::string line, s;
    std::ifstream file(relative_path);
    std::cout << "File open: " << file.is_open() << std::endl;
    // file.open("waypoints_drive.csv");

    visualization_msgs::Marker marker;

    std::vector<double> line_vector;
    
    if(!file.is_open())
    {
        throw "waypoints/waypoints*.csv file failed to open, check relative path";
    }
    else
    {
        int marker_id = 1;
        while(getline(file, line))
        {
            Waypoint p;

            std::stringstream ss(line);

            while(getline(ss, s, ',')) 
            {
                // store token string in the vector
                line_vector.push_back(stod(s));
            }

            p.x = line_vector[0]; // str to double
            p.y = line_vector[1]; // str to double

            waypoints_.push_back(p);        

            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position.x = p.x;
            marker.pose.position.y = p.y;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            marker.id = marker_id++;
            marker.scale.x = marker_size;
            marker.scale.y = marker_size;
            marker.scale.z = marker_size;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.header.frame_id = "map";

            marker_array_.markers.push_back(marker);

            line_vector.clear();
        }
        file.close();
    }
        vis_waypoint_pub_.publish(marker_array_);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){


    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, orientation);
    tf::Vector3 translation(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0.0);
    transform_ego_T_map_ = tf::Transform(orientation, translation).inverse();


    // Identify closest waypoint,
    update_tracking_waypoint();
    Waypoint tracking_point = waypoints_.at(idx_current_);

    // Find Projection
    double steering = calculate_steering(tracking_point);

    // Publish twist message
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = speed_;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = steering;

    twist_pub_.publish(twist_msg); 

}


// Waypoint find_tracking_point(geometry_msgs::Pose::ConstPtr& pose)
// {
//     // Find tracking point
//     for (int i = 0; i < waypoints_.size(); i++)
//     {
//         double distance = sqrt(pow(pose->position.x - waypoints_.at(i).x, 2) + pow(pose->position.y - waypoints_.at(i).y, 2));
//         double min_dist = __DBL_MAX__;
//         Waypoint tracking_point;
//         if (distance < min_dist && distance > l_ && !waypoints_.at(i).has_visited)
//         {
//             min_dist = distance;
//             tracking_point = waypoints_.at(i);
//         }
        
//     }

//     return tracking_point;
// }


void update_tracking_waypoint(){
    Waypoint curr = waypoints_.at(idx_current_);
    // std::cout << "waypoint x: " <<  curr.x << std::endl;
    // std::cout << "waypoint y: " <<  curr.y << std::endl;
    curr = transformToEgoFrame(curr);
    // std::cout << "waypoint x: " <<  curr.x << std::endl;
    // std::cout << "waypoint y: " <<  curr.y << std::endl;
    std::cout << "Current index: " << idx_current_ << std::endl;

    if(curr.x*curr.x + curr.y*curr.y < l_*l_){
        std::cout << "Reached waypoint number : " << idx_current_ << std::endl;
        idx_current_++;
        if(idx_current_ == waypoints_.size()){
            std::cout << "Reached final waypoint" << std::endl;
            speed_ = 0.0;
        }
    }
}


Waypoint transformToEgoFrame(Waypoint wpt)
{
    tf::Vector3 pt(wpt.x, wpt.y, 0.0);

    try
    {
        pt = transform_ego_T_map_ * pt;
        wpt.x = pt.getX();
        wpt.y = pt.getY();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return wpt;
}


double calculate_steering(Waypoint wp){
 
    // Transform these points to the ego frame
    Waypoint tracking = transformToEgoFrame(wp);

    float dist = sqrt(pow(tracking.x, 2) + pow(tracking.y, 2));
    tracking.x = l_ * tracking.x / dist;
    tracking.y = l_ * tracking.y / dist;

    double curvature = (2*abs(tracking.y))/pow(l_, 2);
    double steering_angle = steering_gain_ * curvature;

    // std::cout << "Steering angle: " << steering_angle << std::endl;
    
    if(tracking.y < 0)
        steering_angle = -steering_angle;

    return steering_angle;  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle n_;


    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vis_waypoint_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/waypoints", 10);

    // Initialize ROS subscriber to recieve odometry message
    pose_sub_ = n_.subscribe("/slam_odom", 10, pose_callback);

    csv_to_waypoints();

    // create a timer that calls the 'timerCallback' function every second
    ros::Timer timer = n_.createTimer(ros::Duration(5.0), [&](const ros::TimerEvent& event) {
    // publish the message
        // std::cout<<"Displaying waypointd " << std::endl;
        vis_waypoint_pub_.publish(marker_array_);
    });

    // ros::Time time = ros::Time::now();
    std::cout << "Starting node" << std::endl;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#include "navigation/pure_pursuit.h"

PurePursuit::PurePursuit()
{ 
    // Initialize ROS publisher
    twist_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Initialize ROS subscriber to recieve odometry message
    pose_sub_ = n.subscribe("/odom", 10, &PurePursuit::pose_callback, this);
    

    // Initialize waypoints
    // TODO :: load waypoints  

    current_tracking_point_ = waypoints_.at(0);
}



void PurePursuit::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // update car transform

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(pose_msg->pose.orientation, orientation);
    tf::Vector3 translation(pose_msg->pose.position.x, pose_msg->pose.position.y, 0.0);
    tf::Transform transform_map_to_ego = tf::Transform(orientation, translation).inverse();


    // Identify closest waypoint,
    update_tracking_waypoint();
    Waypoint tracking_point = waypoints_.at(idx_current_);



    // Find Projection
    double steering = calculate_steering(tracking_point);
    std::cout << "Steering angle: " << steering << std::endl;

    // Publish twist message
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = speed_;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = steering;
    

}


Waypoint PurePursuit::find_tracking_point(geometry_msgs::Pose::ConstPtr& pose)
{
    // Find tracking point
    for (int i = 0; i < waypoints_.size(); i++)
    {
        double distance = sqrt(pow(pose->position.x - waypoints_.at(i).x, 2) + pow(pose->position.y - waypoints_.at(i).y, 2));
        double min_dist = __DBL_MAX__;
        Waypoint tracking_point;
        if (distance < min_dist && distance > l_ && !waypoints_.at(i).has_visited)
        {
            min_dist = distance;
            tracking_point = waypoints_.at(i);
        }
        
    }

    return tracking_point;
}


void PurePursuit::update_tracking_waypoint(){
    Waypoint curr = waypoints_.at(idx_current_);
    curr = transformToEgoFrame(curr);
    if(curr.x*curr.x + curr.y*curr.y < l_*l_){
        std::cout << "Reached waypoint number : " << idx_current_ << std::endl;
        idx_current_++;
        if(idx_current_ == waypoints_.size()){
            std::cout << "Reached final waypoint" << std::endl;
            velocity_ = 0;
        }
    }
}


Waypoint PurePursuit::transformToEgoFrame(Waypoint wpt)
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


void PurePursuit::calculate_steering(Waypoint wp){
 
    // Transform these points to the ego frame
    Waypoint tracking = transformToEgoFrame(wp);

    float dist = sqrt(pow(tracking.x, 2) + pow(tracking.y, 2));
    tracking.x = this->l_ * tracking.x / dist;
    tracking.y = this->l_ * tracking.y / dist;

    double curvature = (2*abs(tracking.y))/pow(l_, 2);
    double steering_angle = steering_gain_ * curvature;
    
    if(tracking_waypoint_.y < 0)
        steering_angle = -steering_angle;

    return steering_angle;  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    ros::Rate loop_rate(10);

    PurePursuit pure_pursuit;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
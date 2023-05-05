#include <navigation/our_localization_example.h>


using namespace std;
using namespace gtsam;


// Factorgraph stuff 
NonlinearFactorGraph graph;
auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0));
int node_count = 1;
pose old_odom_pose;
pose new_odom_pose;
Values initialEstimate;
bool isLidarWorking;
auto gpsNoise = noiseModel::Diagonal::Sigmas(Vector2(1, 1));  // 1m std on x,y
auto goodLidarNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 1m std on x,y
auto badLidarNoise = noiseModel::Diagonal::Sigmas(Vector2(10, 10));  // 1m std on x,y


void add_odom_factor(){
    node_count++;
    pose diff;
    diff.x = new_odom_pose.x - old_odom_pose.x;
    diff.y = new_odom_pose.y - old_odom_pose.y;
    graph.emplace_shared<BetweenFactor<Pose2> >(node_count -1, node_count, Pose2(diff.x, diff.y, 0.0), odometryNoise);
    initialEstimate.insert(node_count, Pose2(new_odom_pose.x, new_odom_pose.y, 0));
}

void add_gps_factor(pose gps){
    // Add odometry factor
    graph.emplace_shared<UnaryFactor>(node_count, gps.x, gps.y, gpsNoise);
}


void add_lidar_factor(pose lidar){
    // Add odometry factor
    if(isLidarWorking){
        graph.emplace_shared<UnaryFactor>(node_count, lidar.x, lidar.y, goodLidarNoise);
    }
    else{
        graph.emplace_shared<UnaryFactor>(node_count, lidar.x, lidar.y, badLidarNoise);
    }
}

void run_inference(){
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");
}


void run_our_example(){
    
    std::string odom_file = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_odom_drift.csv";
    std::string lidar_file = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_lidar_inside.csv";
    std::string gps_file = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_gps.csv";


    std::vector<pose> gps_data = csv_to_poses(gps_file);
    std::vector<pose> odom_data = csv_to_poses(odom_file);
    std::vector<pose> lidar_data = csv_to_poses(lidar_file);

    // Initial estimate is the first odom value
    pose x0;
    x0.x = odom_data[0].x;
    x0.y = odom_data[0].y;
    isLidarWorking = true;
    node_count = 1;

    std::cout << "Loaded files!" << std::endl;
    initialEstimate.insert(node_count, Pose2(x0.x, x0.y, 0));
    // add_odom_factor();

    for (int i = 2; i < gps_data.size(); i++){
        
        
        new_odom_pose = odom_data[i];
        old_odom_pose = odom_data[i-1]; //TODO :: needs work
        add_odom_factor();
        if(i%2 == 0){
            add_gps_factor(gps_data[i]);
        }
        if(i%1 == 0){
            add_lidar_factor(lidar_data[i]);
        }
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        Values result = optimizer.optimize();
        initialEstimate = result;

    }

    std::vector<pose> final_traj;
    for (int i = 1; i < node_count; i++){
        pose ps;
        ps.x = initialEstimate.at<Pose2>(i).x();
        ps.y = initialEstimate.at<Pose2>(i).y();
        final_traj.push_back(ps);
    }

    write_poses_to_csv(final_traj, "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/gtsam_results/result1.csv");
    initialEstimate.print("Final Result:\n");
    std::cout << "The value in the last node is " << initialEstimate.at<Pose2>(node_count) << std::endl;
    
}







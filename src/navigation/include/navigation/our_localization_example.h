#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <string>
#include <cmath>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>


using namespace std;
using namespace gtsam;

struct pose{

    double x;
    double y;
    double theta;
};

class UnaryFactor: public NoiseModelFactor1<Pose2> {

  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  double mx_, my_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  ~UnaryFactor() override {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const override {
    // The measurement function for a GPS-like measurement h(q) which predicts the measurement (m) is h(q) = q, q = [qx qy qtheta]
    // The error is then simply calculated as E(q) = h(q) - m:
    // error_x = q.x - mx
    // error_y = q.y - my
    // Node's orientation reflects in the Jacobian, in tangent space this is equal to the right-hand rule rotation matrix
    // H =  [ cos(q.theta)  -sin(q.theta) 0 ]
    //      [ sin(q.theta)   cos(q.theta) 0 ]
    const Rot2& R = q.rotation();
    if (H) (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
};  // 



std::vector<pose> csv_to_poses(std::string relative_path)
{
    // std::string relative_path = "/home/indro/ESE650/src/navigation/waypoints/waypoints_direct.csv";
    // std::string fname = "waypoints.csv";
    std::string line, s;
    std::ifstream file(relative_path);
    std::cout << "File open: " << file.is_open() << std::endl;
    // file.open("waypoints_drive.csv");
    std::vector<double> line_vector;
    std::vector<pose> poses;
    if(!file.is_open())
    {
        throw "waypoints/waypoints*.csv file failed to open, check relative path";
    }
    else
    {
        while(getline(file, line))
        {
            pose p;

            std::stringstream ss(line);

            while(getline(ss, s, ',')) 
            {
                // store token string in the vector
                line_vector.push_back(stod(s));
            }
            p.x = line_vector[0]; // str to double
            p.y = line_vector[1]; // str to double

            poses.push_back(p);        


            line_vector.clear();
        }
        file.close();
    }
    return poses;
}


// Write a function to write a vector of poses to a csv file
void write_poses_to_csv(std::vector<pose> poses, std::string filename){
    std::ofstream myfile;
    myfile.open (filename);
    for (int i = 0; i < poses.size(); i++){
        myfile << poses[i].x << "," << poses[i].y << std::endl;
    }
    myfile.close();
}
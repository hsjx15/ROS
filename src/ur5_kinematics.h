/** 
 * Auther: hshi17 04/09/18
 **/

#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>

namespace ur5_kinematics {

const double PI = 3.141592653589793238;

// parameters of UR5
const double _L0 = 0.0892;
const double _L1 = 0.425;
const double _L2 = 0.392;
const double _L3 = 0.1093;
const double _L4 = 0.09475;
const double _L5 = 0.0825;

// define coordinate basis vectors
const Eigen::Vector3d _e1 = (Eigen::Vector3d() << 1, 0, 0).finished();
const Eigen::Vector3d _e2 = (Eigen::Vector3d() << 0, 1, 0).finished();
const Eigen::Vector3d _e3 = (Eigen::Vector3d() << 0, 0, 1).finished();

// define axes of UR5
const Eigen::Vector3d _w1 = _e3;
const Eigen::Vector3d _q1 = (Eigen::Vector3d() << 0, 0, _L0).finished();
const Eigen::Vector3d _w2 = _e2;
const Eigen::Vector3d _q2 = (Eigen::Vector3d() << 0, 0, _L0).finished();
const Eigen::Vector3d _w3 = _e2;
const Eigen::Vector3d _q3 =(Eigen::Vector3d() << _L1, 0, _L0).finished();
const Eigen::Vector3d _w4 = _e2;
const Eigen::Vector3d _q4 =(Eigen::Vector3d() << _L1+_L2, 0, _L0).finished();
const Eigen::Vector3d _w5 = -1*_e3;
const Eigen::Vector3d _q5 =(Eigen::Vector3d() << _L1+_L2, _L3, _L0).finished();
const Eigen::Vector3d _w6 = _e2;
const Eigen::Vector3d _q6 =(Eigen::Vector3d() << _L1+_L2, _L3, _L0-_L4).finished();

// define homogeneous transformation of the initial position
const Eigen::Matrix4d _gst0 = (Eigen::Matrix4d() << -1, 0, 0, _L1+_L2, 0, 0, 1, _L3+_L5, 0, 1, 0, _L0-_L4, 0, 0, 0, 1).finished();

// kinetics and controls
// trajectory_msgs::JointTrajectory ur5_rate_control(const std::vector<double>& initial_joint_positions, const Eigen::Matrix4d& gst_initial, const Eigen::Vector3d& position_desired, const Eigen::Vector3d& orientation_desired, double increment, double K);
// trajectory_msgs::JointTrajectory ur5_rate_control(const std::vector<double>& initial_joint_positions, const std::vector<double>& position_desired, const std::vector<double>& orientation_desired, double increment, double K);
void ur5_rate_control(trajectory_msgs::JointTrajectory& trajectory, const std::vector<double>& initial_joint_positions, const std::vector<double>& position_desired_vector, const std::vector<double>& orientation_desired_vector, double increment, double K);


tf::Point ur5_FK_position (const std::vector<double>& positions);

Eigen::Matrix4d ur5_ForwardKinematics(const std::vector<double>& positions);

Eigen::Matrix<double,6,6> ur5BodyJacobian(const std::vector<double>& joint_positions);

// utility functions
double manipulability(const Eigen::Matrix<double,6,6> J, const std::string& measure);
Eigen::Matrix<double,6,1> getXi(const Eigen::Matrix4d& g);
Eigen::Matrix4d GenerateHomoTrans (const Eigen::Vector3d& position, const Eigen::Vector3d& orientation);
Eigen::Matrix<double,6,1> RevoluteTwist (const Eigen::Vector3d& q, const Eigen::Vector3d& omega);
Eigen::Matrix4d TwistExp (const Eigen::Matrix<double,6,1>& xi, double theta);
Eigen::Matrix<double,6,6> AdjTrans(const Eigen::Matrix4d& g);
Eigen::Matrix4d Inv_HomoTrans (const Eigen::Matrix4d& g);
Eigen::Matrix3d Omega2Omega_hat (const Eigen::Vector3d& omega);
Eigen::Matrix3d ROTX (double phi);
Eigen::Matrix3d ROTY (double theta);
Eigen::Matrix3d ROTZ (double psi);

// get twists of each axis
const Eigen::VectorXd _xi1 = RevoluteTwist(_q1, _w1);
const Eigen::VectorXd _xi2 = RevoluteTwist(_q2, _w2);
const Eigen::VectorXd _xi3 = RevoluteTwist(_q3, _w3);
const Eigen::VectorXd _xi4 = RevoluteTwist(_q4, _w4);
const Eigen::VectorXd _xi5 = RevoluteTwist(_q5, _w5);
const Eigen::VectorXd _xi6 = RevoluteTwist(_q6, _w6);

}

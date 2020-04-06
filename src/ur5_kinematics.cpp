/** 
 * Auther: hshi17 04/09/18
 **/

#include "ur5_kinematics.h"

namespace ur5_kinematics {

Eigen::Matrix4d ur5_ForwardKinematics(const std::vector<double>& positions) {

    // forward kinematics

    // get screws of each axis
    Eigen::Matrix4d S1;     S1 = TwistExp(_xi1, positions[0]);
    Eigen::Matrix4d S2;     S2 = TwistExp(_xi2, positions[1]);
    Eigen::Matrix4d S3;     S3 = TwistExp(_xi3, positions[2]);
    Eigen::Matrix4d S4;     S4 = TwistExp(_xi4, positions[3]);
    Eigen::Matrix4d S5;     S5 = TwistExp(_xi5, positions[4]);
    Eigen::Matrix4d S6;     S6 = TwistExp(_xi6, positions[5]);

    // compute end effector post gst
    Eigen::Matrix4d gst = S1*S2*S3*S4*S5*S6*_gst0;
    return gst;

}

tf::Point ur5_FK_position (const std::vector<double>& positions) {
    Eigen::Matrix4d gst = ur5_ForwardKinematics(positions);
    double x, y, z;
    x = gst(0,3);
    y = gst(1,3);
    z = gst(2,3);
    return tf::Point(x, y, z);
}

// trajectory_msgs::JointTrajectory ur5_rate_control(const std::vector<double>& initial_joint_positions, const std::vector<double>& position_desired_vector, const std::vector<double>& orientation_desired_vector, double increment, double K) {
void ur5_rate_control(trajectory_msgs::JointTrajectory& trajectory, const std::vector<double>& initial_joint_positions, const std::vector<double>& position_desired_vector, const std::vector<double>& orientation_desired_vector, double increment, double K) {
    
    // initialization
    double T_step = 1/K;                // the initial step size
    double v_k_thres = 0.001;           // the threshold for v_k, 1 mm
    double omega_k_thres = PI*1/360;    // the threshold for omega_k, 1 deg
    double epsilon = 1e-5;              // the threshold for Jacobian matrix singularity
    double final_err = 0;               // final error
    double time_from_start = increment; // time duration for trajectory

    Eigen::Matrix4d gst_initial = ur5_ForwardKinematics(initial_joint_positions);
                // ROS_INFO("current joint: %lf, %lf, %lf, %lf, %lf, %lf", initial_joint_positions(0), initial_joint_positions(1), initial_joint_positions(2), initial_joint_positions(3), initial_joint_positions(4), initial_joint_positions(5));
    Eigen::VectorXd position_desired = Eigen::Map<Eigen::VectorXd>(std::vector<double> (position_desired_vector).data(), position_desired_vector.size());
    Eigen::VectorXd orientation_desired = Eigen::Map<Eigen::VectorXd>(std::vector<double> (orientation_desired_vector).data(), orientation_desired_vector.size());

    // initialize trajecotry
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    trajectory_point.positions = initial_joint_positions;
    trajectory_point.time_from_start = ros::Duration(time_from_start);
    trajectory.points.push_back(trajectory_point);


    // generate gst_desired
    Eigen::Matrix4d gst_desired;
    gst_desired = GenerateHomoTrans(position_desired, orientation_desired);

    // get inverse matrix of gdesired
    Eigen::Matrix4d gst_desired_inv = Inv_HomoTrans(gst_desired);

    // calculate current xi
    Eigen::Matrix4d gtt = gst_desired_inv * gst_initial;
    Eigen::VectorXd xi = getXi(gtt);

    // calculate current distance
    Eigen::Vector3d v_k = xi.head(3);
    Eigen::Vector3d omega_k = xi.tail(3);

    // implement discrete-time resolved rate control

    std::vector<double> current_joint_positions = initial_joint_positions;
    Eigen::Matrix4d gst_current = gst_initial;

    while ((v_k.norm()>v_k_thres) || (omega_k.norm()>omega_k_thres)) {

        // calculate Jacobian matrix
        Eigen::Matrix<double, 6, 6> Jbst = ur5BodyJacobian(current_joint_positions);

        // check for singularity
        double mu_min = manipulability(Jbst, "sigmamin");
                    // ROS_INFO("current mu_min is: %lf", mu_min);
        double mu_det = manipulability(Jbst, "detjac");
                    // ROS_INFO("current mu_det is: %lf", mu_det);
        double mu_inv = manipulability(Jbst, "invcond");
                    // ROS_INFO("current mu_inv is: %lf", mu_inv);
        double mu = std::min({std::abs(mu_min), std::abs(mu_det), std::abs(mu_inv)});
                    ROS_INFO("current mu is: %lf", mu);
        if (mu<epsilon) {
            ROS_WARN("condition near a singularity.");
            final_err = -1;
            return;
            // return trajectory;
        }

        // adjust time step
        double T_step_adjust = T_step*xi.norm()/(Jbst.inverse()*xi).norm();

        // update new joint angles
        Eigen::VectorXd q_current = Eigen::Map<Eigen::VectorXd>(current_joint_positions.data(), current_joint_positions.size());
        Eigen::Matrix<double,6,1> q_diff = K*T_step_adjust*(Jbst.inverse()*xi);
        while (q_diff.norm() > 1.0) {
            q_diff = q_diff/std::exp(xi.norm());
        }
        q_current = q_current - q_diff;
        current_joint_positions = {q_current(0), q_current(1), q_current(2), q_current(3), q_current(4), q_current(5)};

        // calculate suitable time interval for UR5 to move
        double T_real = 5*T_step*std::sqrt(q_diff.norm());
        T_real = std::max(T_real, 0.8);
        time_from_start = time_from_start + T_real;

        // update trajectory point
        trajectory_point.positions = current_joint_positions;
        trajectory_point.time_from_start = ros::Duration(time_from_start);
        trajectory.points.push_back(trajectory_point);

        // update current transformation
        gst_current = ur5_ForwardKinematics(current_joint_positions);

        // calculate updated xi
        gtt = gst_desired_inv * gst_current;
        xi = getXi(gtt);

        // calculate updated distance
        v_k = xi.head(3);
        omega_k = xi.tail(3);

    }

    // calculate final position err
    Eigen::Vector3d p_desired = gst_desired.col(3).head(3);
    Eigen::Vector3d p = gst_current.col(3).head(3);
    final_err = (p-p_desired).norm();
    ROS_INFO("destination reached, residual position error is %lf", final_err);

    // return trajectory;

}

Eigen::Matrix<double,6,6> ur5BodyJacobian(const std::vector<double>& joint_positions) {

    // forward kinematics

    // get screws of each axis
    Eigen::Matrix4d S1;     S1 = TwistExp(_xi1, joint_positions[0]);
    Eigen::Matrix4d S2;     S2 = TwistExp(_xi2, joint_positions[1]);
    Eigen::Matrix4d S3;     S3 = TwistExp(_xi3, joint_positions[2]);
    Eigen::Matrix4d S4;     S4 = TwistExp(_xi4, joint_positions[3]);
    Eigen::Matrix4d S5;     S5 = TwistExp(_xi5, joint_positions[4]);
    Eigen::Matrix4d S6;     S6 = TwistExp(_xi6, joint_positions[5]);

    // compute end effector post gst
    Eigen::Matrix4d gst;
    gst = S1*S2*S3*S4*S5*S6*_gst0;

    // get spatial Jacobian
    Eigen::Matrix<double, 6, 1> Jsst1 = _xi1;
    Eigen::Matrix<double, 6, 1> Jsst2 = AdjTrans(S1)*_xi2;
	Eigen::Matrix<double, 6, 1> Jsst3 = AdjTrans(S1*S2)*_xi3;
	Eigen::Matrix<double, 6, 1> Jsst4 = AdjTrans(S1*S2*S3)*_xi4;
	Eigen::Matrix<double, 6, 1> Jsst5 = AdjTrans(S1*S2*S3*S4)*_xi5;
	Eigen::Matrix<double, 6, 1> Jsst6 = AdjTrans(S1*S2*S3*S4*S5)*_xi6;

    Eigen::Matrix<double, 6, 6> Js = (Eigen::MatrixXd(6,6) << Jsst1, Jsst2, Jsst3, Jsst4, Jsst5, Jsst6).finished();

    // get body Jacobian
    Eigen::Matrix<double, 6, 6> Jb = AdjTrans(Inv_HomoTrans(gst))*Js ;

    return Jb;

}

double manipulability(const Eigen::Matrix<double,6,6> J, const std::string& measure) {
    // singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double,6,1> s = (Eigen::VectorXd(6) << svd.singularValues()).finished();
    double mu;
    if (!measure.compare("sigmamin")) {
        mu = s.minCoeff();
    } else if (!measure.compare("detjac")) {
        mu = J.determinant();
    } else if (!measure.compare("invcond")) {
        mu = s.minCoeff()/s.maxCoeff();
    } else {
        // throw exception  
    }
    return mu;
}


Eigen::Matrix<double,6,1> getXi(const Eigen::Matrix4d& g) {

    Eigen::Matrix3d R = g.block<3,3>(0,0);
    Eigen::Vector3d p = g.col(3).head(3);
    Eigen::Vector3d omega;
    Eigen::Vector3d v;
    double theta;

    if ((R - Eigen::Matrix3d::Identity()).norm() < 1e-3) {  // special case, R is an identity matrix
        omega = Eigen::Vector3d::Zero();
        if (p.norm() < 1e-6) {      // g is an identity matrix
            theta = 0;
            v = Eigen::Vector3d::Zero();
        } else {        // g is not an identity matrix
            theta = p.norm();
            v = p/theta;
        }
    } else {    // normal case
        theta = acos((R.trace()-1)/2);
        if (std::abs(theta) > 1e-6) {   // theta is not near zero
            omega << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
            omega = omega/2/sin(theta);
        } else {
            omega = Eigen::Vector3d::Zero();
        }
        v = ((Eigen::Matrix3d::Identity()-R)*Omega2Omega_hat(omega)+omega*omega.transpose()*theta).inverse()*p;
    }

    Eigen::VectorXd xi(6);
    xi.head(3) = v;
    xi.tail(3) = omega;
    xi = theta*xi;
    return xi;

}

Eigen::Matrix4d GenerateHomoTrans (const Eigen::Vector3d& position, const Eigen::Vector3d& orientation) {
    Eigen::Matrix4d gst;
    // XYZ convention
    Eigen::Matrix3d R = ROTX(orientation(0))*ROTY(orientation(1))*ROTZ(orientation(2));
    gst.block<3,3>(0,0) = R;
    gst.col(3).head(3) << position(0), position(1), position(2);
    gst.row(3) << 0, 0, 0, 1;
    return gst;
}

Eigen::Matrix<double,6,1> RevoluteTwist (const Eigen::Vector3d& q, const Eigen::Vector3d& omega) {
    Eigen::Matrix3d omega_hat;
    omega_hat = Omega2Omega_hat(omega);

    Eigen::VectorXd xi(6);
    xi.head(3) = -1*omega_hat*q;
    xi.tail(3) = omega;
    return xi;
}

Eigen::Matrix4d TwistExp (const Eigen::Matrix<double,6,1>& xi, double theta) {
    Eigen::Vector3d v;  v = xi.head(3);
    Eigen::Vector3d omega;  omega = xi.tail(3);

    Eigen::Matrix3d omega_hat;  omega_hat = Omega2Omega_hat(omega);

    Eigen::Matrix4d tw;
    tw.block<3,3>(0,0) = omega_hat;
    tw.col(3).head(3) = v;
    tw.row(3) = Eigen::MatrixXd::Zero(1,4);

    Eigen::Matrix4d S;
    tw = theta*tw;
    S = tw.exp();
    return S;
}

Eigen::Matrix<double,6,6> AdjTrans(const Eigen::Matrix4d& g) {
    Eigen::Matrix3d R = g.block<3,3>(0,0);
    Eigen::Vector3d p = g.col(3).head(3);
    Eigen::Matrix3d p_hat = Omega2Omega_hat(p);
    Eigen::MatrixXd Adg(6,6);
    Adg.block<3,3>(0,0) = R;
    Adg.block<3,3>(0,3) = p_hat*R;
    Adg.block<3,3>(3,0) = Eigen::Matrix3d().Zero();
    Adg.block<3,3>(3,3) = R;
    return Adg;
}

Eigen::Matrix4d Inv_HomoTrans (const Eigen::Matrix4d& g) {
    Eigen::Matrix3d R = g.block<3,3>(0,0);
    Eigen::Vector3d p = g.col(3).head(3);
	Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Matrix4d g_inv;
    g_inv = g;
	g_inv.block<3,3>(0,0) = R_inv;
    g_inv.col(3).head(3) = -R_inv*p;
    return g_inv;
}

Eigen::Matrix3d Omega2Omega_hat (const Eigen::Vector3d& omega) {
    Eigen::Matrix3d omega_hat;
    omega_hat.row(0) << 0, -1*omega(2), omega(1);
    omega_hat.row(1) << omega(2), 0, -1*omega(0);
    omega_hat.row(2) << -1*omega(1), omega(0), 0;
    return omega_hat;
}

Eigen::Matrix3d ROTX (double phi) {
    Eigen::Matrix3d Rx;
    Rx.row(0) << 1, 0, 0;
    Rx.row(1) << 0, cos(phi), -sin(phi);
    Rx.row(2) << 0, sin(phi), cos(phi);
    return Rx;
}

Eigen::Matrix3d ROTY (double theta) {
    Eigen::Matrix3d Ry;
    Ry.row(0) << cos(theta), 0, sin(theta);
    Ry.row(1) << 0, 1, 0;
    Ry.row(2) << -sin(theta), 0, cos(theta);
    return Ry;
}

Eigen::Matrix3d ROTZ(double psi) {
    Eigen::Matrix3d Rz;
    Rz.row(0) << cos(psi), -sin(psi), 0;
    Rz.row(1) << sin(psi), cos(psi), 0;
    Rz.row(2) << 0, 0, 1;
    return Rz;
}

}
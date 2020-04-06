/** 
 * Auther: hshi17 04/09/18
 **/

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <edumip_msgs/EduMipState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include "ur5_kinematics.h"

namespace ur5_catcher {

class ur5_catcher_rviz {
public:

    ur5_catcher_rviz();
    ur5_catcher_rviz(double position_increment);

private:

    void ur5_catcher_initilizer();

    void ur5_catch_signal_Callback(const std_msgs::Bool signal);
    void ur5_move_Callback(const trajectory_msgs::JointTrajectoryPointConstPtr& joint_goal);
    void joint_state_Callback(const sensor_msgs::JointStateConstPtr& joint_state);
    void edumip_state_Callback(const edumip_msgs::EduMipStateConstPtr& edumip_state);

    ros::NodeHandle node;
    ros::Subscriber ur5_catch_signal_sub;
    ros::Subscriber ur5_move_sub;
    ros::Subscriber joint_state_sub;
    ros::Subscriber edumip_state_sub;
    ros::Publisher  joint_trajectory_pub;

    double _position_increment;       // how much we move between steps for trajectory generation
    std::vector<double> edumip_position;
    std::vector<double> edumip_orientation;

    trajectory_msgs::JointTrajectory trajectory;
    trajectory_msgs::JointTrajectoryPoint current_positions;

    double shoulder_pan_joint;
    double shoulder_lift_joint;
    double elbow_joint;
    double wrist_1_joint;
    double wrist_2_joint;
    double wrist_3_joint;

    // supplimentary code 
    tf::TransformBroadcaster br;
    tf::Transform transform;

};

ur5_catcher_rviz::ur5_catcher_rviz(): _position_increment(1e-3) {
    ur5_catcher_initilizer();
}

ur5_catcher_rviz::ur5_catcher_rviz(double position_increment) : _position_increment(position_increment) {
    ur5_catcher_initilizer();
}

void ur5_catcher_rviz::ur5_catcher_initilizer() {
    ur5_catch_signal_sub  = node.subscribe("/ur5_catch_signal", 1, &ur5_catcher_rviz::ur5_catch_signal_Callback, this);
    ur5_move_sub = node.subscribe("move_ur5", 1, &ur5_catcher_rviz::ur5_move_Callback, this);
    joint_state_sub = node.subscribe("joint_states", 10, &ur5_catcher_rviz::joint_state_Callback, this);
    edumip_state_sub = node.subscribe("/edumip/state", 10, &ur5_catcher_rviz::edumip_state_Callback, this);
    joint_trajectory_pub = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

    // initialize trajectory joint names
    trajectory.joint_names.push_back( "shoulder_pan_joint" );
    trajectory.joint_names.push_back( "shoulder_lift_joint" );
    trajectory.joint_names.push_back( "elbow_joint" );
    trajectory.joint_names.push_back( "wrist_1_joint" );
    trajectory.joint_names.push_back( "wrist_2_joint" );
    trajectory.joint_names.push_back( "wrist_3_joint" );
}

void ur5_catcher_rviz::ur5_catch_signal_Callback(const std_msgs::Bool signal) {
    if (signal.data == true) {
                ROS_INFO("signal received.");      

    // generate catcher transformation   
    std::vector<double> catcher_position = edumip_position;
    std::vector<double> catcher_orientation = edumip_orientation;

    // mark catcher goal transformation
    transform.setOrigin(tf::Vector3(catcher_position[0], catcher_position[1], catcher_position[2]));
    tf::Quaternion q;
	q.setRPY(catcher_orientation[0], catcher_orientation[1], catcher_orientation[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "catcher_goal"));

    // The current position of the end effector
    tf::Point current = ur5_kinematics::ur5_FK_position(current_positions.positions);

    // check forward kinematics
    transform.setOrigin(current);
	q.setRPY(0,0,0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "end_effector"));
    

    // begin trajectory generation (rate control method)
    trajectory.points.clear();
    ur5_kinematics::ur5_rate_control(trajectory, current_positions.positions, catcher_position, catcher_orientation, _position_increment, 1.0);

    // send trajectory
    joint_trajectory_pub.publish(trajectory);
    ROS_INFO("trajectory published.");
    }
}

void ur5_catcher_rviz::ur5_move_Callback(const trajectory_msgs::JointTrajectoryPointConstPtr& joint_goal) {
    trajectory.points.clear();
    trajectory.points.push_back(current_positions);

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    
    // check speed limit

    // set trajectory
    trajectory_point.positions = joint_goal->positions;
    trajectory_point.time_from_start = joint_goal->time_from_start;
    trajectory.points.push_back(trajectory_point);

    // publish the trajectory
    joint_trajectory_pub.publish(trajectory);
}

void ur5_catcher_rviz::joint_state_Callback(const sensor_msgs::JointStateConstPtr& joint_state) {
    // read joints accordingly in a certain sequnce
    for (size_t i=0; i<joint_state->name.size(); i++) {
        if (joint_state->name[i] == "shoulder_pan_joint") {
            shoulder_pan_joint = joint_state->position[i];
        } else if (joint_state->name[i] == "shoulder_lift_joint") {
            shoulder_lift_joint = joint_state->position[i];
        } else if (joint_state->name[i] == "elbow_joint") {
            elbow_joint = joint_state->position[i];
        } else if (joint_state->name[i] == "wrist_1_joint") {
            wrist_1_joint = joint_state->position[i];
        } else if (joint_state->name[i] == "wrist_2_joint") {
            wrist_2_joint = joint_state->position[i];
        } else if (joint_state->name[i] == "wrist_3_joint") {
            wrist_3_joint = joint_state->position[i];
        }
    }
    current_positions.positions = {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,wrist_3_joint};
    // current_positions.velocities = joint_state->velocity;
    // current_positions.effort = joint_state->effort;
    current_positions.time_from_start = ros::Duration(_position_increment);
}

void ur5_catcher_rviz::edumip_state_Callback(const edumip_msgs::EduMipStateConstPtr& edumip_state) {  
    edumip_position = {edumip_state->body_frame_northing, -1*edumip_state->body_frame_easting, 0.134};
    edumip_orientation = {0.0, edumip_state->setpoint_theta, -1*edumip_state->body_frame_heading};          // XYZ convention
}



}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ur5_catcher_rviz");
                ROS_INFO("ur5 catcher starting here.");

	ur5_catcher::ur5_catcher_rviz ur5_catcher(1e-3);

	ros::spin();
	return 0;
}
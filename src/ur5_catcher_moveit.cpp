/**
 * Auther: hshi17 04/26/18
 **/

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <edumip_msgs/EduMipState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

namespace ur5_catcher {

double Catcher_Move_Distance_ = 0.3;
double Gripper_EduMIP_Distance_ = 0.11;
double EduMIP_ARtag_Shift_ = 0.0;
const double UR5_Surface_Height_ = 1.50;

double camera_pose_x = -0.1;
double camera_pose_y = 0;
double camera_pose_z = 3.1;
double camera_orien_x = 1;
double camera_orien_y = 0;
double camera_orien_z = 0;
double camera_orien_w = 0;

tf::Transform Camera_Pose_(
    tf::Quaternion(camera_orien_x, camera_orien_y, camera_orien_z, camera_orien_w), 
    tf::Vector3(camera_pose_x, camera_pose_y, camera_pose_z));
// const tf::Transform Base_Link_(
//     tf::Quaternion(0, 0, std::sqrt(2.0)/2.0, std::sqrt(2.0)/2.0),
//     tf::Vector3(0, 0, 1.510));

geometry_msgs::Pose Trans_to_Pose (const tf::Transform& trans) {
    geometry_msgs::Pose pose;
    pose.orientation.x = trans.getRotation().getX();
    pose.orientation.y = trans.getRotation().getY();
    pose.orientation.z = trans.getRotation().getZ();
    pose.orientation.w = trans.getRotation().getW();
    pose.position.x = trans.getOrigin().getX();
    pose.position.y = trans.getOrigin().getY();
    pose.position.z = trans.getOrigin().getZ(); 
    return pose;
}

double Pose_Distance (const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    double delta_x = pose1.position.x - pose2.position.x;
    double delta_y = pose1.position.y - pose2.position.y;
    double delta_z = pose1.position.z - pose2.position.z;
    double distance = std::sqrt(std::pow(delta_x, 2.0)+std::pow(delta_y, 2.0)+std::pow(delta_z, 2.0));
    return distance;
}

class ur5_catcher_moveit {
public:

    ur5_catcher_moveit(const std::string& PLANNING_GROUP, const std::string& REFERENCE_FRAME, const std::string& ROBOT_DESCRIPTION);

    void ur5_move_home();

    bool ur5_check_reachability(tf::Transform& trans);  // check if certain pose is within UR5's catch range

    bool ur5_availability();    // check if UR5 is avaiable for catching motion

    geometry_msgs::Pose generate_pickup_pose();
    geometry_msgs::Pose generate_drop_pose();
    geometry_msgs::Pose generate_catcher_pose(const tf::Transform& trans);

    void ur5_add_desk();
    void ur5_add_edumip();
    void ur5_publish_collision_objects();

    void ur5_remove_last();
    void ur5_remove_all();
    
    bool ur5_move(const geometry_msgs::Pose& gripper_pose, const std::string& motion_name);
    bool ur5_plan(const geometry_msgs::Pose& gripper_pose);

    void ur5_get_current_joint_states();
    void ur5_get_trajectory_points();

    void ur5_open_gripper();
    void ur5_close_gripper();

private:

    // void edumip_state_Callback(const edumip_msgs::EduMipStateConstPtr& edumip_state);
    // void edumip_height_Callback(const std_msgs::Float64& height);
    void aruco_pose1_Callback(const geometry_msgs::PoseConstPtr& pose);
    void aruco_pose2_Callback(const geometry_msgs::PoseConstPtr& pose);
    void set_aruco_pose(const geometry_msgs::PoseConstPtr& pose, tf::Transform& trans);

    // can only be called via ur5_move() function
    bool ur5_execute();

    ros::NodeHandle node;
    // ros::Subscriber edumip_state_sub;
    // ros::Subscriber edumip_height_sub;
    ros::Subscriber aruco_pose1_sub;    // edumip marker
    ros::Subscriber aruco_pose2_sub;    // fixed marker
    ros::Publisher gripper_controller_pub;

    tf::Transform edumip_pose;
    bool edumip_availability;
    tf::Transform fixed_pose;
    bool fixed_availability;

    bool edumip_in_sight;

    // setup moveit!
    const std::string planning_group;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_visual_tools::MoveItVisualTools visual_tools;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<std::string> object_ids;

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup *joint_model_group;

    int plan_attempts;
};

ur5_catcher_moveit::ur5_catcher_moveit(const std::string& PLANNING_GROUP,
        const std::string& REFERENCE_FRAME, const std::string& ROBOT_DESCRIPTION) : 
    edumip_pose{tf::Quaternion(0,0,0,0), tf::Vector3(0,0,0)},
    edumip_availability{false},
    fixed_pose{tf::Quaternion(0,0,0,0), tf::Vector3(0,0,0)},
    fixed_availability{false},
    edumip_in_sight{false},
    planning_group(PLANNING_GROUP),
    move_group(PLANNING_GROUP),
    robot_model_loader(ROBOT_DESCRIPTION),
    kinematic_state(new robot_state::RobotState(robot_model_loader.getModel())),
    visual_tools(REFERENCE_FRAME),
    plan_attempts{3}
{   
    ros::NodeHandle nh_private("~");
    nh_private.param("catcher_move_distance", ur5_catcher::Catcher_Move_Distance_, 0.3);
    nh_private.param("gripper_edumip_distance", ur5_catcher::Gripper_EduMIP_Distance_, 0.11);
    nh_private.param("edumip_artag_shift", ur5_catcher::EduMIP_ARtag_Shift_, 0.0);
    nh_private.param("camera_pose_x", ur5_catcher::camera_pose_x, -0.1);
    nh_private.param("camera_pose_y", ur5_catcher::camera_pose_y, 0.0);
    nh_private.param("camera_pose_z", ur5_catcher::camera_pose_z, 3.1);
    nh_private.param("camera_orien_x", ur5_catcher::camera_orien_x, 1.0);
    nh_private.param("camera_orien_y", ur5_catcher::camera_orien_y, 0.0);
    nh_private.param("camera_orien_z", ur5_catcher::camera_orien_z, 0.0);
    nh_private.param("camera_orien_w", ur5_catcher::camera_orien_w, 0.0);
    ur5_catcher::Camera_Pose_.setOrigin(tf::Vector3(camera_pose_x, camera_pose_y, camera_pose_z));
    ur5_catcher::Camera_Pose_.setRotation(tf::Quaternion(camera_orien_x, camera_orien_y,
        camera_orien_z, camera_orien_w));

    // choose planning algorithm
    std::string planning_algorithm;
    nh_private.param<std::string>("planning_algorithm", planning_algorithm, "RRTConnectkConfigDefault");
    move_group.setPlannerId(planning_algorithm);

    // kinematics
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup(this->planning_group);

    // edumip_state_sub = node.subscribe("/edumip/state", 10, &ur5_catcher_moveit::edumip_state_Callback, this);
    // edumip_height_sub = node.subscribe("/edumip/height", 10, &ur5_catcher_moveit::edumip_height_Callback, this);
    aruco_pose1_sub = node.subscribe("/aruco/pose", 10, &ur5_catcher_moveit::aruco_pose1_Callback, this);
    aruco_pose2_sub = node.subscribe("/aruco/pose2", 10, &ur5_catcher_moveit::aruco_pose2_Callback, this);
    gripper_controller_pub = node.advertise<trajectory_msgs::JointTrajectory>("gripper_controller/command",1);

    // moveit! visualization
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    // get basic information
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // move to home position
    ur5_move_home();
}

void ur5_catcher_moveit::ur5_move_home() {
    std::vector<double> joint_group_positions = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
    this->move_group.setStartState(*this->move_group.getCurrentState());
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    double success;
    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("UR5 %s to home position", success ? "successfully moved" : "failed to move");
    ros::Duration(1.0).sleep();
                visual_tools.prompt("moving home completed, next step");
    if (!success) {
        this->ur5_remove_all();
        ros::shutdown();
    } else {
        this->ur5_remove_all();
    }
}

bool ur5_catcher_moveit::ur5_check_reachability(tf::Transform& trans) {
    // use IK to check pose reachability
    // the number of attempts to be made at solving IK: 10
    // the timeout of each attempt: 0.1s
    geometry_msgs::Pose catcher_pose = this->generate_catcher_pose(trans);
    bool found_ik = this->kinematic_state->setFromIK(this->joint_model_group,
        catcher_pose, 10, 0.1);
    if (found_ik) {
                    ROS_INFO("find IK solution.");
        this->ur5_add_desk();
        this->ur5_add_edumip();
        this->ur5_publish_collision_objects();

        catcher_pose.position.z += ur5_catcher::Catcher_Move_Distance_;
        if (this->ur5_plan(catcher_pose)) {
                        ROS_INFO("Found trajectory to upper pose.");
            // catcher_pose.position.z -= ur5_catcher::Catcher_Move_Distance_;
            // if (this->ur5_plan(catcher_pose)) {
                        // ROS_INFO("Found trajectory to catch pose.");
                this->ur5_remove_all();
                return true; 
            // }
        }
                ROS_INFO("Did not find trajectory.");
        this->ur5_remove_all();
    } else {
                ROS_INFO("Did not find IK solution.");
    }
    return false;
}

bool ur5_catcher_moveit::ur5_availability() {
    if (edumip_availability && fixed_availability) {
        return true;
    }
}

void ur5_catcher_moveit::ur5_add_desk() {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.1;

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = ur5_catcher::UR5_Surface_Height_ - 0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    
    ROS_INFO("Add table into the world");    
    collision_objects.push_back(collision_object);
}

void ur5_catcher_moveit::ur5_add_edumip() {
    moveit_msgs::CollisionObject collision_edumip;
    collision_edumip.header.frame_id = move_group.getPlanningFrame();
    collision_edumip.id = "EduMIP";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;      // shrinked size for catch
    primitive.dimensions[1] = 0.07;     // shrinked size for catch
    primitive.dimensions[2] = 0.07;

    geometry_msgs::Pose table_pose;
    table_pose = ur5_catcher::Trans_to_Pose(this->edumip_pose);
                // ROS_INFO("artag pose is: %lf, %lf, %lf", table_pose.position.x, table_pose.position.y, table_pose.position.z);
    table_pose.position.z = std::max(table_pose.position.z-0.035, 0.035);

    collision_edumip.primitives.push_back(primitive);
    collision_edumip.primitive_poses.push_back(table_pose);
    collision_edumip.operation = collision_edumip.ADD;

    ROS_INFO("Add edumip into the world");
    collision_objects.push_back(collision_edumip);
}

void ur5_catcher_moveit::ur5_publish_collision_objects() {
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep();     // sleep to allow MoveGroup to receive and process the collision object message
}

void ur5_catcher_moveit::ur5_remove_last() {
    object_ids.clear();
    moveit_msgs::CollisionObject collision_object = collision_objects.back();
                ROS_INFO("Remove %s from the world", collision_object.id.c_str());
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    collision_objects.pop_back();
    ros::Duration(1.0).sleep();     // sleep to allow MoveGroup to receive and process the collision object message
}

void ur5_catcher_moveit::ur5_remove_all() {
    object_ids.clear();
    for (std::size_t i = 0; i < collision_objects.size(); i++) {
        moveit_msgs::CollisionObject collision_object = collision_objects[i];
                    ROS_INFO("Remove %s from the world", collision_object.id.c_str());
        object_ids.push_back(collision_object.id);
    }
    planning_scene_interface.removeCollisionObjects(object_ids);
    collision_objects.clear();
    ros::Duration(1.0).sleep();     // sleep to allow MoveGroup to receive and process the collision object message

}

bool ur5_catcher_moveit::ur5_move(const geometry_msgs::Pose& gripper_pose, const std::string& motion_name) {
    bool success;
    success = this->ur5_plan(gripper_pose);
    ROS_INFO("Visualizing plan (%s) %s", motion_name.c_str(), success ? "SUCCESS" : "FAILED");
                // this->visual_tools.prompt("now begin plan execution");
    if (success == true) {
        success = this->ur5_execute();
        ROS_INFO("Executing plan (%s) %s", motion_name.c_str(), success ? "SUCCESS" : "FAILED");
    }
    if (!success) {
        this->ur5_move_home();
    }
                this->visual_tools.prompt("next step");
    return success;
}

bool ur5_catcher_moveit::ur5_plan(const geometry_msgs::Pose& gripper_pose) {

    bool success = false;
    int plan_attempt = 0;

    // plan a trajectory
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(gripper_pose);
    while ((plan_attempt < this->plan_attempts) && !(success)) {
        success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        plan_attempt ++;
    }

    // visualize the plan in rviz
    // visual_tools.deleteAllMarkers();
    // visual_tools.publishAxisLabeled(gripper_pose, "gripper");
    // visual_tools.trigger();

    // if (success) {
    //     this->ur5_get_trajectory_points();
    // }

    return success;
}

bool ur5_catcher_moveit::ur5_execute() {

    // execute the trajectory
    // always throw "start point deviates from current robot state" error

    bool success = false;
    moveit::planning_interface::MoveItErrorCode ErrorCode;
    ErrorCode = this->move_group.execute(this->my_plan);
    if (ErrorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        success = true;
    } else if (ErrorCode == moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED){
        // it's highly likely this error is caused due to the time difference between 
        // ROS and Moveit!, which caused drop of the first few points from the trajectory message,
        // leading to the failure of the hardware driver following the trajectory  
        success = true;
        ROS_INFO("WARNING: control failed error occured, ");
    } else {
        ROS_INFO("current error code is %d", ErrorCode.val);
    }
                // this->visual_tools.prompt("execution finished.");
    return success;
}

// show current joint states
void ur5_catcher_moveit::ur5_get_current_joint_states() {

    std::vector<std::string> joint_names;
    joint_names = this->move_group.getJointNames();
    std::string joint_names_string;
    for (int i=0; i<joint_names.size(); i++) {
        joint_names_string += joint_names[i];
        joint_names_string += " ";
    }
    ROS_INFO("joint names are: %s", joint_names_string.c_str());

    std::vector<double> current_joint_states;
    current_joint_states = this->move_group.getCurrentJointValues();
    std::string joint_values_string;
    for (int i=0; i<current_joint_states.size(); i++) {
        joint_values_string += std::to_string(current_joint_states[i]);
        joint_values_string += " ";
    }
    ROS_INFO("current joint states are: %s", joint_values_string.c_str());
}

// show planeed trajectory
void ur5_catcher_moveit::ur5_get_trajectory_points() {

    std::vector<std::string> joint_names;
    joint_names = this->move_group.getJointNames();
    std::string joint_names_string;
    for (int i=0; i<joint_names.size(); i++) {
        joint_names_string += joint_names[i];
        joint_names_string += " ";
    }
    ROS_INFO("joint names are: %s", joint_names_string.c_str());

    std::string joint_positions;
    for(int i=0; i<this->my_plan.trajectory_.joint_trajectory.points.size(); i++) {
        joint_positions = "";
        for (int j=0; j<this->my_plan.trajectory_.joint_trajectory.points[i].positions.size(); j++) {
            joint_positions += std::to_string(this->my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
            joint_positions += std::string(" ");
        }
        ROS_INFO("the position of joints are %s", joint_positions.c_str());
    }
}

void ur5_catcher_moveit::ur5_open_gripper() {
    trajectory_msgs::JointTrajectory trajectory;

    trajectory.header.seq = 0;
    trajectory.header.stamp.sec = 0;
    trajectory.header.stamp.nsec = 0;
    trajectory.header.frame_id = "";
    trajectory.joint_names = {"left_gripper_joint", "right_gripper_joint"};

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = {1.0, 1.0};
    trajectory_point.velocities = {0.01, 0.01};
    trajectory_point.accelerations = {};
    trajectory_point.effort = {};
    trajectory_point.time_from_start.sec = 1;
    trajectory_point.time_from_start.nsec = 0;

    trajectory.points.push_back(trajectory_point);

    gripper_controller_pub.publish(trajectory);
    ros::Duration(1.0).sleep();

            ROS_INFO("gripper opened");
}

void ur5_catcher_moveit::ur5_close_gripper() {
    trajectory_msgs::JointTrajectory trajectory;

    trajectory.header.seq = 0;
    trajectory.header.stamp.sec = 0;
    trajectory.header.stamp.nsec = 0;
    trajectory.header.frame_id = "";
    trajectory.joint_names = {"left_gripper_joint", "right_gripper_joint"};

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = {0.0, 0.0};
    trajectory_point.velocities = {0.01, 0.01};
    trajectory_point.accelerations = {};
    trajectory_point.effort = {};
    trajectory_point.time_from_start.sec = 1;
    trajectory_point.time_from_start.nsec = 0;

    trajectory.points.push_back(trajectory_point);

    gripper_controller_pub.publish(trajectory);
    ros::Duration(1.0).sleep();

            ROS_INFO("gripper closed");
}

geometry_msgs::Pose ur5_catcher_moveit::generate_pickup_pose() {
    geometry_msgs::Pose pose = generate_catcher_pose(this->edumip_pose);
    return pose;
}

geometry_msgs::Pose ur5_catcher_moveit::generate_drop_pose() {
    geometry_msgs::Pose pose = generate_catcher_pose(this->fixed_pose);
    return pose;
}

geometry_msgs::Pose ur5_catcher_moveit::generate_catcher_pose(const tf::Transform& trans) {
    tf::Transform catcher_trans = trans;
    catcher_trans.setRotation(catcher_trans.getRotation() *
        tf::Quaternion(0, std::sqrt(2)/2.0, 0, std::sqrt(2)/2.0));
    geometry_msgs::Pose catcher_pose = ur5_catcher::Trans_to_Pose(catcher_trans);
    catcher_pose.position.z += ur5_catcher::Gripper_EduMIP_Distance_;
    return catcher_pose;

/*    
    tf::Transform catcher_trans = this->edumip_pose;
                ROS_INFO("current rotation: %lf, %lf, %lf, %lf", edumip_pose.getRotation().getX(),
                    edumip_pose.getRotation().getY(), edumip_pose.getRotation().getZ(), edumip_pose.getRotation().getW());
    catcher_trans.setRotation(catcher_trans.getRotation() * 
        tf::Quaternion(0, std::sqrt(2)/2.0, 0, std::sqrt(2)/2.0));
    catcher_trans.setOrigin(tf::Vector3(catcher_trans.getOrigin().getX(),
        catcher_trans.getOrigin().getY(),
        catcher_trans.getOrigin().getZ() + ur5_catcher::Gripper_EduMIP_Distance_));

    geometry_msgs::Pose catcher_pose = ur5_catcher::Trans_to_Pose(catcher_trans);
    return catcher_pose;
*/
}

/* commentted by hs, not rely on edumip state to determine edumiop pose
void ur5_catcher_moveit::edumip_state_Callback(const edumip_msgs::EduMipStateConstPtr& edumip_state) {
    edumip_pose.position.x = edumip_state->body_frame_northing;
    edumip_pose.position.y = -1*edumip_state->body_frame_easting;
    // edumip_pose.position.z = 1.534;;
    tf::Quaternion q;
    q.setRPY(0.0, edumip_state->setpoint_theta, -1*edumip_state->body_frame_heading);   // XYZ convention   
    edumip_pose.orientation.x = q[0];
    edumip_pose.orientation.y = q[1];
    edumip_pose.orientation.z = q[2];
    edumip_pose.orientation.w = q[3]; 
}

void ur5_catcher_moveit::edumip_height_Callback(const std_msgs::Float64& height) {
    edumip_pose.position.z = height.data;
}
*/

void ur5_catcher_moveit::aruco_pose1_Callback(const geometry_msgs::PoseConstPtr& pose) {
    if (!(this->edumip_in_sight)) {
        ROS_INFO("EduMIP is in the sight of camera.");
        this->edumip_in_sight = true;
    }

    geometry_msgs::Pose current_pose = *pose;
    static geometry_msgs::Pose last_pose = current_pose;
    static geometry_msgs::Pose last_still_pose = current_pose;

    if (!this->edumip_availability) {
        if (ur5_catcher::Pose_Distance(current_pose, last_pose) < 1e-3) {
            // EduMIP is still
            if (ur5_catcher::Pose_Distance(current_pose, last_still_pose) > 1e-2) {
                // moved enough distance, update pose
                set_aruco_pose(pose, this->edumip_pose);
                // compensate for ARtag shift on EduMIP
                this->edumip_pose.setOrigin(tf::Vector3(
                    this->edumip_pose.getOrigin().getX()-ur5_catcher::EduMIP_ARtag_Shift_,
                    this->edumip_pose.getOrigin().getY(), this->edumip_pose.getOrigin().getZ()));
                if (ur5_check_reachability(this->edumip_pose)) {
                    ROS_INFO("current position is within transporting range.");
                    this->edumip_availability = true;
                } else {
                    ROS_INFO("current position is out of transporting range, move closer.");
                    this->edumip_availability = false;
                }
            }
            last_still_pose = current_pose;
        }
    }
    last_pose = current_pose;
}

void ur5_catcher_moveit::aruco_pose2_Callback(const geometry_msgs::PoseConstPtr& pose) {
    set_aruco_pose(pose, this->fixed_pose);
    // if (ur5_check_reachability(this->fixed_pose)) {  // current condition no need to check this
        this->fixed_availability = true;
    // } else {
        // this->fixed_availability = false;
    // }
}

void ur5_catcher_moveit::set_aruco_pose(const geometry_msgs::PoseConstPtr& pose, tf::Transform& trans) {
    trans.setOrigin(tf::Vector3(ur5_catcher::Camera_Pose_.getOrigin().getX() + pose->position.x,
        ur5_catcher::Camera_Pose_.getOrigin().getY() - pose->position.y,
        ur5_catcher::Camera_Pose_.getOrigin().getZ() - pose->position.z));
    trans.setRotation(ur5_catcher::Camera_Pose_.getRotation() * 
        tf::Quaternion(pose->orientation.x, pose->orientation.y,
        pose->orientation.z, pose->orientation.w));
}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ur5_catcher_moveit");
                ROS_INFO("ur5 catcher starting here.");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ur5_catcher::ur5_catcher_moveit my_ur5_catcher("manipulator", "world", "robot_description");
    ros::Duration(1).sleep();

    geometry_msgs::Pose gripper_pose;

    while(ros::ok()) {
        if (my_ur5_catcher.ur5_availability()) {

                    ROS_INFO("EduMIP is within catching range");
                    
        // generate catcher transformation
        gripper_pose = my_ur5_catcher.generate_pickup_pose();

        // add collision object: table
        my_ur5_catcher.ur5_add_desk();

        // add collision object: edumip
        my_ur5_catcher.ur5_add_edumip();

        // publish simulated collision objects
        my_ur5_catcher.ur5_publish_collision_objects();

        /**
         *  begin EduMIP transporting
         */

        // step 1, move to a position a liitle higher than the desired position
        gripper_pose.position.z += ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "above EduMIP")){
            break;
        }


        /* open gripper */
        my_ur5_catcher.ur5_open_gripper();

        // step 2, move down gripper
        gripper_pose.position.z -= ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "catch EduMIP")){
            break;
        }
        my_ur5_catcher.ur5_remove_last();

        // /* close gripper */
        my_ur5_catcher.ur5_close_gripper();

        // step 3, pick up EduMIP
        gripper_pose.position.z += ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "pick up EduMIP")) {
            break;
        }

        /* receive infromation about next position */
        gripper_pose = my_ur5_catcher.generate_drop_pose();

        // step 4, transport EduMIP
        gripper_pose.position.z += ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "transport EduMIP")) {
            break;
        }

        // step 5, put down EduMIP
        gripper_pose.position.z -= ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "put down EduMIP")) {
            break;
        }

        /* open gripper */
        my_ur5_catcher.ur5_open_gripper();

        // step 6, move up gripper
        my_ur5_catcher.ur5_add_edumip();
        gripper_pose.position.z += ur5_catcher::Catcher_Move_Distance_;
        if (!my_ur5_catcher.ur5_move(gripper_pose, "move up gripper")) {
            break;
        }

        // // step 7, move home
        my_ur5_catcher.ur5_move_home();

        /**
         *  EduMIP transporting finished
         */

        my_ur5_catcher.ur5_remove_all();
        ros::shutdown();
        }
    }
    
    // ros::waitForShutdown();

    // ros::spin();

	return 0;
}

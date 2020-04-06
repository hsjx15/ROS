/** Auther: hshi17 02/27/18
 *  package: edumip_my_robot
 **/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <edumip_msgs/EduMipState.h>
#include <nav_msgs/Odometry.h>

namespace edumip_my_robot_state_publisher {

class StatePublisher {
public:
	StatePublisher();
private:

	void stateCallback(const edumip_msgs::EduMipStateConstPtr& state);
	void heightCallback(const std_msgs::Float64& height);
	void timerCallback(const ros::TimerEvent&);

	ros::NodeHandle node;
	sensor_msgs::JointState joint_state;

	ros::Subscriber state_sub;
	ros::Subscriber height_sub;		// for gazebo simulation
	ros::Publisher joint_pub;
	ros::Timer timer;

	double edumip_height_;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	std::string tf_prefix_;

	// publish odometry message
	ros::Time current_time;
	ros::Time last_time;
	std::string odom_frame_id;
	std::string odom_child_frame_id;
	ros::Publisher odom_pub;
	nav_msgs::Odometry odom;
	geometry_msgs::Pose last_pose;
};

StatePublisher::StatePublisher() : 
	edumip_height_{0.034},		// in case no publish of height message (default on ground)
	tf_prefix_{""},
	odom_frame_id{"simu_odom"},
	odom_child_frame_id{"edumip_body"}
{
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("tf_prefix", tf_prefix_, std::string(""));
	nh_private.param<std::string>("odom_frame_id", odom_frame_id, "simu_odom");
	nh_private.param<std::string>("odom_child_frame_id", odom_child_frame_id, "edumip_body");

	state_sub = node.subscribe("/edumip/state", 10, &StatePublisher::stateCallback, this);
	height_sub = node.subscribe("/edumip/height", 10, &StatePublisher::heightCallback, this);
	joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
	timer = node.createTimer(ros::Duration(0.1), &StatePublisher::timerCallback, this);
	odom_pub = node.advertise<nav_msgs::Odometry>("/edumip/odometry", 10);
}

void StatePublisher::stateCallback(const edumip_msgs::EduMipStateConstPtr& state) {

	transform.setOrigin(tf::Vector3(state->body_frame_northing, -1*state->body_frame_easting, edumip_height_));
	tf::Quaternion q;
	q.setRPY(0, 0, -state->body_frame_heading);
				// ROS_INFO("body frame heading: %lf", state->body_frame_heading);
				// ROS_INFO("current quat: %lf, %lf, %lf, %lf", q[0], q[1], q[2], q[3]);
	transform.setRotation(q);

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	tf::StampedTransform tf_stamped_trans = tf::StampedTransform(transform, current_time, odom_frame_id, odom_child_frame_id);
	tf_stamped_trans.frame_id_ = tf::resolve(tf_prefix_, tf_stamped_trans.frame_id_);
	tf_stamped_trans.child_frame_id_ = tf::resolve(tf_prefix_, tf_stamped_trans.child_frame_id_);
	br.sendTransform(tf_stamped_trans);
	
	joint_state.header.stamp = current_time;
	joint_state.name.resize(4);
	joint_state.position.resize(4);
	joint_state.name[0] = "jointL";
	joint_state.position[0] = state->wheel_angle_L;
	joint_state.name[1] = "jointR";
	joint_state.position[1] = state->wheel_angle_R;
	joint_state.name[2] = "front_wheel_base_joint";
	joint_state.position[2] = 0;
	joint_state.name[3] = "front_wheel_joint";
	joint_state.position[3] = 0;

	// publish odometry message over ROS
	odom.header.stamp = current_time;
	odom.header.frame_id = tf::resolve(tf_prefix_, odom_frame_id);
	odom.child_frame_id = tf::resolve(tf_prefix_, odom_child_frame_id);

	// set the position
	tf::poseTFToMsg(transform, odom.pose.pose);
	odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
							0, 1e-3, 0, 0, 0, 0,
							0, 0, 1e-9, 0, 0, 0,
							0, 0, 0, 1e-9, 0, 0,
							0, 0, 0, 0, 1e-9, 0,
							0, 0, 0, 0, 0, 1e0};
	
	// update velocity information
	odom.twist.twist.linear.x = (odom.pose.pose.position.x - last_pose.position.x)/dt;
	odom.twist.twist.linear.y = (odom.pose.pose.position.y - last_pose.position.y)/dt;
	odom.twist.twist.angular.z = (odom.pose.pose.orientation.z - last_pose.orientation.z)/dt;
	odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
							 0, 1e-3, 0, 0, 0, 0,
							 0, 0, 1e-9, 0, 0, 0,
							 0, 0, 0, 1e-9, 0, 0,
							 0, 0, 0, 0, 1e-9, 0,
							 0, 0, 0, 0, 0, 1e0};

	last_pose = odom.pose.pose;
}

void StatePublisher::heightCallback(const std_msgs::Float64& height) {
	edumip_height_ = height.data;
}

void StatePublisher::timerCallback(const ros::TimerEvent&) {
	joint_pub.publish(joint_state);
	odom_pub.publish(odom);
}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "edumip_my_horizontal_state_publisher");
	edumip_my_robot_state_publisher::StatePublisher state_publisher;

	ros::spin();
	return 0;
}

/** Auther: hshi17 02/27/18
 *  package: edumip_my_robot
 **/

#include <ros/ros.h>
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
	void timerCallback(const ros::TimerEvent&);

	ros::NodeHandle node;
	sensor_msgs::JointState joint_state;

	ros::Subscriber state_sub;
	ros::Publisher joint_pub;
	ros::Timer timer;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	std::string tf_prefix_;

	// publish odometry message
	ros::Publisher odom_pub;
	nav_msgs::Odometry odom;
};

StatePublisher::StatePublisher() {
	state_sub = node.subscribe("/edumip/state", 10, &StatePublisher::stateCallback, this);
	joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
	timer = node.createTimer(ros::Duration(0.1), &StatePublisher::timerCallback, this);
	odom_pub = node.advertise<nav_msgs::Odometry>("/edumip/odometry", 10);
}

void StatePublisher::stateCallback(const edumip_msgs::EduMipStateConstPtr& state) {

	transform.setOrigin(tf::Vector3(state->body_frame_northing, -1*state->body_frame_easting, 0.034/2));
	tf::Quaternion q;
	q.setRPY(0, 0, -state->body_frame_heading);
	transform.setRotation(q);

	std::string tf_prefix_key;
	ros::NodeHandle n_tilde("~");
	n_tilde.searchParam("tf_prefix", tf_prefix_key);
	n_tilde.param<std::string>(tf_prefix_key, tf_prefix_, std::string(""));

	tf::StampedTransform tf_stamped_trans = tf::StampedTransform(transform, ros::Time::now(), "world", "edumip_body");
	tf_stamped_trans.frame_id_ = tf::resolve(tf_prefix_, tf_stamped_trans.frame_id_);
	tf_stamped_trans.child_frame_id_ = tf::resolve(tf_prefix_, tf_stamped_trans.child_frame_id_);
	br.sendTransform(tf_stamped_trans);
	
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(2);
	joint_state.position.resize(2);
	joint_state.name[0] = "jointL";
	joint_state.position[0] = state->wheel_angle_L;
	joint_state.name[1] = "jointR";
	joint_state.position[1] = state->wheel_angle_R;

	// publish odometry message over ROS
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";
	// set the position
	tf::poseTFToMsg(transform, odom.pose.pose);
}

void StatePublisher::timerCallback(const ros::TimerEvent&) {
	joint_pub.publish(joint_state);
	odom_pub.publish(odom);
}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "edumip_my_state_publisher");
	edumip_my_robot_state_publisher::StatePublisher state_publisher;

	ros::spin();
	return 0;
}
/** Auther: hshi17 02/18/18
 *  package: joy_twist
 **/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class JoyTwist {
  public:
	// constructor
  	JoyTwist();
  private:
	// callback functions
  	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void timerCallback(const ros::TimerEvent&);

	ros::NodeHandle node;
	geometry_msgs::Twist twist;

	// velocity
	int linear_, angular_;
	// input signal amplify parameters
	double l_scale_, a_scale_;

	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Timer timer;
};

// initialization
JoyTwist::JoyTwist():
 linear_(1), angular_(0), l_scale_(1), a_scale_(1)
 {
	// read default parameters from parameter server
 	node.param("axis_linear", linear_, linear_);
 	node.param("axis_angular", angular_, angular_);
 	node.param("scale_angular", a_scale_, a_scale_);
 	node.param("scale_linear", l_scale_, l_scale_);

	pub = node.advertise<geometry_msgs::Twist>("/edumip/cmd",1);
	sub = node.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyTwist::joyCallback, this);
	timer = node.createTimer(ros::Duration(0.1), &JoyTwist::timerCallback, this);
}

void JoyTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	// genearte twist according to /joy topic message
	twist.angular.z = a_scale_*joy->axes[angular_];
	twist.linear.x = l_scale_*joy->axes[linear_];
}

void JoyTwist::timerCallback(const ros::TimerEvent&) {
	// publish velocity topic
	pub.publish(twist);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "joy_twist");
	JoyTwist joy_twist;

	ros::spin();
	return 0;
}

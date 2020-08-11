/* Author: Fernando González fergonzaramos@yahoo.es  */

#include "kobuki_mover/KobukiMover.hpp"
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>

namespace kobuki_mover
{

	KobukiMover::KobukiMover()
	: nh_("~")
	{
		// Set Node Params:

		initParams();

		// For Debug:

		if (DEBUG)
			printParams();

		// Set publishers and subscribers:

		vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);

		vel_setter_srv_ = nh_.advertiseService("/kobuki_vel_setter",
			&KobukiMover::velSetterCb, this);
	}

	bool
	KobukiMover::velSetterCb(ros_tutorials_msgs::kobuki_vel::Request & request,
		ros_tutorials_msgs::kobuki_vel::Response & response)
	{
		v_ = request.linear_vel;
		w_ = request.angular_vel;

		response.linear_vel = v_;
		response.angular_vel = w_;

		ROS_WARN("Spped changed! v: %f, w: %f\n", v_, w_);

		return true;
	}

	void
	KobukiMover::printParams()
	{
		ROS_WARN("Velocity Topic: %s\n", vel_topic_.c_str());
		ROS_WARN("Linear Velocity: %f\n", v_);
		ROS_WARN("Angular Velocity: %f\n", w_);
	}

	void
	KobukiMover::initParams()
	{
		// Default Values:

		vel_topic_ = "/mobile_base/commands/velocity";
		v_ = 0.0;
		w_ = M_PI / 5.0;

		// Get params from param server with default values:

		nh_.param("vel_topic", vel_topic_, vel_topic_);
		nh_.param("linear_vel", v_, v_);
		nh_.param("angular_vel", w_, w_);
	}

	void
	KobukiMover::update()
	{
		geometry_msgs::Twist msg;

		msg.linear.x = v_;
		msg.angular.z = w_;

		vel_pub_.publish(msg);

		ROS_INFO("Step!\n");
	}

};	// namespace kobuki_mover

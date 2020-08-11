/* Author: Fernando González fergonzaramos@yahoo.es  */

#ifndef KOBUKI_MOVER__KOBUKI_MOVER_H_
#define KOBUKI_MOVER__KOBUKI_MOVER_H_

#include <string>
#include <ros/ros.h>
#include <ros_tutorials_msgs/kobuki_vel.h>

namespace kobuki_mover
{

#define DEBUG 1

class KobukiMover
{
public:

	KobukiMover();

	void update();

private:

	void initParams();
	void printParams();

	bool velSetterCb(ros_tutorials_msgs::kobuki_vel::Request & request,
		ros_tutorials_msgs::kobuki_vel::Response & response);

	ros::NodeHandle nh_;

	ros::Publisher vel_pub_;
	ros::ServiceServer vel_setter_srv_;

	std::string vel_topic_;
	double v_, w_;

};
};	// namespace kobuki_mover

#endif	// KOBUKI_MOVER__KOBUKI_MOVER_H_

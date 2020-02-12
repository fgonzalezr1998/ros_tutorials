#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>

class Pub
{

public:
	Pub()
	{
		vel_topic_ = "/mobile_base/commands/velocity";
		vel_ = Pi / 4.0;
		pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);
	}

	void
	update()
	{
		geometry_msgs::Twist msg;
		msg.angular.z = vel_;
		pub_.publish(msg);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	std::string vel_topic_;
	float vel_;
	float Pi = 3.1416;

};

int
main(int argc, char **argv)
{

	ros::init(argc, argv, "kobuki_vel_publisher_node");

	Pub pub;
	ros::Rate rate(10); //10 Hz
	while(ros::ok())
	{
		pub.update();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

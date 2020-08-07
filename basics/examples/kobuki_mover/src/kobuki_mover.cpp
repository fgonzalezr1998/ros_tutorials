#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>

#define HZ 5
#define DEBUG 1

class KobukiMover
{
public:
	KobukiMover()
	:	nh_("~")
	{
		// Set Node Params:

		initParams();

		//For Debug:

		if(DEBUG)
			printParams();

		// Set publishers and subscribers:

		vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);
	}

	void
	update()
	{
		geometry_msgs::Twist msg;

		msg.linear.x = v_;
		msg.angular.z = w_;

		vel_pub_.publish(msg);

		ROS_INFO("Step!\n");
	}

private:

	void
	printParams()
	{
		ROS_WARN("Velocity Topic: %s\n", vel_topic_.c_str());
		ROS_WARN("Linear Velocity: %f\n", v_);
		ROS_WARN("Angular Velocity: %f\n", w_);
	}

	void
	initParams()
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

	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	std::string vel_topic_;
	double v_, w_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "kobuki_mover_node");

	KobukiMover km;

	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		km.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

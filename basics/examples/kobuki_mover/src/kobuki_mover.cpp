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
	{
		// Set Node Params:

		init_params();

		//For Debug
		if(DEBUG){
			print_params();
		}

		// Set publishers and subscribers:

		vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);
		bumper_sub_ = nh_.subscribe(bumper_topic_, 1, &KobukiMover::bumperCallback, this);
	}

	void
	update()
	{
		ROS_INFO("Step!\n");
	}

private:

	void
	bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr & msg)
	{
		;
	}

	void
	print_params()
	{
		ROS_WARN("Velocity Topic: %s\n", vel_topic_.c_str());
		ROS_WARN("Bumper Topic: %s\n", bumper_topic_.c_str());
		ROS_WARN("Linear Velocity: %f\n", v_);
		ROS_WARN("Angular Velocity: %f\n", w_);
	}

	void
	init_params()
	{
		// Default Values:

		vel_topic_ = "/mobile_base/commands/velocity";
		bumper_topic_ = "/mobile_base/events/bumper";
		v_ = 0.5f;
		w_ = M_PI / 5.0;

		// Get params from param server with default values:

		nh_.param("vel_topic", vel_topic_, vel_topic_);
		nh_.param("bumper_topic", bumper_topic_, bumper_topic_);
		nh_.param("linear_vel", v_, v_);
		nh_.param("angular_vel", v_, v_);
	}

	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber bumper_sub_;
	std::string vel_topic_, bumper_topic_;
	float v_, w_;
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

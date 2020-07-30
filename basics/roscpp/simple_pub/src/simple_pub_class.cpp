#include <ros/ros.h>
#include <std_msgs/String.h>

class Publisher
{
public:

	Publisher()
	{
		ROS_INFO("%s\n", "I'm The Publisher Object");

		pub_ = nh_.advertise<std_msgs::String>("/talk", 1);
	}

	void
	doWork()
	{
		std_msgs::String msg;
		msg.data = "Hello World";

		pub_.publish(msg);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_pub_class_node");

	Publisher pub;

	//2Hz rate

	ros::Rate loop_rate(2);

	while(ros::ok()){
		pub.doWork();
		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

#include <ros/ros.h>
#include <std_msgs/String.h>

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_pub_node");

	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::String>("/talk", 1);

	//2Hz rate

	ros::Rate loop_rate(2);

	std_msgs::String msg;
	msg.data = "Hello World";

	while(ros::ok()){
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

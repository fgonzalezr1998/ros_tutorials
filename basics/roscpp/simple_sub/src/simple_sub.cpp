#include <ros/ros.h>
#include <std_msgs/Int32.h>

void
callback(const std_msgs::Int32::ConstPtr & msg)
{
	ROS_INFO("I received: %d\n", msg->data);
}

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_sub_node");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/talk", 1, callback);

	ros::spin();

exit(EXIT_SUCCESS);
}

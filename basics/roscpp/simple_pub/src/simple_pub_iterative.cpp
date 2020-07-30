#include <ros/ros.h>
#include <std_msgs/Int32.h>

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_pub_iterative_node");

	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int32>("/talk", 1);

	//2Hz rate

	ros::Rate loop_rate(2);

	std_msgs::Int32 count;
	count.data = 0;

	while(ros::ok()){
		pub.publish(count);
		ros::spinOnce();
		count.data++;
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

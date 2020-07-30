#include <ros/ros.h>
#include <std_msgs/Int32.h>

class Publisher
{
public:

	Publisher()
	{
		ROS_INFO("%s\n", "I'm The Publisher Object");

		count_.data = 0;
		pub_ = nh_.advertise<std_msgs::Int32>("/talk", 1);
	}

	void
	doWork()
	{
		pub_.publish(count_);
		count_.data++;
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	std_msgs::Int32 count_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_pub_class_iterative_node");

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

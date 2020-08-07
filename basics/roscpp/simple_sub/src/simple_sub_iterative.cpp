#include <ros/ros.h>
#include <std_msgs/Int32.h>

class Subscriber
{
public:

	Subscriber()
	:	received_(false)
	{
		ROS_INFO("%s\n", "I'm the Subscriber");
		sub_ = nh_.subscribe("/talk", 1, &Subscriber::callback, this);
	}

	void
	doWork()
	{
		if(!received_)
			return;

		received_ = false;
		ROS_INFO("I received: %d\n", recv_msg_.data);
	}

private:

	void
	callback(const std_msgs::Int32::ConstPtr & msg)
	{
		recv_msg_ = *msg;
		received_ = true;
	}

	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	std_msgs::Int32 recv_msg_;
	bool received_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_sub_iterative_node");

	Subscriber sub;

	//¿Que pasa si le ponemos 1Hz?

	ros::Rate loop_rate(2);
	while(ros::ok()){
		sub.doWork();
		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

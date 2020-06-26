#include <ros/ros.h>

class TFTransformer
{
public:
	TFTransformer()
	{
		ROS_INFO("%s\n", "Hello World");
	}

private:

	ros::NodeHandle nh_;
	ros::Subscriber poitcloud_sub_;

};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "point_tf_transformer_node");

	TFTransformer transformer;
	ros::Rate loop_rate(5);		//5Hz

	while(ros::ok())
	{
		ROS_INFO("%s\n", "Step!");
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

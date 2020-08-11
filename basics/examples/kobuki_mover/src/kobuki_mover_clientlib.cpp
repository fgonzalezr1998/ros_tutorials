#include <ros/ros.h>
#include "kobuki_mover/KobukiMover.hpp"

#define HZ 10

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "kobuki_mover_clientlib_node");

	kobuki_mover::KobukiMover km;

	ros::Rate loop_rate(HZ);
	while (ros::ok()) {
		km.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

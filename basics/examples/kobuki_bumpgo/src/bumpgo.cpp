#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0

class BumpGo
{
public:
  BumpGo()
	: state_(GOING_FORWARD), pressed_(false)
  {
    bumper_sub_ = nh_.subscribe("/mobile_base/events/bumper", 1, &BumpGo::bumperCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr & msg)
  {
    pressed_ = msg->state;
  }

  void step()
  {
    geometry_msgs::Twist cmd;

    switch (state_) {

	    case GOING_FORWARD:
	      cmd.linear.x = 0.15;
	      cmd.angular.z = 0.0;
	      if (pressed_) {
	        press_ts_ = ros::Time::now();
	        state_ = GOING_BACK;
	        ROS_INFO("GOING_FORWARD -> GOING_BACK");
	      }
	      break;

	    case GOING_BACK:
	      cmd.linear.x = -0.1;
	      cmd.angular.z = 0.0;

	      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME ) {
	        turn_ts_ = ros::Time::now();
	        state_ = TURNING;
	        ROS_INFO("GOING_BACK -> TURNING");
	      }
	      break;

	    case TURNING:
	      cmd.linear.x = 0.0;
	      cmd.angular.z = -0.3;
	      if ((ros::Time::now() - turn_ts_).toSec() > TURNING_TIME ) {
	        state_ = GOING_FORWARD;
	        ROS_INFO("TURNING -> GOING_FORWARD");
	      }
	      break;
    }

    vel_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING = 2;

  int state_;

  bool pressed_;

  ros::Time press_ts_;
  ros::Time turn_ts_;

  ros::Subscriber bumper_sub_;
  ros::Publisher vel_pub_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bumpgo_node");

  BumpGo bumpgo;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    bumpgo.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

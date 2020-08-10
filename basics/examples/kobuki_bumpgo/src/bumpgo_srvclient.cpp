#include <err.h>
#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include "ros_tutorials_msgs/kobuki_vel.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0

class BumpGo
{
public:
  BumpGo()
	: o_state_(TURNING), n_state_(GOING_FORWARD), pressed_(false),
    v_(0.08), w_(0.2)
  {
    bumper_sub_ = nh_.subscribe("/mobile_base/events/bumper", 1, &BumpGo::bumperCallback, this);
    srv_client_ = nh_.serviceClient<ros_tutorials_msgs::kobuki_vel>("/kobuki_vel_setter");
  }

  void
  bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr & msg)
  {
    pressed_ = msg->state;
  }

  void
  step()
  {
    int st = n_state_;

    if(o_state_ != n_state_)
      sendSpeed_(n_state_);

    switch (n_state_) {

	    case GOING_FORWARD:
	      if (pressed_) {
	        press_ts_ = ros::Time::now();
	        n_state_ = GOING_BACK;
	        ROS_WARN("GOING_FORWARD -> GOING_BACK");
	      }
	      break;

	    case GOING_BACK:
	      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME ) {
	        turn_ts_ = ros::Time::now();
	        n_state_ = TURNING;
	        ROS_WARN("GOING_BACK -> TURNING");
	      }
	      break;

	    case TURNING:
	      if ((ros::Time::now() - turn_ts_).toSec() > TURNING_TIME ) {
	        n_state_ = GOING_FORWARD;
	        ROS_WARN("TURNING -> GOING_FORWARD");
	      }
	      break;
    }

    o_state_ = st;
  }

private:

  void
  sendSpeed_(int st)
  {
    // Call to service

    ros_tutorials_msgs::kobuki_vel srv;
    switch (st) {
      case GOING_FORWARD:
        srv.request.linear_vel = v_;
        srv.request.angular_vel = 0.0;
        break;

      case GOING_BACK:
        srv.request.linear_vel = -v_;
        srv.request.angular_vel = 0.0;
        break;

      case TURNING:
        srv.request.linear_vel = 0.0;
        srv.request.angular_vel = w_;
        break;
    }

    if (! srv_client_.call(srv)) {
      ROS_ERROR("Failed to call service kobuki_vel");
      errx(EXIT_FAILURE, "%s\n", "Failed to call service kobuki_vel");
    }

    // Print response:

    ROS_INFO("Speed setted succesfully: %f, %f", srv.response.linear_vel,
      srv.response.angular_vel);
  }

  ros::NodeHandle nh_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING = 2;

  int n_state_, o_state_;  // Declare new and old states:
  double v_, w_;
  bool pressed_;

  ros::Time press_ts_;
  ros::Time turn_ts_;

  ros::Subscriber bumper_sub_;
  ros::ServiceClient srv_client_;
};


int
main(int argc, char **argv)
{
  ros::init(argc, argv, "bumpgo_node");

  BumpGo bumpgo;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    bumpgo.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

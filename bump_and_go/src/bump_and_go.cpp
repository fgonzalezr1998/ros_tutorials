#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>

class BumpGo
{
public:
  BumpGo():
    nh_("~")
  {
        bumper_ = nh_.subscribe("/mobile_base/events/bumper", 1, &BumpGo::callback, this);
        state_ = 0;
        bumper_pos_ = 0;
  }

  void callback(kobuki_msgs::BumperEvent::ConstPtr& bump)
  {
    /*
    bumper_pos_ = bump->bumper;
    state_ = bump->state;
    */
    ;
  }

  void step()
  {
    /*
    ROS_INFO("Bumper: %d\n", bumper_pos_);
    ROS_INFO("State: %d\n", state_);
    */
    ;
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher vel_;
  ros::Subscriber bumper_;
  uint8_t state_, bumper_pos_;

};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "kobuki_bumper_and_go");

  BumpGo bg;
  ros::Rate rate (10);
  while(ros::ok())
  {
    bg.step();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

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
        vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        state_ = 0;
        bumper_pos_ = 0;
  }

  void callback(const kobuki_msgs::BumperEvent::ConstPtr& bump)
  {

    bumper_pos_ = bump->bumper;
    state_ = bump->state;
  }

  void step()
  {
    float w;
    float v;
    if (state_ == 0)
    {
        v = 0.13;
        w = 0;
    }else{
        v = -0.12;
        if (bumper_pos_ == 0 || bumper_pos_ == 1)
        {
          w = -pi/6;
        }
        else
        {
          w = pi/6;
        }
    }

    geometry_msgs::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    vel_.publish(msg);
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher vel_;
  ros::Subscriber bumper_;
  uint8_t state_, bumper_pos_;
  float pi = 3.1416;

};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "kobuki_bump_and_go");

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

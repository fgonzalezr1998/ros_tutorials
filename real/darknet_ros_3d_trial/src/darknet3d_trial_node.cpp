#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <vector>

#define HZ 5

class Darknet3DTrial
{
public:
  Darknet3DTrial()
  {
    darknet3d_sub_ = nh_.subscribe(
      "/darknet_ros_3d/bounding_boxes", 1, &Darknet3DTrial::darknet3dCb, this);
  }

  void
  step()
  {
    double center_x, center_y, center_z;

    for(auto bbx : bboxes_)
    {
      // Calculate certer point in 3D coordinates:

      center_x = (bbx.xmin + bbx.xmax) / 2.0;
      center_y = (bbx.ymin + bbx.ymax) / 2.0;
      center_z = (bbx.zmin + bbx.zmax) / 2.0;

      ROS_INFO("%s: (%lf, %lf, %lf)\n", bbx.Class.c_str(), center_x, center_y, center_z);

      // If you want, you can publish these values for other node can use it

    }
    ROS_INFO("---------------------------");
  }

private:
  void
  darknet3dCb(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg)
  {
    // This callback, only save the 3D bounding boxes in 'bboxes_' variable:
    
    bboxes_ = msg->bounding_boxes;
  }

  ros::NodeHandle nh_;
  ros::Subscriber darknet3d_sub_;
  std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> bboxes_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "darknet3d_trial_node");

  Darknet3DTrial darknet3d_trial;

  // Configure the loop frequency (Hertzios):

  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ros::spinOnce();
    darknet3d_trial.step();
    loop_rate.sleep();
  }

  return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class TFTransformer
{
public:
	TFTransformer():
		tf_listener_(tf_buffer_), pc_received_(false)
	{
		pointcloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &TFTransformer::pointcloudCb, this);
	}

	void
	step()
	{
		geometry_msgs::TransformStamped transform;
		sensor_msgs::PointCloud2 cloud_out;

		if(! pc_received_)
			return;
			
		try
		{
			transform = tf_buffer_.lookupTransform("camera_link", original_cloud_.header.frame_id,
	        													original_cloud_.header.stamp + ros::Duration(2.0), ros::Duration(2.0));

			tf2::doTransform(original_cloud_, cloud_out, transform);
		}catch(tf2::TransformException& ex){
			ROS_WARN("%s", ex.what());
      return;
		}
		ROS_INFO("%s\n", "Everithing was OK!");
	}

private:

	void
	pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		original_cloud_ = *msg;
		pc_received_ = true;
	}

	ros::NodeHandle nh_;
	ros::Subscriber pointcloud_sub_;
	tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
	sensor_msgs::PointCloud2 original_cloud_;
	bool pc_received_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "point_tf_transformer_node");

	TFTransformer transformer;
	ros::Rate loop_rate(5);		//5Hz

	while(ros::ok())
	{
		transformer.step();
		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(EXIT_SUCCESS);
}

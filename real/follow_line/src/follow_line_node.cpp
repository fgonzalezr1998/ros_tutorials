#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <string>

class RGBSub
{
public:

	RGBSub()
	{
		img_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &RGBSub::imgcb, this);
	}


	void
	imgcb(const sensor_msgs::Image::ConstPtr& msg)
	{
		enc_ = msg->encoding;
		cv_bridge::CvImagePtr cv_ptr;
		cv::Mat my_img;
		cv::Mat img_hsv;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc_);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		my_img = cv_ptr->image;
		//bgr to hsv
		cv::cvtColor(my_img, img_hsv, CV_BGR2HSV);
		// APLICAR FILTRO DE COLOR


		cv::imshow("OPENCV_WINDOW", img_hsv);
		cv::waitKey(1);
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber img_sub_;
	std::string enc_;
};

int
main(int argc, char **argv)
{

	ros::init(argc, argv, "follow_line_node");

	RGBSub sub;

	ros::spin();

	return 0;
}

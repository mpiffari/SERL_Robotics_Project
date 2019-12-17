#include "arrow_finder_computation.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>

ArrowFinder finder;

ros::CallbackQueue *callback_queue;
ros::Subscriber img_info_sub;
ros::Subscriber img_sub;

int image_height, image_width;
bool info_received = false;

// ================== CALLBACK FUNCTIONS ==================
void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
	sensor_msgs::CameraInfo info_msg = *msg;
	if(! info_received) {
		image_height = info_msg.height;
		image_width = info_msg.width;
		info_received = true;
	}
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	sensor_msgs::Image img_msg = *msg;
  	try {
		finder.setImage(cv_bridge::toCvShare(msg, "bgr8")->image, image_height, image_width);
  	} catch (cv_bridge::Exception& e) {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}  
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "arrow_finder_main");
  	ros::NodeHandle nh;

	// Create the callback queue;
	callback_queue = new ros::CallbackQueue();

	// Camera image info subscriber
	ros::SubscribeOptions ops_info = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(
					"/camera/camera_info", // Tpic name
					10, // Queue length
					infoCallback, // Callback function called when informations of images are available
					ros::VoidPtr(), // Tracked object, we don't need one thus NULL
					callback_queue // Pointer to callback queue object
			);
	img_info_sub = nh.subscribe(ops_info);
	img_info_sub = nh.subscribe("/camera/camera_info", 10, infoCallback);

	// Camera Image subscriber
	ros::SubscribeOptions ops_img = ros::SubscribeOptions::create<sensor_msgs::Image>(
					"/camera/image_raw_throttle", // Topic name
					10, // Queue length
					imageCallback, // Callback function called when an image is available
					ros::VoidPtr(), // Tracked object, we don't need one thus NULL
					callback_queue // Pointer to callback queue object
			);
	img_sub = nh.subscribe(ops_img);
	img_sub = nh.subscribe("/camera/image_raw_throttle", 10, imageCallback);

	ros::spin();
  return 0;
}
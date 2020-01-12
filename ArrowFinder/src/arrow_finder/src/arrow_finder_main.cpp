#include "arrow_finder_computation.hpp"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include "/home/serl/SERL_Project/Eigen_library/Eigen/Dense"

ArrowFinder finder(asin(1.75/3.5),0.83);

//int image_height, image_width;
ros::CallbackQueue *callback_queue;
ros::Subscriber info_sub;
ros::Subscriber img_sub;
bool info_received = true;

void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
	sensor_msgs::CameraInfo info_msg = *msg;
 	
	if(info_received) {
		info_received = false;

		float f_x = msg->K[0];
        float f_y = msg->K[4];
        float c_x = msg->K[2];
        float c_y = msg->K[5];
		finder.setCameraIntrinsicParameters(f_x,f_y,c_x,c_y);
	}
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	sensor_msgs::Image img_msg = *msg;
  	try {
			list<arrow_info> arrows = finder.findArrows(cv_bridge::toCvShare(msg, "bgr8")->image);
			const arrow_info* biggestArrow = finder.getBiggestArrow(arrows);
			if(biggestArrow != nullptr){
				VectorXf result(3);
				result = finder.worldCoordinates(biggestArrow);
				cout << "Orientamento della freccia rispetto all'asse U [deg] " << biggestArrow->orientation << endl;
				cout << "Posizione della freccia rispetto alla base [m]: [U,V,W]\n"<< result << endl;
			}
  	} catch (cv_bridge::Exception& e) {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "spot_finder_main");
  	ros::NodeHandle nh;

	//create the callback queue;
	callback_queue = new ros::CallbackQueue();

	// Camera Info subscriber
	ros::SubscribeOptions ops_info = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(
					"/camera/camera_info", // topic name
					10, // queue length
					infoCallback, // callback
					ros::VoidPtr(), // tracked object, we don't need one thus NULL
					callback_queue // pointer to callback queue object
			);
	info_sub = nh.subscribe(ops_info);
	info_sub = nh.subscribe("/camera/camera_info", 10, infoCallback);

	// Camera Image subscriber
	ros::SubscribeOptions ops_img = ros::SubscribeOptions::create<sensor_msgs::Image>(
					"/camera/image_raw", // topic name
					10, // queue length
					imageCallback, // callback
					ros::VoidPtr(), // tracked object, we don't need one thus NULL
					callback_queue // pointer to callback queue object
			);
	img_sub = nh.subscribe(ops_img);
	img_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);
	ros::spin();
  return 0;
}

/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include <usb_cam/RGBCamera.hpp>
#include <usb_cam/Image.hpp>
#include "usb_cam/usb_cam.hpp"
#include "usb_cam/rgb_colors.hpp"

#include <sstream>

namespace usb_cam {

class UsbCamNode
{
public:
	// private ROS node handle
	ros::NodeHandle node_;
	sensor_msgs::Image img_;
	image_transport::CameraPublisher image_pub_;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;


	// shared image message
	star::RGBCamera rgb_camera;
	star::Image star_image;

	cv::Mat g_last_image;

	UsbCamNode() :
		node_("~")
	{
		rgb_camera.setup();

		//  ###########################################################################
		//  #####################  ROS specific  ######################################
		//  ###########################################################################

		// load the camera info
//		img_.header.frame_id = std::string("usb_cam");
//		camera_name_ = std::string("head_camera");
//		camera_info_url_ = std::string("");
//	    video_device_name_ = std::string("/dev/video0");
//		int framerate_ = 30;

		// advertise the main image topic
		image_transport::ImageTransport it(node_);
		image_pub_ = it.advertiseCamera("image_raw", 1);

		cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, std::string("head_camera"), std::string("")));

		// check for default camera info
		if (!cinfo_->isCalibrated())
		{
			cinfo_->setCameraName(std::string("/dev/video0"));
			sensor_msgs::CameraInfo camera_info;
			camera_info.header.frame_id = std::string("usb_cam");
			camera_info.width = 640;
			camera_info.height = 480;
			cinfo_->setCameraInfo(camera_info);
		}
	}

	virtual ~UsbCamNode()
	{
		rgb_camera.shutdown();
	}

	bool take_and_send_image()
	{
		rgb_camera.grabImage(star_image);


		/* **********************************
		 * *** publish the image with ROS ***
		 * **********************************/

		// stamp the image
		img_.header.stamp = ros::Time::now();
		// fill the info
		img_.encoding = star_image.encoding;
		img_.height = star_image.height;
		img_.width = star_image.width;
		img_.step = star_image.step;

		img_.data.clear();
		for(unsigned int i=0; i < star_image.height*star_image.step; i++)
			img_.data.push_back(star_image.data[i]);

		img_.is_bigendian = star_image.is_bigendian;

		// grab the camera info
		sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
		ci->header.frame_id = std::string("usb_cam");
		ci->header.stamp = img_.header.stamp;

		std::cout << "########################" << std::endl;
		std::cout << "width = " << img_.width << std::endl;
		std::cout << "height = " << img_.height << std::endl;
		std::cout << "encoding = " << img_.encoding << std::endl;
		std::cout << "size = " << img_.data.size() << std::endl;

		// publish the image
		image_pub_.publish(img_, *ci);


		/* **********************************
		 * *** show the image with OpneCV ***
		 * **********************************/
/*
 	 	// Versione STAR: Funziona, ma i colori sono invertiti
		int type;
		if (star_image.encoding == sensor_msgs::image_encodings::BGR8)   type = CV_8UC3;
		if (star_image.encoding == sensor_msgs::image_encodings::MONO8)  type = CV_8UC1;
		if (star_image.encoding == sensor_msgs::image_encodings::RGB8)   type = CV_8UC3;
		if (star_image.encoding == sensor_msgs::image_encodings::MONO16) type = CV_16UC1;
		if (img_.encoding == sensor_msgs::image_encodings::BGRA8)  type = CV_8UC4;
		if (star_image.encoding == sensor_msgs::image_encodings::RGBA8)  type = CV_8UC4;

		type = CV_8UC3;

		const cv::Mat &cv_image = cv::Mat(star_image.height, star_image.width, type, const_cast<uchar*>(&star_image.data[0]), star_image.step);

		cv::imshow("USB_CAM", cv_image);
		cv::waitKey(1);
*/
/*
 	 	// Versione ROS: Funziona, ma i colori sono invertiti
		int type;
		if (img_.encoding == sensor_msgs::image_encodings::BGR8)   type = CV_8UC3;
		if (img_.encoding == sensor_msgs::image_encodings::MONO8)  type = CV_8UC1;
		if (img_.encoding == sensor_msgs::image_encodings::RGB8)   type = CV_8UC3;
		if (img_.encoding == sensor_msgs::image_encodings::MONO16) type = CV_16UC1;
		if (img_.encoding == sensor_msgs::image_encodings::BGRA8)  type = CV_8UC4;
		if (img_.encoding == sensor_msgs::image_encodings::RGBA8)  type = CV_8UC4;

		const cv::Mat &cv_image = cv::Mat(img_.height, img_.width, type, const_cast<uchar*>(&img_.data[0]), img_.step);

		cv::imshow("USB_CAM", cv_image);
		cv::waitKey(1);
*/

/*
		// Non funziona

		cv::Mat colored_image = cv::Mat(img_.height, img_.width, type);
		int label;
		for (size_t j = 0; j < img_.height; ++j) {
			for (size_t i = 0; i < img_.width; ++i) {
				label = image.at<int>(j, i);
				cv::Vec3d rgb = rgb_colors::getRGBColor(label);
				// result image should be BGR
				colored_image.at<cv::Vec3b>(j, i) = cv::Vec3b(int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255));
			}
		}
		cv::imshow("USB_CAM", colored_image);
		cv::waitKey(1);
*/

/*
		const sensor_msgs::ImageConstPtr& source = &img_;
		// ------------------------------------------------
		// Convert to OpenCV native BGR color
		cv_bridge::CvImageConstPtr cv_ptr;

		try {
			cv_bridge::CvtColorForDisplayOptions options;
			options.do_dynamic_scaling = true;
			options.colormap = 0; // ???
			// Set min/max value for scaling to visualize depth/float image.
			options.min_image_value = 0;
			if (img_.encoding == "32FC1") {
				options.max_image_value = 10;  // 10 [m]
			} else if (img_.encoding == "16UC1") {
				options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
			}

			cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(&source), "", options);


			g_last_image = cv_ptr->image;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",
					img_.encoding.c_str(), e.what());
		}
		if (!g_last_image.empty()) {
			const cv::Mat &image = g_last_image;
			cv::imshow("USB_CAM", image);
			cv::waitKey(1);
		}
*/

		return true;
	}

	bool spin()
	{
		ros::Rate loop_rate(30);
		while (node_.ok())
		{
			if (rgb_camera.is_capturing()) {
				if (!take_and_send_image()) printf("USB camera did not respond in time.");
			}

			ros::spinOnce();
			loop_rate.sleep();

		}
		return true;
	}

};

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "usb_cam");
	usb_cam::UsbCamNode a;
	a.spin();
	return EXIT_SUCCESS;
}

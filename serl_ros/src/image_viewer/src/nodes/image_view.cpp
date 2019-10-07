/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#include <image_view/ImageViewConfig.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

int g_count;
cv::Mat g_last_image;
boost::format g_filename_format;
boost::mutex g_image_mutex;
std::string g_window_name;
bool g_gui;
ros::Publisher g_pub;
bool g_do_dynamic_scaling;
int g_colormap;
double g_min_image_value;
double g_max_image_value;

void reconfigureCb(image_view::ImageViewConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(g_image_mutex);
  g_do_dynamic_scaling = config.do_dynamic_scaling;
  g_colormap = config.colormap;
  g_min_image_value = config.min_image_value;
  g_max_image_value = config.max_image_value;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(g_image_mutex);

  // Convert to OpenCV native BGR color
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = g_do_dynamic_scaling;
    options.colormap = g_colormap;
    // Set min/max value for scaling to visualize depth/float image.
    if (g_min_image_value == g_max_image_value) {
      // Not specified by rosparam, then set default value.
      // Because of current sensor limitation, we use 10m as default of max range of depth
      // with consistency to the configuration in rqt_image_view.
      options.min_image_value = 0;
      if (msg->encoding == "32FC1") {
        options.max_image_value = 10;  // 10 [m]
      } else if (msg->encoding == "16UC1") {
        options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
      }
    } else {
      options.min_image_value = g_min_image_value;
      options.max_image_value = g_max_image_value;
    }
    cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
    g_last_image = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",
                       msg->encoding.c_str(), e.what());
  }
  if (g_gui && !g_last_image.empty()) {
    const cv::Mat &image = g_last_image;
    cv::imshow(g_window_name, image);
    cv::waitKey(1);
  }
  if (g_pub.getNumSubscribers() > 0) {
    g_pub.publish(cv_ptr);
  }
}

static void mouseCb(int event, int x, int y, int flags, void* param)
{
  if (event == cv::EVENT_LBUTTONDOWN) {
    ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    return;
  } else if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }

  boost::mutex::scoped_lock lock(g_image_mutex);

  const cv::Mat &image = g_last_image;

  if (image.empty()) {
    ROS_WARN("Couldn't save image, no data!");
    return;
  }

  std::string filename = (g_filename_format % g_count).str();
  if (cv::imwrite(filename, image)) {
    ROS_INFO("Saved image %s", filename.c_str());
    g_count++;
  } else {
    boost::filesystem::path full_path = boost::filesystem::complete(filename);
    ROS_ERROR_STREAM("Failed to save image. Have permission to write there?: " << full_path);
  }
}

static void guiCb(const ros::WallTimerEvent&)
{
    // Process pending GUI events and return immediately
    cv::waitKey(1);
}

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("HELLO IT'S ME");
  ROS_INFO(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("usb_camera/image_raw", 1000, chatterCallback);
  ros::spin();
}

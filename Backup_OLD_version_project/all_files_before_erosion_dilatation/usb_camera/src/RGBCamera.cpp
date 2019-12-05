/********************************************************************************
 *
 * RGBCamera
 *
 * Copyright (c) 2019
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: RGBCamera.cpp
 * Created: October 5, 2019
 * Author: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 * -------------------------------------------------------------------------------
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************
 */

//#include "star/functionality/sensor_rgbcamera/RGBCamera.hpp"
#include "usb_cam/RGBCamera.hpp"

namespace star
{
/*
extern "C" BOOST_SYMBOL_EXPORT RGBCamera rgb_camera;
RGBCamera rgb_camera;
*/
RGBCamera::RGBCamera()
{
}

RGBCamera::~RGBCamera()
{
}
void RGBCamera::setup()
{
/*
	MonocularCameraInterface::setup();

	registerProperty(accuracy, "Accuracy");
*/

	// ##############################################
	// ############ Camera properties ###############
	// ##############################################

    // grab the parameters
    video_device_name_ = std::string("/dev/video0");

    image_width_ = 640;
    image_height_ = 480;
    framerate_ = 30;

    brightness_ = -1; 	//0-255, -1 "leave alone"
    contrast_ = -1;		//0-255, -1 "leave alone"
    saturation_ = -1;	//0-255, -1 "leave alone"
    sharpness_ = -1;	//0-255, -1 "leave alone"

    // possible values: mmap, read, userptr
    io_method_name_ = std::string("mmap");

    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    pixel_format_name_ = std::string("yuyv");

    // enable/disable autofocus
    autofocus_ = false;
    focus_ = -1;		//0-255, -1 "leave alone"

    // enable/disable autoexposure
    autoexposure_ = true;
    exposure_ = 100;
    gain_ = -1;			//0-100?, -1 "leave alone"

    // enable/disable auto white balance temperature
    auto_white_balance_ = true;
    white_balance_ = 4000;

/*
    // load the camera info
    img_.header.frame_id = std::string("usb_cam");
    camera_name_ = std::string("head_camera");
    camera_info_url_ = std::string("");
*/

    // set the IO method
    usb_cam::UsbCam::io_method io_method = usb_cam::UsbCam::io_method_from_string(io_method_name_);
    if(io_method == usb_cam::UsbCam::IO_METHOD_UNKNOWN)
    {
      printf("Unknown IO method '%s'", io_method_name_.c_str());
// TODO : shutdown
//      node_.shutdown();
//      return;
    }

    // set the pixel format
    usb_cam::UsbCam::pixel_format pixel_format = usb_cam::UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == usb_cam::UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      printf("Unknown pixel format '%s'", pixel_format_name_.c_str());
      // TODO : shutdown
      //      node_.shutdown();
      //      return;
    }

    // start the camera
    grabber.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
      grabber.set_v4l_parameter("brightness", brightness_);

    if (contrast_ >= 0)
      grabber.set_v4l_parameter("contrast", contrast_);

    if (saturation_ >= 0)
      grabber.set_v4l_parameter("saturation", saturation_);

    if (sharpness_ >= 0)
      grabber.set_v4l_parameter("sharpness", sharpness_);

    if (gain_ >= 0)
      grabber.set_v4l_parameter("gain", gain_);

    // check auto white balance
    if (auto_white_balance_)
      grabber.set_v4l_parameter("white_balance_temperature_auto", 1);
    else {
      grabber.set_v4l_parameter("white_balance_temperature_auto", 0);
      grabber.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_) {
      // turn down exposure control (from max of 3)
      grabber.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      grabber.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_) {
      grabber.set_auto_focus(1);
      grabber.set_v4l_parameter("focus_auto", 1);
    }
    else {
      grabber.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
        grabber.set_v4l_parameter("focus_absolute", focus_);
    }

	// ##############################################
	// ######### Camera Info properties #############
	// ##############################################




	// ##############################################
	// ############ Image properties ################
	// ##############################################

	// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html

	/* The distortion model used. Supported models are listed in
	 * sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
	 * simple model of radial and tangential distortion - is sufficient. */
	std::string distortion_model;

	/* The distortion parameters, size depending on the distortion model.
	 * For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3). */
	double D[5];

	/* Intrinsic camera matrix for the raw (distorted) images.
	 *     [fx  0 cx]
	 * K = [ 0 fy cy]
	 *     [ 0  0  1]
	 * Projects 3D points in the camera coordinate frame to 2D pixel
	 * coordinates using the focal lengths (fx, fy) and principal point
	 * (cx, cy). */
	double K[9];  // 3x3 row-major matrix

	/* Rectification matrix (stereo cameras only)
	 * A rotation matrix aligning the camera coordinate system to the ideal
	 * stereo image plane so that epipolar lines in both stereo images are
	 * parallel. */
	double R[9];  // 3x3 row-major matrix

	/* Projection/camera matrix
	 *     [fx'  0  cx' Tx]
	 * P = [ 0  fy' cy' Ty]
	 *     [ 0   0   1   0]
	 * By convention, this matrix specifies the intrinsic (camera) matrix
	 *  of the processed (rectified) image. That is, the left 3x3 portion
	 *  is the normal camera intrinsic matrix for the rectified image.
	 * It projects 3D points in the camera coordinate frame to 2D pixel
	 *  coordinates using the focal lengths (fx', fy') and principal point
	 *  (cx', cy') - these may differ from the values in K.
	 * For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
	 *  also have R = the identity and P[1:3,1:3] = K.
	 * For a stereo pair, the fourth column [Tx Ty 0]' is related to the
	 *  position of the optical center of the second camera in the first
	 *  camera's frame. We assume Tz = 0 so both cameras are in the same
	 *  stereo image plane. The first camera always has Tx = Ty = 0. For
	 *  the right (second) camera of a horizontal stereo pair, Ty = 0 and
	 *  Tx = -fx' * B, where B is the baseline between the cameras.
	 * Given a 3D point [X Y Z]', the projection (x, y) of the point onto
	 *  the rectified image is given by:
	 *  [u v w]' = P * [X Y Z 1]'
	 *         x = u / w
	 *         y = v / w
	 *  This holds for both images of a stereo pair. */
	double P[12];  // 3x4 row-major matrix

}

void RGBCamera::grabImage(Image &image) {
	grabber.grab_image(&image);



/*
	image.height = image_height;
	image.width = image_width;
	image.encoding = image_encoding;
	image.is_bigendian = is_bigendian;
	image.step = image_step;

	image.data = grabber.grab_image();
*/
}

bool RGBCamera::is_capturing() {
	return grabber.is_capturing();
}

void RGBCamera::shutdown() {
	grabber.shutdown();
}


} // namespace star

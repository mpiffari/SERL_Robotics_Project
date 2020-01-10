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
 * File: RGBCamera.hpp
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
#ifndef RGB_CAMERA_H
#define RGB_CAMERA_H
/*
#include <boost/config.hpp> // for BOOST_SYMBOL_EXPORT
#include <star/functionality/sensor_core/MonocularCameraInterface.hpp>
#include <star/functionality/sensor_core/image_encodings.hpp>
#include <star/functionality/sensor_core/Image.hpp>
#include <star/component_framework/VProperty.hpp>
*/
#include <usb_cam/Image.hpp>
#include <usb_cam/usb_cam.hpp>

namespace star {
//class RGBCamera : public MonocularCameraInterface {
class RGBCamera {
public:
	RGBCamera();

	~RGBCamera();

	void setup();

	bool is_capturing();

	void grabImage(Image &image);

	void shutdown();

private:
	//	VProperty<int> accuracy;	// [cm]

	// parameters
	std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
	bool streaming_status_;
	int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
	white_balance_, gain_;
	bool autofocus_, autoexposure_, auto_white_balance_;

	// shared image message
	star::Image img_;

	usb_cam::UsbCam grabber;
};

} // namespace star  

#endif

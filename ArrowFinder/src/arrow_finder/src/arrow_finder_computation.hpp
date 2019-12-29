/********************************************************************************
 *
 * Arrow finder computation
 *
 * Copyright (c) 2018
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: arrow_finder_computation.hpp
 * Created: September 2019
 * Author: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
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
 *******************************************************************************/
#ifndef ARROWFINDER
#define ARROWFINDER

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <list>
#include <iterator>
#include "/home/serl/SERL_Project/Eigen_library/Eigen/Dense"


// Parameters used to filter out noisily points in the original image
// [pixel^2] Lower threshold useful to reduce the presence of undesired figure
#define lower_area_rect         50 
#define lower_area_triang       5
// [pixel] Lower threshold useful to recognize square figure near rectangular one
#define lower_dst_rect_triang   50

using namespace Eigen;
using namespace std;
using namespace cv;

struct arrow_info {
	CvPoint center;  	// Computed center of the arrow
	float orientation;	// Orientation in degrees
	float area; 
};

struct composed_arrow_info {
    CvPoint center_triangle;
    CvPoint center_rectangle;
	float area; 
};

class ArrowFinder {
	public:
	    std::vector<cv::Point2f> output;	// Marker coordinates
	    ArrowFinder();
	    ~ArrowFinder();

	   // bool setup(const std::string &filename);

	    /*
	    	return: true if we find an image
	    */
	    void setImage(cv::Mat original_image, int image_height, int image_width);

		/*
		*	@return: a list containing all the arrows found into original_image
		*/
	    list<arrow_info> findArrows(cv::Mat image);

	    /*
	    * @return: the biggest arrow in the arrow_list
	    */
	    const arrow_info* getBiggestArrow( list<arrow_info> arrow_list);

	    // Convert from image coordiantes to world coordinates
	    VectorXf worldCoordinates(const arrow_info* arrow);


	    //TODO: result of computation will be pubblished with ROS throw a topic
	    //or keep indipendent from ROS

	private:
		Mat tinyRedFiltering(Mat &image_masked_red);
		void Erosion(Mat in, Mat &out);
		void Dilation(Mat in, Mat &out);

};

#endif

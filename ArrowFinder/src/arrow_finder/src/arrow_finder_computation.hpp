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



//Parameters used to filter out noisily points in the original image
//[pixel^2] Lower threshold useful to reduce the presence of undesired figure
#define lower_area_rect         50
#define lower_area_triang       5
#define lower_dst_rect_triang   50 //[pixel] Lower threshold used to recognize when a square and a triangular makes an arrow

//Erosion and dilation tuning parameters
#define max_elem        2
#define max_kernel_size 21


using namespace Eigen;
using namespace std;
using namespace cv;

struct arrow_info {
    CvPoint center;     // Computed center of the arrow
    float orientation;  // Orientation in degrees
    float area;
};

struct composed_arrow_info {
    CvPoint center_triangle;
    CvPoint center_rectangle;
    float area;
};

class ArrowFinder {
    private:
        // Images
        IplImage img2;
        IplImage* img3;
        
        // Red mask CSV threshold
        int MinH_R = 0;
        int MaxH_R = 10;
        int MinS_R = 70;
        int MaxS_R = 255;
        int MinV_R = 172;
        int MaxV_R = 255;

        // Blue mask CSV threshold
        int MinH_B = 100;
        int MaxH_B = 180;
        int MinS_B = 0;
        int MaxS_B = 255;
        int MinV_B = 65;
        int MaxV_B = 200;

        //Erosion and dilation parameters definition
        int erosion_elem = 0;
        int dilation_elem = 0;
        int erosion_size = 1; 	//Setting of erosion type MORPH_RECT
        int dilation_size = 3;	//Setting of dilation type MORPH_RECT

        MatrixXf R_traslation = MatrixXf(4,4);  		//Traslation of point acquired from ground position to camera height (see documentation)
        MatrixXf R_rot_cam_inclination = MatrixXf(4,4);	//Rotation of the point acquired of an angle equals to "cam_inclination" (see documentation)
        MatrixXf R_rot_camera = MatrixXf(4,4);      	//Rotation from the frame of the camera to the base ground (see documentation)
        VectorXf U_cam = VectorXf(2);               	//Coordinates of the point in the image frame
        VectorXf X_camera_normalized = VectorXf(3);		//Coordinates of the point in the normalized frame
        VectorXf X_camera = VectorXf(3);            	//Coordinates of the point in the camera frame
        VectorXf X_camera_augmented = VectorXf(4); 		//Useful for passage from 2D to 3D by adding "fictitious" third coordinate
        VectorXf X_World = VectorXf(4);

        const char* nameMainImageWindow   = "Field Of View";
        const char* erosionImageWindow    = "Erosion demo";
        const char* dilatationImageWindow = "Dilatation demo";

        bool showColorsThresholdTrackbar = false; //Flag to activate trackbar window for HSV threshold tuning
        bool showErosionTrackbar         = false; //Flag to activate trackbar window for erosion tuning
        bool showDilatationTrackbar      = false; //Flag to activate trackbar window for dilatation tuning

        // Camera tilt
        float cam_inclination;

        // Intrinsic parameters of the camera (focal and center position)
        float f_x;
        float f_y;
        float c_x;
        float c_y;
        float h_cam; // Height of the camera from the ground expressed in [m]
        float scale; // Scale factor used to detect deepness of the point

        //Image dimensions
        int image_height;
        int image_width;

    public:
        ArrowFinder(float cam_inclination=0, float cam_height=0);
        ~ArrowFinder();

        /*
        *   @return: a list containing all the arrows found into original_image
        */
        list<arrow_info> findArrows(cv::Mat image);

        /*
        * @return: the biggest arrow in the arrow_list
        */
        const arrow_info* getBiggestArrow( list<arrow_info> arrow_list);

        /*
        * @return: the world coordiantes from image coordinates
        */
        VectorXf worldCoordinates(const arrow_info* arrow);

        // Setup parameters
        void setCameraIntrinsicParameters(float f_x, float f_y, float c_x, float c_y);
        void setCameraPhysicalParameters(float cam_inclination, float cam_height);

    private:
        //Filter of tiny contour in image with red pixel
        Mat tinyRedFiltering(Mat &image_masked_red);
        void Erosion(Mat in, Mat &out);
        void Dilation(Mat in, Mat &out);
};

#endif

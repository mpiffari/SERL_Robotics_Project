/********************************************************************************
 *
 * StarSpotFinder
 *
 * Copyright (c) 2018
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: StarSpotFinder.hpp
 * Created: October 8, 2018
 * Author: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * This is a refactored version of the usb_cam library by Robert Bosch LLC (c) 2014
 * The dependencies to ROS have been removed
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

#include "StarSpotFinder.hpp"

#include <unistd.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <list>
#include<iterator>

#define PI 3.14159265

using namespace std;
using namespace cv;

int flag = false;
int start = 0;
int minT= 0, maxT = 255; // for black and white threshold
// Black mask
int MinH_b=0;
int MaxH_b=255;
int MinS_b=0;
int MaxS_b=193;
int MinV_b=0;
int MaxV_b=141;
// Red mask

float alpha_cam;	//Angolo asse telecamera rispetto alla verticale in gradi
float r;    	//Raggio marker

// Red mask
int MinH_R=0;
int MaxH_R=10;
int MinS_R=70;
int MaxS_R=255;
int MinV_R=172;
int MaxV_R=255;

// Blue mask
int MinH_B=100;
int MaxH_B=180;
int MinS_B=0;
int MaxS_B=255;
int MinV_B=65;
int MaxV_B=200;

int PARAM;

StarSpotFinder::StarSpotFinder() {
}

StarSpotFinder::~StarSpotFinder() {
}

void StarSpotFinder::setImage(cv::Mat image, int image_height, int image_width) {
        
	output.clear();


	//slider parametro perimetro
	namedWindow("Immagine acquisita");
	createTrackbar("Luminosità", "Immagine acquisita", &PARAM, 200);//luminosità

	//Settaggio trackbar per impostazione colore.
	const char* str_r = "HSV RED";
	namedWindow(str_r);
	createTrackbar("MinH red", str_r, &MinH_R, 255);
	createTrackbar("MaxH red", str_r, &MaxH_R, 255);
	createTrackbar("MinS red", str_r, &MinS_R, 255);
	createTrackbar("MaxS red", str_r, &MaxS_R, 255);
	createTrackbar("MinV red", str_r, &MinV_R, 255);
	createTrackbar("MaxV red", str_r, &MaxV_R, 255);

	const char* str_b = "HSV BLUE";
	namedWindow(str_b);
	createTrackbar("MinH blue", str_b, &MinH_B, 255);
	createTrackbar("MaxH blue", str_b, &MaxH_B, 255);
	createTrackbar("MinS blue", str_b, &MinS_B, 255);
	createTrackbar("MaxS blue", str_b, &MaxS_B, 255);
	createTrackbar("MinV blue", str_b, &MinV_B, 255);
	createTrackbar("MaxV blue", str_b, &MaxV_B, 255);

	
	//gaussian blur
	image.convertTo(image,-1,1,-PARAM);
	GaussianBlur(image, image, Size(3,3), 0);
	
	Mat hsv;
	Mat hsv_red, hsv_blue;
	Mat img_masked_red, img_masked_blue, img_masked_red_blue;


	cvtColor(image, hsv, CV_BGR2HSV);	//Converto in formato HSV

	inRange(hsv, Scalar(MinH_R, MinS_R, MinV_R), Scalar(MaxH_R, MaxS_R, MaxV_R), hsv_red);	//Range di colori d considerare - red
	inRange(hsv, Scalar(MinH_B, MinS_B, MinV_B), Scalar(MaxH_B, MaxS_B, MaxV_B), hsv_blue);	//Range di colori d considerare - blue
	
	//bitwise_or(hsv_red, hsv_red, hsv);
	image.copyTo(img_masked_red,hsv_red);
	image.copyTo(img_masked_blue,hsv_blue);
	bitwise_or(hsv_red, hsv_blue, hsv);
	image.copyTo(img_masked_red_blue,hsv);

	//imshow("image masked",img_masked);
	

	imshow(str_r,hsv_red);
	imshow(str_b,hsv_blue);
	//imshow("IMAGE MASKED BY RED AND BLUE", hsv);

	IplImage tmp1=img_masked_red_blue;
	IplImage* output = &tmp1;

 	IplImage tmp=img_masked_red;
	IplImage* img = &tmp;

	/*IplImage img2 = image;
	IplImage* img3 = &img2;*/

	/*IplImage tt = hsv;
	IplImage* hsv2 = &tt;

	//cvCopy(img,img,hsv2);
	//bitwise_or(image,image,image,hsv);
	//img = &(*img & *hsv2);
	//imshow("prova",image);

	/*namedWindow("image after mask");
	cvShowImage("image after mask",img);
*/

	//cvErode(img, img, 0, 2);
	//smooth the original image using Gaussian kernel to remove noise
	//cvSmooth(img, img, CV_GAUSSIAN,3,3);

	//converting the original image into grayscale
	IplImage* imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1); 
	cvCvtColor(img,imgGrayScale,CV_BGR2GRAY);

	//thresholding the grayscale image to get better results
	//cvThreshold(imgGrayScale,imgGrayScale,minT,maxT,CV_THRESH_BINARY_INV);

	CvSeq* contour;  //hold the pointer to a contour
	CvSeq* result;   //hold sequence of points of a contour
	CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

	//finding all contours in the image
	cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	IplImage img2 = image;
	IplImage* img3 = &img2;
	//printf("%s\n","______________ciclo_____________" );
	bool flag=false;
	CvSeq* contour_i=contour;

	list <CvPoint> centri_rettangoli;
	
	//iterating through each contour
	while(contour) {
		//obtain a sequence of points of the countour, pointed by the variable 'countour'
		//cout<<"PARAM: "<<cvContourPerimeter(contour)*(((float)PARAM)/1000)<<endl;
		result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*/*(((float)PARAM)/1000)*/ 0.045, 0);
		
		/*area = fabs(cvContourArea(result, CV_WHOLE_SEQ));
		if(area>maxArea) {
			maxArea = area;
		}*/


		//if there are 7 vertices  in the contour and the area of the triangle is more than 100 pixels
		if(result->total >= 4  && result->total <= 4 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>50) {
			cvDrawContours(img3, result, cvScalar(255,255,255), cvScalar(255,255,255), 100, 2);
			//cout<<"PARAM: "<<cvContourPerimeter(contour)*(((float)PARAM)/1000)<<endl;
			flag=true;
	         //iterating through each point
	         CvPoint *pt[4];
	         for(int i=0;i<4;i++) {
	             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
	             cvCircle(output, *pt[i], 5, cvScalar(0,0,255));
	         }
	         	
	   
	         //drawing lines around the heptagon
	         cvLine(output, *pt[0], *pt[1], cvScalar(0,0,255),2);
	         cvLine(output, *pt[1], *pt[2], cvScalar(0,0,255),2);
	         cvLine(output, *pt[2], *pt[3], cvScalar(0,0,255),2);
	         cvLine(output, *pt[3], *pt[0], cvScalar(0,0,255),2);
	         //free(pt);
		     //std::cout<<"*******************Vertici "<<result->total;
			//std::cout<<"*******************AREA "<<fabs(cvContourArea(result, CV_WHOLE_SEQ))<<endl;

	        CvMoments moments;
	        cvMoments(result, &moments);
	        int centro_x = moments.m10 / moments.m00;
	        int centro_y = moments.m01 / moments.m00;
	        //cout<<"centro x: "<<centro_x<<" | centro y: "<<centro_y<<endl;
	        centri_rettangoli.push_back(CvPoint(centro_x,centro_y));
	     }
	     
     	//printf("aerea %d, vertici %d\n",fabs(cvContourArea(result, CV_WHOLE_SEQ)), result->total);
		//obtain the next contour
		contour = contour->h_next;
		/*if(result->total>2){
			std::cout<<"*******************Vertici "<<result->total;
			std::cout<<"*******************AREA "<<fabs(cvContourArea(result, CV_WHOLE_SEQ))<<endl;

		}*/
		 
	}

	list<CvPoint>::const_iterator iterator;
	for (iterator = centri_rettangoli.begin(); iterator != centri_rettangoli.end(); ++iterator) {
	   cout <<"("<< (*iterator).x <<","<<(*iterator).y<<")"<<endl;
	   cvCircle(output, *iterator, 3, cvScalar(255,255,255),3);
	}
	/*while(contour_i && !flag)
	{
	     result = cvApproxPoly(contour_i, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour_i)*0.02, 0);
	     printf("aerea %d, vertici %d\n",fabs(cvContourArea(result, CV_WHOLE_SEQ)), result->total);
	     contour_i = contour_i->h_next;
	}*/


	cvReleaseImage(&imgGrayScale);
    free(contour);
    cvReleaseMemStorage(&storage);
	//-----------TRIANGOLI BLUE

	tmp=img_masked_blue;
	img = &tmp;

	imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1); 
	cvCvtColor(img,imgGrayScale,CV_BGR2GRAY);
	
	storage = cvCreateMemStorage(0);
	cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));


	img2 = image;
	img3 = &img2;
	//printf("%s\n","______________ciclo_____________" );
	flag=false;
	contour_i=contour;

	list <CvPoint> centri_triangoli;
	
	//iterating through each contour
	while(contour) {
		//obtain a sequence of points of the countour, pointed by the variable 'countour'
		result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.12, 0);
		
		/*area = fabs(cvContourArea(result, CV_WHOLE_SEQ));
		if(area>maxArea) {
			maxArea = area;
		}*/


		//if there are 7 vertices  in the contour and the area of the triangle is more than 100 pixels
		if(result->total >= 3  && result->total <= 3 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>5) {
			cvDrawContours(img3, result, cvScalar(255,255,0), cvScalar(255,255,0), 100, 2);

			flag=true;
			//iterating through each point
			CvPoint *pt[3];
			for(int i=0;i<3;i++) {
			 pt[i] = (CvPoint*)cvGetSeqElem(result, i);
			 cvCircle(output, *pt[i], 5, cvScalar(255,0,0));
			}
				

			//drawing lines around the heptagon
			cvLine(output, *pt[0], *pt[1], cvScalar(255,0,0),2);
			cvLine(output, *pt[1], *pt[2], cvScalar(255,0,0),2);
			cvLine(output, *pt[2], *pt[0], cvScalar(255,0,0),2);
			//free(pt);
	        CvMoments moments;
	        cvMoments(result, &moments);
	        int centro_x = moments.m10 / moments.m00;
	        int centro_y = moments.m01 / moments.m00;
	        //cout<<"centro x: "<<centro_x<<" | centro y: "<<centro_y<<endl;
	        centri_triangoli.push_back(CvPoint(centro_x,centro_y));
		     
	     }
	     
     	//printf("aerea %d, vertici %d\n",fabs(cvContourArea(result, CV_WHOLE_SEQ)), result->total);
		//obtain the next contour
		contour = contour->h_next;
	}

	
	list<CvPoint>::const_iterator i;
	for (i = centri_triangoli.begin(); i != centri_triangoli.end(); ++i) {
	   cout <<"("<< (*i).x <<","<<(*i).y<<")"<<endl;
	   cvCircle(output, *i, 3, cvScalar(255,0,255),3);
	}








 
	//show the image in which identified shapes are marked   
	cvNamedWindow("Tracked");
	cvShowImage("Tracked",output);

	// Immagine acquisita da camera in rgb8
 	imshow("Immagine acquisita",image);
    cv::waitKey(30);


    cvReleaseImage(&imgGrayScale);
    //cvReleaseImage(&img);
    free(contour);
    cvReleaseMemStorage(&storage);
	//free(result);
	//free(storage);
	//~centri_rettangoli;
}



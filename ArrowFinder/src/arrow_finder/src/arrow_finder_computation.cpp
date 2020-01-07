/********************************************************************************
*
* Arrow finder computation
*
* Copyright (c) 2019
* All rights reserved.
*
* Davide Brugali, Università degli Studi di Bergamo
*
* -------------------------------------------------------------------------------
* File: arrow_finder_computation.hpp
* Created: September 2019
* Author: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
*
* This is a refactored version of the usb_cam library by Robert Bosch LLC (c) 2014
* for recognition of position and orientation of arrow.
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
#include "arrow_finder_computation.hpp"

//settare altezza camera e ho settato il rettangolo  come un poligono a sei vertici così
//trovo le arrows in diagonale
ArrowFinder::ArrowFinder(int height, int width):image_height(height),image_width(width) {}

ArrowFinder::~ArrowFinder() {}

list<arrow_info> ArrowFinder::findArrows(cv::Mat image) {

  R_traslation <<1,0,0,0,
  0,1,0,0,
  0,0,1,-h_cam,
  0,0,0,1;
  R_rot_cam_inclination   <<1,0,0,0,
  0,cos(cam_inclination),-sin(cam_inclination),0,
  0,sin(cam_inclination),cos(cam_inclination),0,
  0,0,0,1;
  R_rot_camera   <<1,0,0,0,
  0,cos(M_PI/2),-sin(M_PI/2),0,
  0,sin(M_PI/2),cos(M_PI/2),0,
  0,0,0,1;


  list <composed_arrow_info> arrows;
  Mat original_image_hsv;
  Mat hsv_red, hsv_blue;
  Mat img_masked_red, img_masked_blue, img_masked_red_blue;
  Mat image_eroded = Mat::zeros(image_height, image_width, CV_8U);
  Mat image_dilated = Mat::zeros(image_height, image_width, CV_8U);
  Mat superior_half_mask = Mat::zeros(image_height, image_width, CV_8U); // Initialization as all zeros
  Mat image_not_eroded_sup_half = Mat::zeros(image_height, image_width, CV_8U);
  CvSeq* contour;  // Hold the pointer to a contour
  CvSeq* result;   // Hold sequence of points of a contour
  CvMemStorage *storage = cvCreateMemStorage(0); // Storage area for all contours

  IplImage* imgGrayScale;

  list <pair<CvPoint,float>> centers_and_areas_of_rect;
  list <pair<CvPoint,float>> centers_and_areas_of_triang;

  if(showColorsThresholdTrackbar) {
    const char* RedThreshold = "HSV RED";
    namedWindow(RedThreshold);
    createTrackbar("MinH red", RedThreshold, &MinH_R, 255);
    createTrackbar("MaxH red", RedThreshold, &MaxH_R, 255);
    createTrackbar("MinS red", RedThreshold, &MinS_R, 255);
    createTrackbar("MaxS red", RedThreshold, &MaxS_R, 255);
    createTrackbar("MinV red", RedThreshold, &MinV_R, 255);
    createTrackbar("MaxV red", RedThreshold, &MaxV_R, 255);

    const char* BlueThreshold = "HSV BLUE";
    namedWindow(BlueThreshold);
    createTrackbar("MinH blue", BlueThreshold, &MinH_B, 255);
    createTrackbar("MaxH blue", BlueThreshold, &MaxH_B, 255);
    createTrackbar("MinS blue", BlueThreshold, &MinS_B, 255);
    createTrackbar("MaxS blue", BlueThreshold, &MaxS_B, 255);
    createTrackbar("MinV blue", BlueThreshold, &MinV_B, 255);
    createTrackbar("MaxV blue", BlueThreshold, &MaxV_B, 255);
  }

  // Gaussian blur on acquired image
  GaussianBlur(image, image, Size(3,3), 0);

  // Conversion of acquired image from orginal format to HSV color format
  cvtColor(image, original_image_hsv, CV_BGR2HSV);

  // Filter of original image acquired using color threshold (red and blue one)
  inRange(original_image_hsv, Scalar(MinH_R, MinS_R, MinV_R), Scalar(MaxH_R, MaxS_R, MaxV_R), hsv_red);
  inRange(original_image_hsv, Scalar(MinH_B, MinS_B, MinV_B), Scalar(MaxH_B, MaxS_B, MaxV_B), hsv_blue);

  // "img_masked_red" and "img_masked_blue" will contain only the red and the blue part (with some tolerance) of the original image
  image.copyTo(img_masked_red,hsv_red); // Copy the original_image to destination "img_masked_red", using as mask the matrix obtain from red InRange
  image.copyTo(img_masked_blue,hsv_blue); // Copy the original_image to destination "img_masked_blue", using as mask the matrix obtain from blue InRange
  bitwise_or(hsv_red, hsv_blue, original_image_hsv); // After this command we will get the red and blue colour layer together
  image.copyTo(img_masked_red_blue,original_image_hsv); // In "img_masked_red_blue" there will be red and blue parts, from image, matched together


  if(showErosionTrackbar) {
    namedWindow(erosionImageWindow, WINDOW_AUTOSIZE);
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", erosionImageWindow, &erosion_elem, max_elem); //, Erosion);
    createTrackbar( "Kernel size:\n 2n +1", erosionImageWindow, &erosion_size, max_kernel_size); //, Erosion);
  }
  if(showDilatationTrackbar) {
    namedWindow(dilatationImageWindow, WINDOW_AUTOSIZE);
    //moveWindow(dilatationImageWindow, src.cols, 0);
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", dilatationImageWindow, &dilation_elem, max_elem); //, Dilation);
    createTrackbar( "Kernel size:\n 2n +1", dilatationImageWindow, &dilation_size, max_kernel_size); //, Dilation);
  }

  Mat image_without_red_areas = tinyRedFiltering(img_masked_red);

  //Erode and dilatate
  Erosion(image_without_red_areas, image_eroded); // "image_without_red_areas" will be under erosion, while the result of the erosion will load in "image_eroded"

  superior_half_mask(Rect(0, 0, image_width, image_height/2)) = 255;
  image_without_red_areas.copyTo(image_not_eroded_sup_half,superior_half_mask);
  bitwise_or(image_eroded,image_not_eroded_sup_half,image_eroded);
  Dilation(image_eroded, image_dilated);

  // ============================================================= RED RECTANGLE =============================================================
  IplImage tmp1=img_masked_red_blue;
  IplImage* output = &tmp1;

  IplImage tmp=image_dilated;
  IplImage* img_without_noise = &tmp;

  // Conversion of the original image, after removing of tiny red areas-erosion-dilatation, into grayscale
  imgGrayScale = cvCreateImage(cvGetSize(img_without_noise), 8, 1);
  cvCvtColor(img_without_noise,imgGrayScale,CV_BGR2GRAY);

  // Finding all contours in the grayscale image without tiny areas and with erosion and dilatation
  cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  img2 = image;
  img3 = &img2;
  CvSeq* contour_i=contour;

  // Iteration over each countor found in image after tuning (removing of tiny red area - erosion - dilatation)
  while(contour) {
    result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour) * 0.045, 0);
    // If there are 4 to 6 vertices  in the contour and the area of the countour is more than lower_area_rect pixels
    if(result->total >= 4  && result->total <= 6 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>lower_area_rect) {
      cvDrawContours(img3, result, cvScalar(0,0,0), cvScalar(0,0,0), 100, 2); // Draw countor found over the image

      CvPoint *pt[4]; // Could be that the countour has 6 vertices: in this case we will'not consider the last two point
      for(int i = 0; i < 4; i++) {
        pt[i] = (CvPoint*)cvGetSeqElem(result, i);
        cvCircle(output, *pt[i], 5, cvScalar(0,0,255)); // Draw circle over each vertice found
      }

      // Drawing lines around the contour
      cvLine(output, *pt[0], *pt[1], cvScalar(0,0,255),2);
      cvLine(output, *pt[1], *pt[2], cvScalar(0,0,255),2);
      cvLine(output, *pt[2], *pt[3], cvScalar(0,0,255),2);
      cvLine(output, *pt[3], *pt[0], cvScalar(0,0,255),2);

      CvMoments moments;
      cvMoments(result, &moments);
      int center_x = moments.m10 / moments.m00;
      int center_y = moments.m01 / moments.m00;
      pair<CvPoint,float> center_and_area(CvPoint(center_x,center_y), cvContourArea(contour));
      centers_and_areas_of_rect.push_back(center_and_area); // Add the new center to the end of the all centers found in the image

      cvCircle(output, center_and_area.first, 3, cvScalar(255,255,255),3);
      //std::cout <<"("<< center_and_area.first.x << "," << center_and_area.first.y <<")"<< endl;
      //std::cout <<" Coordinate x of the center: "<< center_x <<" and coordinate y of the center: "<< center_y << endl;
      //std::cout <<" ******************* Number of verticies "<<result->total;
      //std::cout <<" ******************* Area "<<fabs(cvContourArea(result, CV_WHOLE_SEQ))<<endl;
    }
    // Obtain the next contour
    contour = contour->h_next;
  }


  cvReleaseImage(&imgGrayScale);
  free(contour);
  cvReleaseMemStorage(&storage);

  // ============================================================= BLUE TRIANGLE =============================================================
  tmp=img_masked_blue;
  img_without_noise = &tmp;

  imgGrayScale = cvCreateImage(cvGetSize(img_without_noise), 8, 1);
  cvCvtColor(img_without_noise,imgGrayScale,CV_BGR2GRAY);

  storage = cvCreateMemStorage(0);
  cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  img2 = image;
  img3 = &img2;
  contour_i=contour;

  // Iteration over each countor found in image after tuning (removing of tiny red area - erosion - dilatation)
  while(contour) {
    result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.12, 0);
    //if there are 3 vertices in the contour and the area of the triangle is more than lower_area_triang pixels
    if(result->total >= 3  && result->total <= 3 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>lower_area_triang) {
      cvDrawContours(img3, result, cvScalar(255,255,0), cvScalar(255,255,0), 100, 2);

      CvPoint *pt[3];
      for(int i=0;i<3;i++) {
        pt[i] = (CvPoint*)cvGetSeqElem(result, i);
        cvCircle(output, *pt[i], 5, cvScalar(255,0,0)); // Draw circle over each vertice found
      }

      // Drawing lines around the contour
      cvLine(output, *pt[0], *pt[1], cvScalar(255,0,0),2);
      cvLine(output, *pt[1], *pt[2], cvScalar(255,0,0),2);
      cvLine(output, *pt[2], *pt[0], cvScalar(255,0,0),2);

      CvMoments moments;
      cvMoments(result, &moments);
      int center_x = moments.m10 / moments.m00;
      int center_y = moments.m01 / moments.m00;
      pair<CvPoint,float> center_and_area(CvPoint(center_x,center_y), cvContourArea(contour));
      centers_and_areas_of_triang.push_back(center_and_area); // Add the new center to the end of the all centers found in the image

      cvCircle(output, center_and_area.first, 3, cvScalar(255,0,255),3);
      //std::cout <<"("<< center_and_area.first.x << "," << center_and_rea.first.y <<")"<< endl;
    }
    contour = contour->h_next;
  }

  // Creation of the arrows ( trinagle + rectangle). Here:
  //  i is the iterator for trinagles
  //  j is the iterator for rectangles
  list<pair<CvPoint,float>>::const_iterator i;
  for (i = centers_and_areas_of_triang.begin(); i != centers_and_areas_of_triang.end(); ++i) {

    list<pair<CvPoint,float>>::const_iterator j;
    pair<CvPoint,float> min = (*(centers_and_areas_of_rect.begin()));
    int min_value = sqrt(pow((*i).first.x-min.first.x,2) + pow((*i).first.y-min.first.y,2));

    // Looking for rectangle with minimum distance from triangle i
    for (j = centers_and_areas_of_rect.begin(); j != centers_and_areas_of_rect.end(); ++j) {
      //cout<<"# "<< sqrt((pow((*i).first.x-(*j).first.x,2) + pow((*i).first.y-(*j).first.y,2))) <<" < " << min_value <<endl;
      if(sqrt(pow((*i).first.x-(*j).first.x,2) + pow((*i).first.y-(*j).first.y,2)) < min_value) {
        min = *j;
        min_value = sqrt(pow((*i).first.x-min.first.x,2) + pow((*i).first.y-min.first.y,2));
        //cout<<"OK"<<endl;
      }
    }

    // Filter out noisy acquisitions using minimum distance thr between center of rectangle and center of triangle
    if(min_value < lower_dst_rect_triang){
      composed_arrow_info f;
      f.center_triangle = (*i).first;
      f.center_rectangle = min.first;
      f.area = (*i).second + min.second;

      arrows.push_back(f); // Add a new arrow
      cvLine(output, f.center_rectangle, f.center_triangle, cvScalar(180,55,69),2);
    }
  }

  // Compute the orientation of the arrows
  list <arrow_info> arrow_output;
  float angular_coeff, angle_deg;
  for (list<composed_arrow_info>::const_iterator i = arrows.begin(); i != arrows.end(); ++i) {
    arrow_info new_arrow;
    new_arrow.center.x = ((*i).center_rectangle.x + (*i).center_triangle.x) / 2;
    new_arrow.center.y = ((*i).center_rectangle.y + (*i).center_triangle.y) / 2;

    angular_coeff = -(float)((*i).center_rectangle.y-(*i).center_triangle.y)/(float)(((*i).center_rectangle.x-(*i).center_triangle.x)+0.0000001);

    if(angular_coeff > 0) {
      angle_deg = atan(angular_coeff)*180/M_PI;
      if(((*i).center_rectangle.y-(*i).center_triangle.y)<=0)
      angle_deg = atan(angular_coeff)*180/M_PI + 180;
    } else {
      angle_deg = atan(angular_coeff)*180/M_PI + 180;
      if(((*i).center_rectangle.y-(*i).center_triangle.y)<=0)
      angle_deg = atan(angular_coeff)*180/M_PI;
    }

    new_arrow.orientation = angle_deg;
    new_arrow.area = (*i).area;
    arrow_output.push_back(new_arrow);
  }

  // Show the image in which identified shapes are marked
  cvNamedWindow("Result");
  cvShowImage("Result",output);

  cvReleaseImage(&imgGrayScale);
  free(contour);
  cvClearMemStorage(storage);
  cvReleaseMemStorage(&storage);
  return arrow_output;
}

const arrow_info* ArrowFinder::getBiggestArrow( list<arrow_info> arrow_list) {
  const arrow_info* max = nullptr;
  //filter on the area of the arrows
  if(arrow_list.size() != 0) {
    list<arrow_info>::const_iterator f = arrow_list.begin();
    max = &(*f);
    for (f; f != arrow_list.end(); ++f) {
      if((*max).area < (*f).area)
      max = &(*f);
    }

    cvCircle(img3, (*max).center, 5, cvScalar(0,255,0),5);
  }
  cvShowImage("Acquired image",img3);
  cv::waitKey(30);
  return max;
}

VectorXf ArrowFinder::worldCoordinates(const arrow_info* arrow){
  U_cam << (*arrow).center.x,(*arrow).center.y;

  // ==== ALGORITMO RICONOSIMENTO DISTANZA ====
  X_camera_normalized << (U_cam[0]-c_x)/f_x ,(U_cam[1]-c_y)/f_y ,1;
  //ssommo pi/2 per mettere l'asse delle z uscente dalla camera se fosse "dritta", cam_inclination è l'inclinazione rispetto al palo
  scale = abs(h_cam/(-sin(cam_inclination + M_PI/2)*X_camera_normalized[1] + cos(cam_inclination + M_PI/2)));

  X_camera = X_camera_normalized * scale;
  X_camera_augmented << X_camera , 1;
  X_World = (R_rot_camera * R_rot_cam_inclination * R_traslation).inverse() * X_camera_augmented;
  return X_World;
}

Mat ArrowFinder::tinyRedFiltering(Mat &image_masked_red) {
	CvMemStorage* tinyRedFilterStorage = cvCreateMemStorage(0);
	CvSeq* tinyRedCountours;
	IplImage* localImgGrayScale;

	// TODO: capire se è possibile evitare di continuare ad usare questa conversione da Mat a IplImage (https://stackoverflow.com/questions/5192578/opencv-iplimage)
	IplImage tmp6=image_masked_red;
	IplImage* img6 = &tmp6;
	Mat mask = Mat::zeros(image_height, image_width, CV_8U); // All pixel set to 0
	mask(Rect(0, 0, image_width, image_height)) = 255;
	IplImage mask_ipl=mask;
	IplImage* mask_ipl2 = &mask_ipl;

	localImgGrayScale = cvCreateImage(cvGetSize(img6), 8, 1);
	// Conversione da scala HSV a scala di grigi dell'immagine
	cvCvtColor(img6,localImgGrayScale,CV_BGR2GRAY);
	cvFindContours(localImgGrayScale, tinyRedFilterStorage, &tinyRedCountours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	while(tinyRedCountours) {
		if(cvContourArea(tinyRedCountours) < 3)
			cvDrawContours(mask_ipl2, tinyRedCountours, cvScalar(0,0,0), cvScalar(0,0,0), 100, 1);
		tinyRedCountours = tinyRedCountours->h_next;
	}

	Mat image_without_red_areas;
	image_masked_red.copyTo(image_without_red_areas,mask);

	cvReleaseImage(&localImgGrayScale);
	free(tinyRedCountours);
	cvClearMemStorage(tinyRedFilterStorage);
	cvReleaseMemStorage(&tinyRedFilterStorage);

	return image_without_red_areas;
}

void ArrowFinder::Erosion(Mat in, Mat &out) {
	int erosion_type = 0;

	if( erosion_elem == 0 )
		erosion_type = MORPH_RECT;
	else if(erosion_elem == 1)
		erosion_type = MORPH_CROSS;
	else if(erosion_elem == 2)
		erosion_type = MORPH_ELLIPSE;

	Mat element = getStructuringElement(erosion_type,
				Size(2*erosion_size + 1, 2*erosion_size+1),
				Point(erosion_size, erosion_size));
				erode(in, out, element);
    //imshow( "Erosion Demo", out );
}

void ArrowFinder::Dilation(Mat in, Mat &out) {
	int dilation_type = 0;

	if( dilation_elem == 0 )
		dilation_type = MORPH_RECT;
	else if(dilation_elem == 1)
		dilation_type = MORPH_CROSS;
	else if(dilation_elem == 2)
		dilation_type = MORPH_ELLIPSE;

	Mat element = getStructuringElement(dilation_type,
				Size(2*dilation_size + 1, 2*dilation_size+1),
				Point(dilation_size, dilation_size));
				dilate(in, out, element);
	//imshow( "Dilation Demo", out);
}



//TODO: remove
/*
void ArrowFinder::setImage(cv::Mat image, int image_height, int image_width) {

image_width = image_width;
image_heigth = image_height;

//filtro sulla freccia di area maggiore
if(arrows.size() != 0) {
list<arrow_info>::const_iterator f = arrows.begin();
arrow_info max = *f;
for (f; f != arrows.end(); ++f) {
if(max.area < (*f).area)
max = *f;
//cout<<"Freccia trovata"<<endl;
}

//calcolo punto medio tra il centro del rettangolo e il centro del triangolo
max.centro_rettangolo.x = (max.centro_rettangolo.x + max.centro_triangolo.x) / 2;
max.centro_rettangolo.y = (max.centro_rettangolo.y + max.centro_triangolo.y) / 2;


cvCircle(img3, max.centro_rettangolo, 5, cvScalar(0,255,0),5);
float angular_coeff = -(float)(max.centro_rettangolo.y-max.centro_triangolo.y)/(float)((max.centro_rettangolo.x-max.centro_triangolo.x)+0.0000001);

float angle_deg;
if(angular_coeff>0) {
angle_deg = atan(angular_coeff)*180/M_PI;
if((max.centro_rettangolo.y-max.centro_triangolo.y)<=0) {
angle_deg = atan(angular_coeff)*180/M_PI + 180;
}
} else {
angle_deg = atan(angular_coeff)*180/M_PI + 180;
if((max.centro_rettangolo.y-max.centro_triangolo.y)<=0) {
angle_deg = atan(angular_coeff)*180/M_PI;
}
}
cout << "Centro rettangolo --> <x =" << max.centro_rettangolo.x << "; y = "<< max.centro_rettangolo.y <<">" <<endl;
cout<<"m= "<<angular_coeff<<" tangente: "<<angle_deg<<endl;
U_cam << max.centro_rettangolo.x,max.centro_rettangolo.y;

// ==== ALGORITMO RICONOSIMENTO DISTANZA ====
X_camera_normalized << (U_cam[0]-c_x)/f_x ,(U_cam[1]-c_y)/f_y ,1;
scale = h_cam/(0.82*X_camera_normalized[1] + 0.57);
X_camera = X_camera_normalized * scale;
X_camera_augmented << X_camera , 1;
X_World = (R_rot_camera * R_rot_cam_inclination * R_traslation).inverse() * X_camera_augmented;

cout<<"Coordinate World: \n*****\n"<<X_World<<"\n*****\n";


}

//show the image in which identified shapes are marked
cvNamedWindow("Tracked");
cvShowImage("Tracked",output);

// Immagine acquisita da camera in rgb8
imshow("Immagine acquisita",image);
cv::waitKey(30);


cvReleaseImage(&imgGrayScale);
free(contour);
cvClearMemStorage(storage);
cvReleaseMemStorage(&storage);


/*CvSeq *j = result;
while(j){
cvClearMemStorage(j->storage);
//cvReleaseMemStorage(&(j->storage));
j = j->h_next;
}

//free(result);
}*/

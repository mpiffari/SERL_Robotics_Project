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


//immagini
#define DEBUG
#ifdef DEBUG

IplImage img2;
IplImage* img3;

#endif


using namespace Eigen;
using namespace std;
using namespace cv;

bool computationInProgress = false;
Mat localImage;

// TODO: put in a global declaration file
const char* nameMainImageWindow = "Field Of View";
const char* erosionImageWindow = "Erosion demo";
const char* dilatationImageWindow = "Dilatation demo";
bool showColorsThresholdTrackbar = false; // Flag to activate trackabar window for tuning of HSV threshold
bool showErosionTrackbar = false; // Flag to activate trackbar windowd for erosion tuning
bool showDilatationTrackbar = false; // Flag to activate trackbar windowd for dilatation tuning

// Image dimensions
int image_height;
int image_width;

//int flag = false;
int start = 0;
float r;    	//Raggio marker

// Red mask CSV threshold
int MinH_R=0;
int MaxH_R=10;
int MinS_R=70;
int MaxS_R=255;
int MinV_R=172;
int MaxV_R=255;

// Blue mask CSV threshold
int MinH_B=100;
int MaxH_B=180;
int MinS_B=0;
int MaxS_B=255;
int MinV_B=65;
int MaxV_B=200;

int PARAM;


// Erosion and dilation parameters definition
int erosion_elem = 0;
int dilation_elem = 0;
int erosion_size = 1; // Setting of erosion type [MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE]
int dilation_size = 3; // Setting of dilation type [MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE]
// Erosion and dilation tuning parameters
int const max_elem = 2;
int const max_kernel_size = 21;
cv::Mat tinyRedFiltering(cv::Mat &image_masked_red);
void Erosion(Mat in, Mat &out);
void Dilation(Mat in, Mat &out);

// Camera tilt
float cam_inclination = asin(2/3.5);
// Intrinsic parameters of the camera (focal and center position)
float f_x = 463.713374;
float f_y = 464.444408;
float c_x = 316.855629;
float c_y = 255.988008;
float h_cam = 0.310; // Height of the camera from the ground expressed in [m]
float scale; // Scale factor used to detect deepness of the point

MatrixXf R_traslation(4,4); // Traslation of point acquired from ground position to camera height (see documentation)
MatrixXf R_rot_theta(4,4); // Rotation of the point acquired of an angle equals to "cam_inclination" (see documentation)
MatrixXf R_rot_camera(4,4); // Rotation from the frame of the camera to the base ground (see documentation)
VectorXf U_cam(2); // Coordinates of the point in the image frame
VectorXf X_camera_normalized(3); // Coordinates of the point in the normalized frame
VectorXf X_camera(3); // Coordinates of the point in the camera frame
VectorXf X_camera_augmented(4); // Useful for passage from 2D to 3D by adding "fictitious" third coordinate
VectorXf X_World(4);

//settare altezza camera e ho settato il rettangolo  come un poligono a sei vertici così
//trovo le arrows in diagonale
ArrowFinder::ArrowFinder() {

}

ArrowFinder::~ArrowFinder() {
}


list<arrow_info> ArrowFinder::findArrows(cv::Mat image) {

  R_traslation <<1,0,0,0,
  0,1,0,0,
  0,0,1,-h_cam,
  0,0,0,1;
  R_rot_theta   <<1,0,0,0,
  0,cos(cam_inclination),-sin(cam_inclination),0,
  0,sin(cam_inclination),cos(cam_inclination),0,
  0,0,0,1;
  R_rot_camera   <<1,0,0,0,
  0,cos(M_PI/2),-sin(M_PI/2),0,
  0,sin(M_PI/2),cos(M_PI/2),0,
  0,0,0,1;


  list <composed_arrow_info> freccie;
  Mat original_image_hsv;
  Mat hsv_red, hsv_blue;
  Mat img_masked_red, img_masked_blue, img_masked_red_blue;
  Mat image_eroded = Mat::zeros(image_height, image_width, CV_8U);
  Mat image_dilated = Mat::zeros(image_height, image_width, CV_8U);
  Mat superior_half_mask = Mat::zeros(image_height, image_width, CV_8U); // Initialization as all zeros
  Mat image_eroded_only_sup_half = Mat::zeros(image_height, image_width, CV_8U);
  CvSeq* contour;  // Hold the pointer to a contour
  CvSeq* result;   // Hold sequence of points of a contour
  CvMemStorage *storage = cvCreateMemStorage(0); // Storage area for all contours

  IplImage* imgGrayScale;

  list <pair<CvPoint,float>> centri_rettangoli;
  list <pair<CvPoint,float>> centri_triangoli;

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
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", erosionImageWindow, &erosion_elem, max_elem); //,Erosion);
    createTrackbar( "Kernel size:\n 2n +1", erosionImageWindow,&erosion_size, max_kernel_size); //,Erosion);
  }
  if(showDilatationTrackbar) {
    namedWindow(dilatationImageWindow, WINDOW_AUTOSIZE);
    //moveWindow(dilatationImageWindow, src.cols, 0);
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", dilatationImageWindow,&dilation_elem, max_elem); //,Dilation);
    createTrackbar( "Kernel size:\n 2n +1", dilatationImageWindow,&dilation_size, max_kernel_size); //,Dilation);
  }

  Mat image_without_red_areas = tinyRedFiltering(img_masked_red);

  //Erode and dilatate
  Erosion(image_without_red_areas, image_eroded); // "image_without_red_areas" will be under erosion, while the result of the erosion will load in "image_eroded"

  superior_half_mask(Rect(0, 0, image_width, image_height/2)) = 255;
  image_without_red_areas.copyTo(image_eroded_only_sup_half,superior_half_mask);
  bitwise_or(image_eroded,image_eroded_only_sup_half,image_eroded);
  Dilation(image_eroded, image_dilated);

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
  //printf("%s\n","______________ciclo_____________" );
  //bool flag=false;
  CvSeq* contour_i=contour;

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
  if(result->total >= 4  && result->total <= 6 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>lower_area_rect) {

    cvDrawContours(img3, result, cvScalar(0,0,0), cvScalar(0,0,0), 100, 2);
    //cout<<"PARAM: "<<cvContourPerimeter(contour)*(((float)PARAM)/1000)<<endl;
    //flag=true;
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
    pair<CvPoint,float> r(CvPoint(centro_x,centro_y), cvContourArea(contour));
    centri_rettangoli.push_back(r);
  }

  //printf("aerea %d, vertici %d\n",fabs(cvContourArea(result, CV_WHOLE_SEQ)), result->total);
  //obtain the next contour
  contour = contour->h_next;
  /*if(result->total>2){
  std::cout<<"*******************Vertici "<<result->total;
  std::cout<<"*******************AREA "<<fabs(cvContourArea(result, CV_WHOLE_SEQ))<<endl;

}*/

}


//TODO: spostare nel ciclo precedente
list<pair<CvPoint,float>>::const_iterator iterator;
for (iterator = centri_rettangoli.begin(); iterator != centri_rettangoli.end(); ++iterator) {
  //cout <<"("<< (*iterator).first.x <<","<<(*iterator).first.y<<")"<<endl;
  cvCircle(output, (*iterator).first, 3, cvScalar(255,255,255),3);
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
img_without_noise = &tmp;

imgGrayScale = cvCreateImage(cvGetSize(img_without_noise), 8, 1);
cvCvtColor(img_without_noise,imgGrayScale,CV_BGR2GRAY);

storage = cvCreateMemStorage(0);
cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));


img2 = image;
img3 = &img2;
//printf("%s\n","______________ciclo_____________" );
//flag=false;
contour_i=contour;



//iterating through each contour
while(contour) {
  //obtain a sequence of points of the countour, pointed by the variable 'countour'
  result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.12, 0);

  /*area = fabs(cvContourArea(result, CV_WHOLE_SEQ));
  if(area>maxArea) {
  maxArea = area;
}*/


//if there are 7 vertices  in the contour and the area of the triangle is more than 100 pixels
if(result->total >= 3  && result->total <= 3 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>lower_area_triang) {
  cvDrawContours(img3, result, cvScalar(255,255,0), cvScalar(255,255,0), 100, 2);

  //flag=true;
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


  pair<CvPoint,float> rr(CvPoint(centro_x,centro_y), cvContourArea(contour));
  centri_triangoli.push_back(rr);

}
contour = contour->h_next;
}

//TODO: spostare nel ciclo precedente
list<pair<CvPoint,float>>::const_iterator i;
for (i = centri_triangoli.begin(); i != centri_triangoli.end(); ++i) {
  //cout <<"("<< (*i).first.x <<","<<(*i).first.y<<")"<<endl;
  cvCircle(output, (*i).first, 3, cvScalar(255,0,255),3);
}





//calcolo le freccie e la loro area
//i è l'iteratore dei triangoli
//j è l'iteratore dei rettangoli
for (i = centri_triangoli.begin(); i != centri_triangoli.end(); ++i) {

  list<pair<CvPoint,float>>::const_iterator j;
  pair<CvPoint,float> min = (*(centri_rettangoli.begin()));
  int min_value = sqrt(pow((*i).first.x-min.first.x,2) + pow((*i).first.y-min.first.y,2));

  //calcolo il rettangolo di distanza minima dal triangolo i
  for (j = centri_rettangoli.begin(); j != centri_rettangoli.end(); ++j) {
    //cout<<"#"<<sqrt((pow((*i).first.x-(*j).first.x,2) + pow((*i).first.y-(*j).first.y,2)))<<" < "<<min_value<<endl;
    if(  sqrt(pow((*i).first.x-(*j).first.x,2) + pow((*i).first.y-(*j).first.y,2)) < min_value){
      min = *j;
      //cout<<"OK"<<endl;
      min_value = sqrt(pow((*i).first.x-min.first.x,2) + pow((*i).first.y-min.first.y,2));
    }
  }

  //filtro le freccie in base alla distanza tra il centro del triangolo e il centro del rettangolo
  if(min_value < lower_dst_rect_triang){
    composed_arrow_info f;
    f.center_triangle = (*i).first;
    f.center_rectangle = min.first;
    f.area = (*i).second + min.second;

    freccie.push_back(f);
    cvLine(output, f.center_rectangle, f.center_triangle, cvScalar(180,55,69),2);
  }
}

//compute the orientation of the arrows
list <arrow_info> arrow_output;
float coefficiente_angolare, angolo_deg;
for (list<composed_arrow_info>::const_iterator i = freccie.begin(); i != freccie.end(); ++i) {
  arrow_info new_arrow;
  new_arrow.center.x = ((*i).center_rectangle.x + (*i).center_triangle.x) / 2;
  new_arrow.center.y = ((*i).center_rectangle.y + (*i).center_triangle.y) / 2;

  coefficiente_angolare = -(float)((*i).center_rectangle.y-(*i).center_triangle.y)/(float)(((*i).center_rectangle.x-(*i).center_triangle.x)+0.0000001);

  if(coefficiente_angolare>0) {
    angolo_deg = atan(coefficiente_angolare)*180/M_PI;
    if(((*i).center_rectangle.y-(*i).center_triangle.y)<=0)
    angolo_deg = atan(coefficiente_angolare)*180/M_PI + 180;
  } else {
    angolo_deg = atan(coefficiente_angolare)*180/M_PI + 180;
    if(((*i).center_rectangle.y-(*i).center_triangle.y)<=0)
    angolo_deg = atan(coefficiente_angolare)*180/M_PI;
  }

  new_arrow.orientation = angolo_deg;
  new_arrow.area = (*i).area;
  arrow_output.push_back(new_arrow);
}


//show the image in which identified shapes are marked
cvNamedWindow("Tracked");
cvShowImage("Tracked",output);

// Immagine acquisita da camera in rgb8
//imshow("Immagine acquisita",image);



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
  cvShowImage("Immagine acquisita",img3);
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
  X_World = (R_rot_camera * R_rot_theta * R_traslation).inverse() * X_camera_augmented;
  return X_World;
}


/*
void ArrowFinder::setImage(cv::Mat image, int image_height, int image_width) {

image_width = image_width;
image_heigth = image_height;

//filtro sulla freccia di area maggiore
if(freccie.size() != 0) {
list<arrow_info>::const_iterator f = freccie.begin();
arrow_info max = *f;
for (f; f != freccie.end(); ++f) {
if(max.area < (*f).area)
max = *f;
//cout<<"Freccia trovata"<<endl;
}

//calcolo punto medio tra il centro del rettangolo e il centro del triangolo
max.centro_rettangolo.x = (max.centro_rettangolo.x + max.centro_triangolo.x) / 2;
max.centro_rettangolo.y = (max.centro_rettangolo.y + max.centro_triangolo.y) / 2;


cvCircle(img3, max.centro_rettangolo, 5, cvScalar(0,255,0),5);
float coefficiente_angolare = -(float)(max.centro_rettangolo.y-max.centro_triangolo.y)/(float)((max.centro_rettangolo.x-max.centro_triangolo.x)+0.0000001);

float angolo_deg;
if(coefficiente_angolare>0) {
angolo_deg = atan(coefficiente_angolare)*180/M_PI;
if((max.centro_rettangolo.y-max.centro_triangolo.y)<=0) {
angolo_deg = atan(coefficiente_angolare)*180/M_PI + 180;
}
} else {
angolo_deg = atan(coefficiente_angolare)*180/M_PI + 180;
if((max.centro_rettangolo.y-max.centro_triangolo.y)<=0) {
angolo_deg = atan(coefficiente_angolare)*180/M_PI;
}
}
cout << "Centro rettangolo --> <x =" << max.centro_rettangolo.x << "; y = "<< max.centro_rettangolo.y <<">" <<endl;
cout<<"m= "<<coefficiente_angolare<<" tangente: "<<angolo_deg<<endl;
U_cam << max.centro_rettangolo.x,max.centro_rettangolo.y;

// ==== ALGORITMO RICONOSIMENTO DISTANZA ====
X_camera_normalized << (U_cam[0]-c_x)/f_x ,(U_cam[1]-c_y)/f_y ,1;
scale = h_cam/(0.82*X_camera_normalized[1] + 0.57);
X_camera = X_camera_normalized * scale;
X_camera_augmented << X_camera , 1;
X_World = (R_rot_camera * R_rot_theta * R_traslation).inverse() * X_camera_augmented;

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

// Filter of tiny contour in image with red pixel
cv::Mat tinyRedFiltering(cv::Mat &image_masked_red) {
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
    if(cvContourArea(tinyRedCountours) < 3) {
      cvDrawContours(mask_ipl2, tinyRedCountours, cvScalar(0,0,0), cvScalar(0,0,0), 100, 1);
    }
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

void Erosion(Mat in, Mat &out)
{
  int erosion_type = 0;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( erosion_type,
    Size( 2*erosion_size + 1, 2*erosion_size+1 ),
    Point( erosion_size, erosion_size ) );
    erode( in, out, element );
    //imshow( "Erosion Demo", out );
  }

  void Dilation(Mat in, Mat &out)
  {
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    Mat element = getStructuringElement( dilation_type,
      Size( 2*dilation_size + 1, 2*dilation_size+1 ),
      Point( dilation_size, dilation_size ) );
      dilate( in, out, element );
      //imshow( "Dilation Demo", out);
    }

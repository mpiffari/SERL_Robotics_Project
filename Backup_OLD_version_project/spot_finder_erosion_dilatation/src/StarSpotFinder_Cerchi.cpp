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

#include <iostream>
#include <time.h>
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

int start = 0;
int minH, maxH, minS, maxS, minV, maxV,z,zz,zzz,zzzz;
float alpha_cam;	//Angolo asse telecamera rispetto alla verticale in gradi
float r;    	//Raggio marker

StarSpotFinder::StarSpotFinder() {
}

StarSpotFinder::~StarSpotFinder() {
}


/*bool StarSpotFinder::setup(const std::string &filename) {
std::cout << "StarSpotFinder::setup" << std::endl;
    conf.loadValues(filename);

    return true;
}*/


void StarSpotFinder::setImage(cv::Mat image, int image_height, int image_width) {
        
	output.clear();
        char scelta;
        
        if(start == 0) {
              cout << "Scegliere colore marcatore (b, r, y, w): ";
              cin >> scelta;

              start = 1;

		switch (scelta)
		{  
		    case 'w':
		    //case 'W': minH=50, maxH=150, minS=0, maxS=75, minV=130, maxV=210;	//Bianco su nero
		    case 'W': minH=0, maxH=255, minS=0, maxS=20, minV=195, maxV=255;	//Bianco
		    break;	
		    
		    case 'b':
		    //case 'B': minH=0, maxH=255, minS=0, maxS=255, minV=0, maxV=85;	//Nero su bianco
		    case 'B': minH=0, maxH=255, minS=0, maxS=100, minV=0, maxV=100;	//Nero
		    break;
		    
		    case 'y':
		    //case 'Y': minH=20, maxH=255, minS=80, maxS=255, minV=110, maxV=235;	//Giallo su nero		                 
		    case 'Y': minH=25, maxH=65, minS=65, maxS=255, minV=65, maxV=235;	//Giallo
		    break;

		    case 'r':
		    //case 'R': minH=90, maxH=90, minS=70, maxS=255, minV=50, maxV=255;  	//Rosso su nero	
		    case 'R': minH=0, maxH=7, minS=225, maxS=255, minV=120, maxV=255;  	//Rosso	                 
		    break;
		 }

		cout << "Inserire angolo cam: ";
		cin >> alpha_cam;

		cout << "Inserire r marker: ";
		cin >> r;
         }	

	const char* windowName="individua marcatori";
	const char* countourWindow="Contours rilevati";
	
	//Parametri	
	float h_cam = 16;     //Altezza telecamera in cm
	float f_y = 49.5;	//Apertura focale verticale della telecamera in gradi
	float f_x = 71;	//Apertura focale orizzontale della telecamera in gradi
	float res_x = 640.0;	//Risoluzione orizzontale telecamera
    float res_y = 480.0;	//Risoluzione verticale telecamera

	//Variabili
	float y_FOV;    //Lunghezza campo visivo in cm	
	float y_FOV_0;  //Distanza tra l'origine e FOV inferiore
	float b;	//Base minore FOV
	float B;	//Base maggiore FOV	
	float y_sup; //Distanza cam e B
	float alpha_t; //Angolo a terra corrispondente a f_x

	y_FOV_0 = h_cam*tan( (alpha_cam-f_y/2)*PI/180.0 ); 
	y_FOV = h_cam*tan( (alpha_cam+f_y/2)*PI/180.0 )-y_FOV_0;
	y_sup = sqrt((y_FOV_0+y_FOV)*(y_FOV_0+y_FOV)+(h_cam)*(h_cam)); 
	B = 2*y_sup*tan((f_x/2)*PI/180.0);
	alpha_t = 2*atan(B/(2*(y_FOV_0+y_FOV)))*180.0/PI;
	b = 2*y_FOV_0*tan((alpha_t/2)*PI/180.0);

	
	//Settaggio trackbar per impostazione colore.
	namedWindow(windowName);
	createTrackbar("MinH", windowName, &minH, 255);
	createTrackbar("MaxH", windowName, &maxH, 255);
	createTrackbar("MinS", windowName, &minS, 255);
	createTrackbar("MaxS", windowName, &maxS, 255);
	createTrackbar("MinV", windowName, &minV, 255);
	createTrackbar("MaxV", windowName, &maxV, 255);

	Mat hsv;

	cvtColor(image, hsv, CV_BGR2HSV);	//Converto in formato HSV


	



	

	if( scelta == 'r' || scelta == 'R')
	{
		Mat1b mask1, mask2;
		inRange(hsv, Scalar(0, minS, minV), Scalar(30, maxS, maxV), mask1);
		inRange(hsv, Scalar(130, minS, minV), Scalar(190, maxS, maxV), mask2);
		hsv = mask1 | mask2;	
	}
	else
		inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), hsv);	//Range di colori d considerare

	//Pre processing
	int blurSize = 5, elementSize = 5;
	medianBlur(hsv, hsv, blurSize);	  //Sfoca l'immagine usando un median filter
/*
	//Contour detection
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(hsv, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));	//Trova i contorni
	
	float area_c = (1/(h_cam)*4000+(80-h_cam))/tan((alpha_cam-7)*PI/180.0);	//Area che ha il marcatore in base all'altezza della cam e all'inclinazione
	//printf("=======================================================\n");
	//printf("Area c: %f \n", area_c);

	double p_o, area_o;	//Perimetro e area dell'oggetto rilevato

	for (size_t i = 0; i < contours.size(); i++)
	{

		p_o=arcLength(contours[i],true);
		area_o=-contourArea(contours[i],true);
		//printf("Area o: %f \n", area_o);

		//Calcolo coordinata x minima e massima della figura
		int max_x = 0;
		int min_x = res_x;

		//Calcolo estremo destro (max_x) e sinistro (min_x) del contorno
		for(vector<Point>::iterator it = contours[i].begin(); it != contours[i].end(); ++it)
		{
			if( max_x < it->x)
				max_x = it->x;
			if( min_x > it->x)
				min_x = it->x;
		}

		float gamma_max = max_x*alpha_t/res_x;	//Angolo pixel x_max orizzontale
		float gamma_min = min_x*alpha_t/res_x;	//Angolo pixel x_min orizzontale	
			
		if(p_o>=sqrt(4*3.14*area_o)-15 && p_o<=sqrt(4*3.14*area_o)+28 && area_o > area_c)    //Filtro in base alla forma e alla dimensione del marcatore
		{	
			//Calcolo centro marcatore
			Moments m = moments(contours[i],true);
			Point po(m.m10/m.m00, m.m01/m.m00);
				
	 		//Mostra centro marcatore
			circle(image, po, 3, Scalar(128,0,0), -1);
				
			float beta = f_y*(res_y-po.y)/res_y;	//Angolo pixel verticale

			//Conversione pixel-centimetri
			float gamma = alpha_t*po.x/res_x;	//Angolo pixel orizzontale
			float y_tot = h_cam * tan( (alpha_cam-f_y/2 + beta)*PI/180.0 );	//Coordinata y marcatore - origine sistema
			float x_tot = tan(abs(alpha_t/2-gamma)*PI/180.0)*y_tot;	//Coordinata x marcatore - origine sistema
				
			float x_max = y_tot*tan(abs(alpha_t/2-gamma_max)*PI/180.0);	//Calcolo estremo destro marcatore
			float x_min = y_tot*tan(abs(alpha_t/2-gamma_min)*PI/180.0);	//Calcolo estremo sinistro marcatore
			float x_imm = (B*y_tot)/(y_FOV_0 + y_FOV); //Larghezza immagine riferita al centro del marker
			
			//Settaggio x_max e x_min in base al sistema di riferimento	
			if( max_x < res_x/2 )
				x_max = -x_max;

			if( min_x < res_x/2 )
				x_min = -x_min;
			
			//Calcolo diametro marcatore 	
			float d = abs(x_max - x_min)*(1.2+cos((60+beta)*PI/180.0));
 			//printf("x max: %f \n", x_max);
 			//printf("x min: %f \n", x_min);
 			//printf("Angolo beta [gradi]: %f \n", beta);
 			//printf("Diametro rilevato: %f \n", d);

			//Filtro in base al diametro del marcatore
			if((d > 1.75*r && d < 2.75*r) && x_imm/2-x_tot > 0.75*r)
			{
				printf("====== Cerchio rilevato ========\n");
				Scalar color( rand()&255, rand()&255, rand()&255 );
				drawContours(image, contours, i, color, FILLED,8,hierarchy);		//Disegna il contorno del marker
				//Settaggio x_tot in base al sistema di riferimento
				if( po.x < res_x/2 )
					x_tot = -x_tot;

				//Salvataggio coordinate del centro del marcatore (in m)
				output.push_back(Point2f(x_tot/100,y_tot/100));
			}
		}
	}
	
	////////////////////////PROVA PER VISUALIZZARE LE COORDINATE//////////////////////////
	for(vector<Point2f>::iterator it = output.begin(); it != output.end(); ++it)
	{
		cout << "Coordinata x: " << it->x << " m" << endl << "Coordinata y: " << it->y << " m" << endl;
		cout << "Distanza puntatore-telecamera : " << sqrt(pow(it->x,2)+pow(it->y,2)) << " m" << endl << endl;
	}
	*/
	imshow(windowName, hsv);
	imshow("Contorno marcatori", image);
        cv::waitKey(30);





   //======================================================
	// Read image
	Mat im = hsv;


 	for (int i = 0; i < im.rows; i++)
    {
        for (int j = 0; j < im.cols; j++)
        {
            im.at<uchar>(i, j) = 255 - im.at<uchar>(i, j);
        }
    }
	
	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
	 
	// Set Area filtering parameters 
	params.filterByArea = true;
	//params.minArea = 1;

	// Set Circularity filtering parameters 
	params.filterByCircularity = true;
	//params.minCircularity = 0.6;

	// Set Convexity filtering parameters 
	params.filterByConvexity = true;
	//params.minConvexity = 0.6;
		
	// Set inertia filtering parameters 
	params.filterByInertia = true;
	//params.minInertiaRatio = 0.6;

	namedWindow("keypoints");
	createTrackbar("area", "keypoints", &z, 500);
	params.minArea = z;
	createTrackbar("circularity", "keypoints", &zz, 100);
	params.minCircularity = (float)zz/100;
	createTrackbar("Convexity", "keypoints", &zzz, 100);
	params.minConvexity = (float)zzz /100;
	createTrackbar("InertiaRatio", "keypoints", &zzzz, 100);
	params.minInertiaRatio = (float)zzzz /100;



	// Set up the detector with default parameters.
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	detector->detect(im, keypoints);
	 
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	 
	// Show blobs
	imshow("keypoints", im_with_keypoints);
	//======================================================
}



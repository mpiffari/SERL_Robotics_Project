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

#ifndef STAR_ARROWFINDER
#define STAR_ARROWFINDER

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

// TODO: il prof vuole aggiungere nell'interfaccia due metodi:
// - setImage
// - getImage --> l'elaborazione deve avvenire solamente quando il robot chiama il metodo getImage e 
// non quando è disponibile un'immagine. In questo modo non tutte le immagini saranno utilizzate, ma solamente
// quelle che voglio.
// Si può realizzare usando un flag booleano per abilitato quando inizio la computazione
// e disabilitato quando finisce; il tutto deve essere regolato da un accesso concorrente.


class ArrowFinder {
  public:
    std::vector<cv::Point2f> output;	//Coordinate dei marcatori
    ArrowFinder();
    ~ArrowFinder();

    bool setup(const std::string &filename);
    void setImage(cv::Mat original_image, int image_height, int image_width);
    void getImage(); // Computation parte
  private:
};

#endif

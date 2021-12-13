/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <math.h>
#include "event-driven/core.h"

namespace ev {

class pixelShifter {
    //angles
    double thetaY;
    double thetaX;
    double CY, SY;
    double CX, SX;
    double xshift;
    double yshift;
    double ts_scaler;

   public:

    pixelShifter();
    
    void setRotation(double pitch, double yaw);

    void setShift(int xoffset, int yoffset, double tsoffset);

    void pttr(int &x, int &y, double &z);
};

pixelShifter drawISOBase(int height, int width, int period, cv::Mat &baseimage);


}
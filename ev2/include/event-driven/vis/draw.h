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

static const cv::Vec3b aqua{151, 174, 6};
static const cv::Vec3b violet{180, 10, 155};
static const cv::Vec3b orange{9, 111, 255};
static const cv::Vec3b lime{9, 250, 222};
static const cv::Vec3b white{255, 255, 255};
static const cv::Vec3b black{0, 0, 0};
static const cv::Vec3b red{0, 0, 255};
static const cv::Vec3b grey{128, 128, 128};
static const cv::Vec3b naqua{0.05 * (cv::Vec3b(255, 255, 255) - aqua)};
static const cv::Vec3b nviolet{0.05 * (cv::Vec3b(255, 255, 255) - violet)};

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

pixelShifter drawISOBase(int height, int width, double period, cv::Mat &baseimage);

class isoImager
{
private:
    cv::Mat base_image;
    ev::pixelShifter ps;
    double time_window{1.0};

public:

    cv::Size init(int height, int width, double time_window)
    {
        this->time_window = time_window;
        //this initialises the pixel shifter and the image with axis drawn
        ps = ev::drawISOBase(height, width, time_window, base_image);
        return base_image.size();
    }

    template <typename T>
    void draw(cv::Mat img, T begin, T end) {
        double t0 = begin.packetTime();
        for (auto a = begin; a != end; a++) {
            int x = a->x;
            int y = a->y;
            double dt = time_window - (a.packetTime() -t0);
            if (dt < 0)
                return;
            double z = dt;
            ps.pttr(x, y, z);
            if (x < 0 || x >= img.cols || y < 0 || y >= img.rows)
                continue;
            if (a->p)
                img.at<cv::Vec3b>(y, x) -= naqua;
            else
                img.at<cv::Vec3b>(y, x) -= nviolet;

            if (dt < 0.05) {
                int x = a->x;
                int y = a->y;
                double z = 0;
                ps.pttr(x, y, z);
                if (x < 0 || x >= img.cols || y < 0 || y >= img.rows)
                    continue;
                if (a->p)
                    img.at<cv::Vec3b>(y, x) = aqua;
                else
                    img.at<cv::Vec3b>(y, x) = violet;
            }
        }
    }
}; 



}
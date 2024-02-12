/*
 *   Copyright (C) 2022 Event-driven Perception for Robotics
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
namespace ev {

class pwtripletvelocity
{
private:
    double tolerance{0.15};
    
    static const std::vector< std::vector<cv::Point> > is;
    static const std::vector<cv::Point2d> vs;

    typedef struct wjv {
        cv::Point2d v;
        int  c;
        // wjv& operator+=(const wjv& rhs) {
        //     this->c += rhs.c;
        //     this->v = {this->v.u + rhs.v.u, this->v.v + rhs.v.v};
        //     return *this;
        // }
    } wjv;

    wjv point_velocity(const cv::Mat &local_sae);

public:
    double prev_update_ts;
    cv::Point2d area_velocity(const cv::Mat &area_sae);

};


}
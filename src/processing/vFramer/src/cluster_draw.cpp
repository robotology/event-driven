/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *           chiara.bartolozzi@iit.it
 *           massimiliano.iacono@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vDraw.h"

using namespace ev;

const std::string clusterDraw::drawtype = "CLE";

std::string clusterDraw::getDrawType()
{
    return clusterDraw::drawtype;
}

std::string clusterDraw::getEventType()
{
    return GaussianAE::tag;
}

void clusterDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);

    //update the 'persistence' the current state of each of the cluster ID's
    for(vQueue::const_iterator qi = eSet.begin(); qi != eSet.end(); qi++) {
        auto vp = is_event<GaussianAE>(*qi);
        if(vp) {
            persistance[vp->ID] = vp;
        }
    }

    std::map<int, event<GaussianAE> >::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        auto v = ci->second;

        //polarity indicates the cluster has died.
        if(!v->polarity) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        double sig_x2_ = v->sigx;
        double sig_y2_ = v->sigy;
        double sig_xy_ = v->sigxy;
        double tmp = sqrt( (sig_x2_ - sig_y2_) * (sig_x2_ - sig_y2_) + 4*sig_xy_*sig_xy_ );
        double l_max = 0.5*(sig_x2_ + sig_y2_ + tmp);
        double l_min = 0.5*(sig_x2_ + sig_y2_ - tmp);

        if(l_min < -5) {
            std::cout << "l_min error: shape distorted" << std::endl;
        }

        double a = sqrt(std::fabs(l_max)) * 5;
        double b = sqrt(std::fabs(l_min)) * 5;
        double alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);

        alpha = alpha * 180 / M_PI; //convert to degrees for openCV ellipse function
        cv::ellipse(image, centr, cv::Size(a,b), alpha, 0, 360, blue, 2);

    }

}

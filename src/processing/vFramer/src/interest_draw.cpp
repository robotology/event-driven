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

const std::string interestDraw::drawtype = "AE-INT";

std::string interestDraw::getDrawType()
{
    return interestDraw::drawtype;
}

std::string interestDraw::getEventType()
{
    return LabelledAE::tag;
}

void interestDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    int r = 2;
    CvScalar c1 = CV_RGB(255, 0, 0);
    CvScalar c2 = CV_RGB(0, 255, 255);

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto v = is_event<ev::LabelledAE>(*qi);
        int px = v->x;
        int py = v->y;
        if(flip) {
            px = Xlimit - 1 - px;
            py = Ylimit - 1 - py;
        }

        cv::Point centr(px, py);
        if(v->ID == 1)
            cv::circle(image, centr, r, c1, CV_FILLED);
        else
            cv::circle(image, centr, r, c2, CV_FILLED);
    }

}

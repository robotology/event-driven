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

const std::string addressDraw::drawtype = "AE";

std::string addressDraw::getDrawType()
{
    return addressDraw::drawtype;
}

std::string addressDraw::getEventType()
{
    return AddressEvent::tag;
}

void addressDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;
        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }


        cv::Vec3b &cpc = image.at<cv::Vec3b>(y, x);

        if(!aep->polarity)
        {
            //blue
            if(cpc[0] == 1) cpc[0] = 0;   //if positive and negative
            else cpc[0] = 160;            //if only positive
            //green
            if(cpc[1] == 60) cpc[1] = 255;
            else cpc[1] = 0;
            //red
            if(cpc[2] == 0) cpc[2] = 255;
            else cpc[2] = 160;
        }
        else
        {
            //blue
            if(cpc[0] == 160) cpc[0] = 0;   //negative and positive
            else cpc[0] = 1;                //negative only
            //green
            if(cpc[1] == 0) cpc[1] = 255;
            else cpc[1] = 60;
            //red
            if(cpc.val[2] == 160) cpc[2] = 255;
            else cpc[2] = 0;
        }
    }
}

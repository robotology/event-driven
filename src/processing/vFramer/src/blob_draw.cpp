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

const std::string blobDraw::drawtype = "BLOB";

std::string blobDraw::getDrawType()
{
    return blobDraw::drawtype;
}

std::string blobDraw::getEventType()
{
    return AE::tag;
}

void blobDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto aep = as_event<AE>(*qi);
        if(!aep) continue;

        int y = aep->y;
        int x = aep->x;

        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        if(!aep->polarity)
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }

    cv::medianBlur(image, image, 5);
    cv::blur(image, image, cv::Size(5, 5));
}

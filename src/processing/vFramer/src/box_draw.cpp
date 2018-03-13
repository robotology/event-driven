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

const std::string boxDraw::drawtype = "BOX";

std::string boxDraw::getDrawType()
{
    return boxDraw::drawtype;
}

std::string boxDraw::getEventType()
{
    return BoxEvent::tag;
}

void boxDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    //update the 'persistence' the current state of each of the cluster ID's
    for(vQueue::const_iterator qi = eSet.begin(); qi != eSet.end(); qi++) {
        auto vBox = is_event<BoxEvent>(*qi);
        int x = vBox->x;
        int y = vBox->y;
        int width = vBox->width;
        int height = vBox->height;
        cv::Rect rect(x, y, width, height);
        cv::rectangle(image, rect, blue);
    }

}

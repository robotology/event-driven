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

const std::string flowDraw::drawtype = "FLOW";

std::string flowDraw::getDrawType()
{
    return flowDraw::drawtype;
}

std::string flowDraw::getEventType()
{
    return FlowEvent::tag;
}

void flowDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
{

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    double vx_mean = 0, vy_mean = 0;

    int line_thickness = 0;
    cv::Scalar line_color = CV_RGB(0,0,255);
    cv::Point p_start,p_end;

    vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window/4) break;

        auto ofp = is_event<ev::FlowEvent>(*qi);

        int x = ofp->x;
        int y = ofp->y;
        float vx = ofp->vx;
        float vy = ofp->vy;

        if(flip) {
            x = Xlimit - 1 - x;
            y = Ylimit - 1 - y;
            double temp; //i'm not sure this is correct for flipping velocity
            temp = vx;  //shouldn't it just be vx = -vx and vy = -vy? (arren.)
            vx = vy;
            vy = temp;
        }

        vx_mean += vx;
        vy_mean += vy;

        //Starting point of the line
        p_start.x = x;
        p_start.y = y;

        double hypotenuse = 15;
        double angle = atan2(vy, vx);

        //Scale the arrow by a factor of three
        p_end.x = (int) (p_start.x + hypotenuse * sin(angle));
        p_end.y = (int) (p_start.y + hypotenuse * cos(angle));

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 5*sin(angle + M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle + M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        p_start.x = (int) (p_end.x - 5*sin(angle - M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle - M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

    }

    //draw the mean velocity in the centre of the image
    vx_mean = vx_mean/eSet.size();
    vy_mean = vy_mean/eSet.size();
    p_start.x = Xlimit/2;
    p_start.y = Ylimit/2;
    double h = 15;
    double theta = atan2(vy_mean, vx_mean);
    p_end.x = (int) (p_start.x + h * sin(theta));
    p_end.y = (int) (p_start.y + h * cos(theta));

    cv::Scalar line_color2 = CV_RGB(0,255,0);
    cv::line(image, p_start, p_end, line_color2, 3, 4);

    //Draw the tips of the arrow
    p_start.x = (int) (p_end.x - 5*sin(theta + M_PI/4));
    p_start.y = (int) (p_end.y - 5*cos(theta + M_PI/4));
    cv::line(image, p_start, p_end, line_color2, 3, 4);

    p_start.x = (int) (p_end.x - 5*sin(theta - M_PI/4));
    p_start.y = (int) (p_end.y - 5*cos(theta - M_PI/4));
    cv::line(image, p_start, p_end, line_color2, 3, 4);

}

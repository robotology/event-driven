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

const std::string isoInterestDraw::drawtype = "ISO-INT";

std::string isoInterestDraw::getDrawType()
{
    return isoInterestDraw::drawtype;
}

std::string isoInterestDraw::getEventType()
{
    return LabelledAE::tag;
}

void isoInterestDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    int skip = 1 + eSet.size() / 50000;

    int r = 1;
    CvScalar c1 = CV_RGB(255, 0, 0);
    CvScalar c2 = CV_RGB(0, 255, 255);

    //ev::vQueue::const_iterator qi;
    //for(qi = eSet.begin(); qi != eSet.end(); qi += skip) {
    for(int i = eSet.size() - 1; i >= 0; i -= skip) {

        LabelledAE *cep = read_as<LabelledAE>(eSet[i]);

        //transform values
        int dt = vTime - cep->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > max_window) continue;
        dt = dt * ts_to_axis + 0.5;
        int px = cep->x;
        int py = cep->y;
        if(flip) {
            px = Xlimit - 1 - px;
            py = Ylimit - 1 - py;
        }
        int pz = dt;
        pttr(px, py, pz);
        px += imagexshift;
        py += imageyshift;

        if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight) {
            continue;
        }

        cv::Point centr(px, py);
        if(cep->ID == 1)
            cv::circle(image, centr, r, c1);
        else
            cv::circle(image, centr, r, c2);
    }

    if(!image.empty()) {
        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {
                cv::Vec3b &pixel = image.at<cv::Vec3b>(y, x);

                if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255) {

                    int px = x, py = y;
                    if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight)
                        continue;

                    isoimage.at<cv::Vec3b>(py, px) = pixel;
                }
            }
        }
    }

    image = isoimage - baseimage;

}

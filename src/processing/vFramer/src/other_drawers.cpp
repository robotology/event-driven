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

#include "vFramerLite.h"

using namespace ev;

// BLOB DRAW //
// ========= //

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

// CIRCLE DRAW //
// =========== //

const std::string circleDraw::drawtype = "CIRC";

std::string circleDraw::getDrawType()
{
    return circleDraw::drawtype;
}

std::string circleDraw::getEventType()
{
    return GaussianAE::tag;
}

void circleDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);
    cv::Scalar red = CV_RGB(255, 0, 0);

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
        if(v->polarity) continue;

        if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;
        if(v->sigxy >= v->sigx) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        cv::circle(image, centr, v->sigx - v->sigxy, red, 1.0);
        cv::circle(image, centr, v->sigx + v->sigxy, red, 1.0);

        continue;
    }

    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        auto v = ci->second;
        if(!v->polarity) continue;

        if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;
        if(v->sigxy >= v->sigx) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        cv::circle(image, centr, v->sigx - v->sigxy, blue, 1.0);
        cv::circle(image, centr, v->sigx + v->sigxy, blue, 1.0);

        continue;
    }

}

// GRAY DRAW //
// =========== //

const std::string grayDraw::drawtype = "GRAY";

std::string grayDraw::getDrawType()
{
    return grayDraw::drawtype;
}

std::string grayDraw::getEventType()
{
    return AddressEvent::tag;
}

void grayDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    image = cv::Scalar(127, 127, 127);
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
            cpc[0] = 0;
            cpc[1] = 0;
            cpc[2] = 0;
        }
        else
        {
            cpc[0] = 255;
            cpc[1] = 255;
            cpc[2] = 255;

        }
    }
}



// BLACK DRAW //
// =========== //

const std::string blackDraw::drawtype = "BLACK";

std::string blackDraw::getDrawType()
{
    return blackDraw::drawtype;
}

std::string blackDraw::getEventType()
{
    return AddressEvent::tag;
}

void blackDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    image = cv::Scalar(255, 255, 255);

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto aep = is_event<AddressEvent>(*qi);
        if(!aep) continue;

        int y = aep->y;
        int x = aep->x;
        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
    }
    /*
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        1,  1, 1,
        1, -8, 1,
        1,  1, 1);

    imgLaplacian = Mat::zeros(img.size());
    filter2D(img, imgLaplacian, kernel);

    cv::GaussianBlur(image, temp, cv::Size(3,3), 3);
    cv::addWeighted(temp, 1.5, image, -0.5, 0, image);
    */
}






// BINARY DRAW //
// =========== //

const std::string binaryDraw::drawtype = "BINARY";

std::string binaryDraw::getDrawType()
{
    return binaryDraw::drawtype;
}

std::string binaryDraw::getEventType()
{
    return AddressEvent::tag;
}

void binaryDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    image = cv::Scalar(255);

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if (dt < 0) dt += ev::vtsHelper::max_stamp;
        if ((unsigned int) dt > display_window) break;

        auto aep = is_event<AddressEvent>(*qi);
        if (!aep) continue;

        int y = aep->y;
        int x = aep->x;

        if (flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        image.at<uchar>(y, x) = 0;
    }
}

void binaryDraw::resetImage(cv::Mat &image)
{
    if(image.empty())
        image = cv::Mat(Ylimit, Xlimit, CV_8UC1);
    image.setTo(255);
}
// STEREO OVERLAY DRAW //
// =================== //

const std::string overlayStereoDraw::drawtype = "OVERLAY";

std::string overlayStereoDraw::getDrawType()
{
    return overlayStereoDraw::drawtype;
}

std::string overlayStereoDraw::getEventType()
{
    return AddressEvent::tag;
}

void overlayStereoDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
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

        if(cpc[0]==0 && cpc[1]==255 && cpc[2]==255) //skip marking already overlapping pixels
            continue;
        if(!aep->channel)
        {
            if(cpc[0]==0 && cpc[1]==0 && cpc[2]==255) //both left and right channel, mark YELLOW
            {
                cpc[0]=0;
                cpc[1]=255;
                cpc[2]=255;
            }
            else   //only left channel, mark BLUE
            {
                cpc[0]=255;
                cpc[1]=0;
                cpc[2]=0;
            }
        }
        else
        {
            if(cpc[0]==255 && cpc[1]==0 && cpc[2]==0)   //both left and right channel, mark YELLOW
            {
                cpc[0]=0;
                cpc[1]=255;
                cpc[2]=255;
            }
            else    //only right channel, mark RED
            {
                cpc[0]=0;
                cpc[1]=0;
                cpc[2]=255;
            }
        }
    }
}

// SAE DRAW //
// =========== //

const std::string saeDraw::drawtype = "SAE";

std::string saeDraw::getDrawType()
{
    return saeDraw::drawtype;
}

std::string saeDraw::getEventType()
{
    return AddressEvent::tag;
}

void saeDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        double decay = (double)dt / (double)display_window;

        auto aep = is_event<AddressEvent>(*qi);

        cv::Vec3b &cpc = image.at<cv::Vec3b>(aep->y, aep->x);
        if(cpc != white)
            continue;

        if(aep->polarity)
            cpc = aqua + (white - aqua) * decay;
        else
            cpc = violet + (white - violet) * decay;

    }
}

// ACCELEROMETER DRAW //
// ================== //

const std::string accDraw::drawtype = "ACC";

std::string accDraw::getDrawType()
{
    return accDraw::drawtype;
}

std::string accDraw::getEventType()
{
    return AddressEvent::tag;
}

void accDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    cv::Scalar pos = CV_RGB(160, 0, 160);
    cv::Scalar neg = CV_RGB(0, 60, 1);

    int radius = 4;

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > (int)display_window) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;

        if((x & 0xF) != 0xD) //accelerometer
            continue;
        x = (x & 0xFF);
        if(aep->type == 0)
            y = Ylimit - radius;
        else {
            if(y <= 127)
                y = radius + 100 + y * 100.0 / 127.0;
            else
                y = radius + 100 + ((y) * 99.0 / 127.0 - 199.78); //negative half-plane
            //x = x;
        }

        // decode the event here: i.e. do the mapping from the x value to x,y location on the image

        // get the pixel: substitute with code to draw a circle from circleDrawer
        y = Ylimit - y - 1;
        cv::Point centr(x, y);

        if(!aep->polarity)
        {
            cv::circle(image, centr, radius, pos, cv::FILLED);
        }
        else
        {
            cv::circle(image, centr, radius, neg, cv::FILLED);
        }

    }
}


// ISO (CIRC) //
// ========= //

const std::string isoCircDraw::drawtype = "ISO-CIRC";
std::string isoCircDraw::getDrawType()
{
    return isoCircDraw::drawtype;
}
std::string isoCircDraw::getEventType()
{
    return ev::GaussianAE::tag;
}
void isoCircDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);
    cv::Scalar red = CV_RGB(255, 0, 0);

    if(eSet.empty()) return;

    if(image.rows != imageheight || image.cols != imagewidth) {
        yWarning() << "Could not draw isoCircDraw. Please draw ISO first";
        return;
    }

    auto v = is_event<GaussianAE>(eSet.back());
    //if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;

    int px1 = v->x;
    int py1 = v->y;
    int pz1 = 0;

    if(flip) {
        px1 = Xlimit - 1 - px1;
        py1 = Ylimit - 1 - py1;
    }

    int px2 = px1;
    int py2 = py1;
    int pz2 = Zlimit * (v->sigy / ev::vtsHelper::max_stamp) + 0.5;

    pttr(px1, py1, pz1);
    pttr(px2, py2, pz2);

    px1 += imagexshift;
    py1 += imageyshift;
    px2 += imagexshift;
    py2 += imageyshift;

    if(px1 < 0) px1 = 0;
    if(px1 >= imagewidth) px1 = imagewidth -1;
    if(py1 < 0) py1 = 0;
    if(py1 >= imageheight) py1 = imageheight -1;
    if(px2 < 0) px2 = 0;
    if(px2 >= imagewidth) px2 = imagewidth -1;
    if(py2 < 0) py2 = 0;
    if(py2 >= imageheight) py2 = imageheight -1;

    cv::Point p1(px1, py1);
    cv::Point p2(px2, py2);

    if(v->polarity)
        cv::line(image, p1, p2, blue, 2.0);
    else
        cv::line(image, p1, p2, red, 2.0);

}

// RASTER DRAW HEAD NETWORK//
// ======= //

const std::string rasterDrawHN::drawtype = "RASTER-HN";

std::string rasterDrawHN::getDrawType()
{
    return rasterDrawHN::drawtype;
}

std::string rasterDrawHN::getEventType()
{
    return AddressEvent::tag;
}

void rasterDrawHN::initialise()
{
    Xlimit = 1024;
    Ylimit = 1024;


    time_scaler = (double) Xlimit / max_window;

    std::vector<int> output_con1 = {61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
    std::vector<int> output_con2 = {71, 72, 73, 74, 75, 76, 77, 78, 79, 80};
    std::vector<int> output_con3 = {34, 51, 54, 55, 56, 82, 58, 60, 59, 43};
    std::vector<int> output_con4 = {44, 81, 46, 47, 48, 49, 50, 83, 52, 53};
    std::vector<int> output_con5 = {84, 85, 86, 87, 88, 89, 90, 91, 42, 39};


    int y = 7;

    lines.push_back(5);
    texts.push_back("unknown");

    for(auto i : output_con5)
        rmap[i] = y++;
    lines.push_back(y + 2);
    texts.push_back("vision");
    y+= 5;

    for(auto i : output_con1)
        rmap[i] = y++;
    lines.push_back(y + 2);
    texts.push_back("yaw ring");
    y+= 5;

    for(auto i : output_con3)
        rmap[i] = y++;
    lines.push_back(y + 2);
    texts.push_back("desired yaw");
    y+= 5;

    for(auto i : output_con2)
        rmap[i] = y++;
    lines.push_back(y + 2);
    texts.push_back("pitch ring");
    y+= 5;

    for(auto i : output_con4)
        rmap[i] = y++;
    lines.push_back(y + 2);
    texts.push_back("desired pitch");
    y+= 3;

    neuron_max = y;
    neuron_min = 0;

}

void rasterDrawHN::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    for(auto i  = 0; i < lines.size(); i++) {
        int y = (lines[i] - neuron_min) * ((double)Ylimit / (1 + neuron_max - neuron_min));
        cv::line(image, cv::Point(0, y), cv::Point(Xlimit, y), aqua, 3);
        cv::putText(image, texts[i], cv::Point(0, y), cv::FONT_HERSHEY_SIMPLEX, 1.0, black, 2);
    }

    for(auto qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;

        auto aep = is_event<AddressEvent>(*qi);

        int y = rmap[aep->_coded_data];

        neuron_max = std::max(neuron_max, y);
        neuron_min = std::min(neuron_min, y);

        y = (y - neuron_min) * ((double)Ylimit / (1 + neuron_max - neuron_min));
        int x = dt * time_scaler;

        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        //image.at<cv::Vec3b>(y, x) = this->black;
        cv::circle(image, cv::Point(x, y), 5, black, cv::FILLED);
    }
}

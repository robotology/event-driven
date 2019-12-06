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
#include "event-driven/vDraw.h"
namespace ev {

// AE DRAW //
// ======= //

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
            if(cpc == aqua)
                cpc = lime;
            else if(cpc != lime)
                cpc = violet;

//            //blue
//            if(cpc[0] == 1) cpc[0] = 0;   //if positive and negative
//            else cpc[0] = 160;            //if only positive
//            //green
//            if(cpc[1] == 60) cpc[1] = 255;
//            else cpc[1] = 0;
//            //red
//            if(cpc[2] == 0) cpc[2] = 255;
//            else cpc[2] = 160;
        }
        else
        {
            if(cpc == violet)
                cpc = lime;
            else if(cpc != lime)
                cpc = aqua;
//            //blue
//            if(cpc[0] == 160) cpc[0] = 0;   //negative and positive
//            else cpc[0] = 1;                //negative only
//            //green
//            if(cpc[1] == 0) cpc[1] = 255;
//            else cpc[1] = 60;
//            //red
//            if(cpc.val[2] == 160) cpc[2] = 255;
//            else cpc[2] = 0;
        }
    }
}

// FLOW DRAW //
// ========= //

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

    int line_thickness = 1;
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
		
        vx_mean += vx;
        vy_mean += vy;

        //Starting point of the line
        p_start.x = x;
        p_start.y = y;

        double hypotenuse = 15;
        double angle = atan2(vy, vx);

        //Scale the arrow by a factor of three
        p_end.x = (int) (p_start.x + hypotenuse * cos(angle));
        p_end.y = (int) (p_start.y + hypotenuse * sin(angle));

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 5*cos(angle + M_PI/4));
        p_start.y = (int) (p_end.y - 5*sin(angle + M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        p_start.x = (int) (p_end.x - 5*cos(angle - M_PI/4));
        p_start.y = (int) (p_end.y - 5*sin(angle - M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

    }

    //draw the mean velocity in the centre of the image
    vx_mean = vx_mean/eSet.size();
    vy_mean = vy_mean/eSet.size();
    p_start.x = Xlimit/2;
    p_start.y = Ylimit/2;
    double h = 15;
    double theta = atan2(vy_mean, vx_mean);
    p_end.x = (int) (p_start.x + h * cos(theta));
    p_end.y = (int) (p_start.y + h * sin(theta));

    cv::Scalar line_color2 = CV_RGB(0,255,0);
    cv::line(image, p_start, p_end, line_color2, 3, 4);

    //Draw the tips of the arrow
    p_start.x = (int) (p_end.x - 5*cos(theta + M_PI/4));
    p_start.y = (int) (p_end.y - 5*sin(theta + M_PI/4));
    cv::line(image, p_start, p_end, line_color2, 3, 4);

    p_start.x = (int) (p_end.x - 5*cos(theta - M_PI/4));
    p_start.y = (int) (p_end.y - 5*sin(theta - M_PI/4));
    cv::line(image, p_start, p_end, line_color2, 3, 4);

}

// SKIN DRAW //
// ========= //

const std::string skinDraw::drawtype = "SKIN";

std::string skinDraw::getDrawType()
{
    return skinDraw::drawtype;
}

std::string skinDraw::getEventType()
{
    return SkinEvent::tag;
}

void skinDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    

    //int radius = 5;
    
    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<SkinEvent>(*qi);

        int index = aep->taxel;

        
        if(tmap.pos.find(index) != tmap.pos.end()){
            //yInfo() << "\n Index " << index <<"  mapped to   "<< std :: get<0>(tmap[index]) << std :: get<1>(tmap[index]) << "\n";
            int x = tmap.xoffset + tmap.scaling* std :: get<0>(tmap.pos[index]);
            int y =  tmap.yoffset + tmap.scaling* std :: get<1>(tmap.pos[index]);
            cv::Point centr(x, y);
            if(!aep->polarity)
            {
                cv::circle(image, centr, radius, aqua, CV_FILLED,CV_AA);
            }
            else
            {
                cv::circle(image, centr, radius, violet, CV_FILLED,CV_AA);
            }
        }
        else
        {
           // yWarning() << "\n Index " << index <<"not mapped! \n";
        }
        

    }
}

// SKIN SAMPLE DRAW //
// ================ //

const std::string skinsampleDraw::drawtype = "SAMPLE";

std::string skinsampleDraw::getDrawType()
{
    return skinsampleDraw::drawtype;
}

std::string skinsampleDraw::getEventType()
{
    return SkinSample::tag;
}

void skinsampleDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    int radius_min = radius;
    int radius_max = 3*radius;

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = as_event<SkinSample>(*qi);
        if(!aep) {
            yError() << "data corruption";
            continue;
        }


        int index = aep->taxel;
        

        if(tmap.pos.find(index) != tmap.pos.end()){
            // yInfo() << "\n Index = " << index ;
            // std :: cout << "\n Index " << index <<"mapped to "<< x << y << "\n";
            if(aep->value > noise){
                int x = tmap.xoffset + tmap.scaling* std :: get<0>(tmap.pos[index]);
                int y =  tmap.yoffset +tmap.scaling* std :: get<1>(tmap.pos[index]);
                cv::Point centr(x, y);
                
                int radius = radius_min + (radius_max-radius_min)* aep->value /max_value;
                if(radius>radius_max){
                    radius = radius_max;
                }
                
                cv::circle(image, centr, radius, red, 1, CV_AA);
            }
        }
        else{
            //yWarning() << "\n Index " << index <<"not mapped! \n";
            //std :: cout << "\n Index " << index <<"not mapped! \n";
        }
    }
}



// SKIN TAXEL DRAW //
// ================ //

const std::string taxelsampleDraw :: drawtype = "TAXEL_SAMPLE";

std::string  taxelsampleDraw :: getDrawType()
{
    return taxelsampleDraw::drawtype;
}

std::string  taxelsampleDraw ::getEventType()
{
    return SkinSample::tag;
}
//--------------------------------



void taxelsampleDraw :: draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{   

    if(eSet.empty()) return;
    ev::vQueue::const_reverse_iterator qi;
    // static float T =0;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto aep = is_event<SkinSample>(*qi);

        int noise = 2500;

        int index = aep->taxel;

        float sample = aep->value/float(noise)*20;

        if(index ==163){

            cv::Point dot(10 + dt*vtsHelper::tsscaler*500, Ylimit - sample);

            cv::circle(image, dot, 1, red, CV_FILLED);
        }

    }

}

const std::string taxeleventDraw :: drawtype = "TAXEL_EVENT";

std::string  taxeleventDraw :: getDrawType()
{
    return taxeleventDraw::drawtype;
}

std::string  taxeleventDraw ::getEventType()
{
    return SkinEvent::tag;
}
//--------------------------------

void taxeleventDraw :: draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    ev::vQueue::const_reverse_iterator qi;
    
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
       // if((unsigned int)dt > display_window) break;


        auto aep = is_event<SkinEvent>(*qi);

        int index = aep->taxel;



        if(index ==163){

            cv::Point dot(10 + dt*vtsHelper::tsscaler*500, 10);

            if(!aep->polarity)
            {
                cv::circle(image, dot, 1, aqua, CV_FILLED);
            }
            else
            {
                cv::circle(image, dot, 1, violet, CV_FILLED);
            }

        }
    }

}




// IMU SAMPLE DRAW //
// ================ //

const std::string imuDraw::drawtype = "IMU";

std::string imuDraw::getDrawType()
{
    return imuDraw::drawtype;
}

std::string imuDraw::getEventType()
{
    return IMUevent::tag;
}

void imuDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    auto radius = 2;
    auto axes = cv::Size(radius, radius);
    auto centre = cv::Point(Xlimit/2, Ylimit/2);
    auto lin_scaler = std::min(Xlimit, Ylimit) * 0.5 / IMUevent::_max_value;
    auto circ_scaler = 360.0 / IMUevent::_max_value;

    cv::line(image, centre, centre - cv::Point(0, Ylimit * 0.25),
             black, 1);
    cv::line(image, centre, centre + cv::Point(Ylimit * 0.25, 0),
             black, 1);
    cv::line(image, centre, centre + cv::Point(Ylimit * 0.25 * 0.71, Ylimit * 0.25 * 0.71),
             black, 1);

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    int i = 0;
    for(auto qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        if(i++ > 10) break;

        auto aep = is_event<IMUevent>(*qi);

        switch(aep->sensor) {
        case 0: { //acc x
            auto p_end = centre - cv::Point(0, aep->value * lin_scaler);
            cv::line(image, centre, p_end, violet, 4);
            break; }
        case 1: {//acc y
            auto p_end = centre + cv::Point(aep->value * lin_scaler, 0);
            cv::line(image, centre, p_end, violet, 4);
            break;}
        case 2: {//acc z
            auto p_end = centre + cv::Point(aep->value * lin_scaler * 0.71,
                                            aep->value * lin_scaler * 0.71);
            cv::line(image, centre, p_end, violet, 4);
            break;}
        case 3: {//rot x
            auto pos = centre - cv::Point(0, Ylimit * 0.25);
            cv::ellipse(image, pos, axes, 0, 0, aep->value * circ_scaler, orange, CV_FILLED);
            break;}
        case 4: {//rot y
            auto pos = centre + cv::Point(Ylimit * 0.25, 0);
            cv::ellipse(image, pos, axes, 0, 0, aep->value * circ_scaler, orange, CV_FILLED);
            break;}
        case 5: {//rot z
            auto pos = centre + cv::Point(Ylimit * 0.25*0.71, Ylimit * 0.25 * 0.71);
            cv::ellipse(image, pos, axes, 0, 0, aep->value * circ_scaler, orange, CV_FILLED);
            break;}
        case 6: {//temp
            //centre = cv::Point(radius, radius);
            //cv::ellipse(image, centre, axes, 0, 0, aep->value * circ_scaler, this->aqua, CV_FILLED);
            break;}
        case 7: {//mag x
            break;}
        case 8: {//mag y
            break;}
        case 9: {//mag z
            break;}

        }
    }
}

// RASTER DRAW //
// ======= //

const std::string rasterDraw::drawtype = "RASTER";

std::string rasterDraw::getDrawType()
{
    return rasterDraw::drawtype;
}

std::string rasterDraw::getEventType()
{
    return AddressEvent::tag;
}

void rasterDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    for(auto qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;

        auto aep = is_event<AddressEvent>(*qi);

        num_neurons = std::max(num_neurons, aep->_coded_data + 1);

        int y = aep->_coded_data * ((double)Ylimit / num_neurons);
        int x = dt * time_scaler;

        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }

        image.at<cv::Vec3b>(y, x) = this->black;
    }
}

// COCHLEA DRAW //
// ======= //

const std::string cochleaDraw::drawtype = "EAR";

std::string cochleaDraw::getDrawType()
{
    return cochleaDraw::drawtype;
}

std::string cochleaDraw::getEventType()
{
    return AddressEvent::tag;
}

void cochleaDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {

        int dt = vTime - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<AddressEvent>(*qi);

        int x  = aep->x * (Xlimit-1) / 64;
        int y = 120;

        cv::Vec3b c;
        if(aep->polarity)
            c = violet;
        else
            c = aqua;

        cv::circle(image, cv::Point(x, y), 5, c, CV_FILLED);

    }
}



// LABELLED-AE DRAW //
// ================ //

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

    //TODO: bring these params in from configuration
    double alpha = 0.1; // proportion of shading for a single event
    double notAlpha = 1.0 - alpha;
    int r = 2; // radius
    int d = r*2+1; // diameter
    CvScalar c1 = CV_RGB(255, 0, 0);
    CvScalar c2 = CV_RGB(60, 0, 255);
    CvScalar c;

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
        if(v->ID == 1)
            c = c1;
        else
            c = c2;
        //2019_09_17 Sim: instead of adding a filled circle, shade a square where side of square is d=diameter of the corresponding circle
        //cv::Point centr(px, py);
        //cv::circle(image, centr, r, c, CV_FILLED);
        cv::Rect roiRect(std::max(px-r, 0), std::max(py-r, 0), std::min(d, Xlimit-px), std::min(d, Ylimit-py));
        cv::Mat roi = image(roiRect);
        cv::Mat color(roi.size(), CV_8UC3, c);
        cv::addWeighted(color, alpha, roi, notAlpha , 0.0, roi);

    }

}

// GAUSSIAN-AE DRAW //
// ================ //

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
            yWarning() << "l_min error: shape distorted";
        }

        double a = sqrt(std::fabs(l_max)) * 5;
        double b = sqrt(std::fabs(l_min)) * 5;
        double alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);

        alpha = alpha * 180 / M_PI; //convert to degrees for openCV ellipse function
        cv::ellipse(image, centr, cv::Size(a,b), alpha, 0, 360, blue, 2);

    }

}

} //namespace ev::

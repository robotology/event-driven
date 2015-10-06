/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "vCircleObserver.h"
#include <math.h>
#include <cv.h>


/*//////////////////////////////////////////////////////////////////////////////
  GEOMETRIC OBSERVER
  ////////////////////////////////////////////////////////////////////////////*/
vGeoCircleObserver::vGeoCircleObserver()
{
    spatialRadius = 32;
    inlierThreshold = 5;
    angleThreshold = 0.1;
    radiusThreshold = 1.5;

    window = 0;

}

/******************************************************************************/
vGeoCircleObserver::~vGeoCircleObserver()
{
    if(window) delete window;
}

/******************************************************************************/
void vGeoCircleObserver::init(int width, int height, int temporalRadius,
                           int spatialRadius, int inlierThresh,
                           double angleThresh, double radThresh)
{
    this->spatialRadius = spatialRadius;
    this->inlierThreshold = inlierThresh;
    this->angleThreshold = angleThresh;
    this->radiusThreshold = radThresh;

    this->window = new emorph::vWindow(width, height, temporalRadius, false);
}

/******************************************************************************/
void vGeoCircleObserver::addEvent(emorph::vEvent &event) {
    window->addEvent(event);
}
/******************************************************************************/
int vGeoCircleObserver::flowcircle(double &cx, double &cy, double &cr)
{

    if(!window) return -1;
    emorph::FlowEvent *vr = window->getMostRecent()->getAs<emorph::FlowEvent>();
    if(!vr) return -1;

    //get the recent event and the surface
    emorph::vQueue q = window->getSMARTSURF(spatialRadius);
    if(q.size() < inlierThreshold) return -1;

    double main_m = vr->getVy() / vr->getVx();
    double main_b = vr->getY() - main_m * vr->getX();

    int max_inliers = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::FlowEvent * v = (*qi)->getAs<emorph::FlowEvent>();
        if(!v) continue;
        if(v == vr) continue;

        int y = v->getY();
        int x = v->getX();
        double dtdy = v->getVy();
        double dtdx = v->getVx();

        double flowmag = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0)) /
                emorph::vtsHelper::tstosecs();
        if(vr->getStamp() - v->getStamp() > flowmag) continue;

        double vm = dtdy / dtdx;
        double vb = y - vm * x;
        if(vm == main_m) continue;

        double xX = (vb - main_b) / (main_m - vm);
        double yX = (main_m*vb - vm*main_b) / (main_m - vm);

        if(vr->getVx() * (xX - vr->getX()) * dtdx * (xX - x) < 0) continue;
        if(vr->getVy() * (yX - vr->getY()) * dtdy * (yX - y) < 0) continue;

        //if the radius doesn't agree between the two points
        double main_rad = sqrt(pow(vr->getX()-xX, 2.0) +
                               pow(vr->getY() - yX, 2.0));
        double rad1 = sqrt(pow(x-xX, 2.0) + pow(y - yX, 2.0));
        if(fabs(main_rad - rad1) < radiusThreshold) continue;

        int inliers = 0;
        for(emorph::vQueue::iterator qi2 = q.begin(); qi2 != q.end(); qi2++) {
            emorph::FlowEvent * v2 = (*qi2)->getAs<emorph::FlowEvent>();
            if(!v2) continue;

            //only allow points with the same side of xX to main_x
            double vm2 = v2->getVy() / v2->getVx();
            double vb2 = v2->getY() - vm2 * v2->getX();
            if(vm2 == main_m) continue;
            double xX2 = (vb2 - main_b) / (main_m - vm2);
            double yX2 = (main_m*vb2 - vm2*main_b) / (main_m - vm2);

            if(vr->getVx() * (xX2 - vr->getX()) * v2->getVx() *
                    (xX2 - v2->getX()) < 0) continue;
            if(vr->getVy() * (yX2 - vr->getY()) * v2->getVy() *
                    (yX2 - v2->getY()) < 0) continue;


            //angle1
            double angle1 = atan2(v2->getVy(), v2->getVx());
            double angle2 = atan2(yX - v2->getY(), xX - v2->getX());
            if(v2->getVx()*(xX-v2->getX()) < 0 ||
                    v2->getVy()*(yX-v2->getY()) < 0) {
                angle2 = atan2(v2->getY() - yX, v2->getX() - xX);
            }

            //this needs to account for wrap
            double adiff = fabs(angle1 - angle2);
            adiff = std::min(adiff, fabs(angle1 - angle2 - 6.18));
            adiff = std::min(adiff, fabs(angle1 - angle2 + 6.18));

            double rad2 = sqrt(pow(v2->getX()-xX, 2.0) +
                               pow(v2->getY() - yX, 2.0));
            double rdiff = fabs(main_rad - rad2);

            if(adiff < angleThreshold && rdiff < radiusThreshold) inliers++;

        }

        if(inliers > max_inliers) {
            max_inliers = inliers;
            cx = xX;
            cy = yX;
        }


    }

    cr = sqrt(pow(cx - vr->getX(), 2.0) + pow(cy - vr->getY(), 2.0));

    return max_inliers;

}

/******************************************************************************/
bool vGeoCircleObserver::calculateCircle(double x1, double x2, double x3,
                                      double y1, double y2, double y3,
                                      double &cx, double &cy, double &cr)
{



    //if we are all on the same line we can't compute a circle
    if(y1 == y2 && y1 == y3) return false;
    if(x1 == x2 && x1 == x3) return false;

    //or if any of the points are duplicate
    if(x1 == x2 && y1 == y2) return false;
    if(x1 == x3 && y1 == y3) return false;
    if(x2 == x3 && y2 == y3) return false;

    //make sure x2 is different to x1 and x3 (else we divide by 0 later)
    if(x2 == x1) {
        double tx = x3, ty = y3;
        x3 = x2; y3 = y2;
        x2 = tx; y2 = ty;
    } else if(x2 == x3) {
        double tx = x1, ty = y1;
        x1 = x2; y1 = y2;
        x2 = tx; y2 = ty;
    }


    //calculate the circle from the 3 points
    double ma = (y2 - y1) / (x2 - x1);
    double mb = (y3 - y2) / (x3 - x2);

    if(ma == mb) return false;
    if(ma != ma || mb != mb) {
        std::cout << "error: (ma|mb) == NaN" << std::endl;
    }

    cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) -
                        ma * (x2 + x3)) / (2 * (mb - ma));
    if(ma)
        cy = -1 * (cx - (x1+x2)/2.0)/ma + (y1+y2)/2.0;
    else
        cy = -1 * (cx - (x2+x3)/2.0)/mb + (y2+y3)/2.0;


    if(cx != cx) {
        std::cout << "error: cx == NaN" << std::endl;
    }
    if(cx == INFINITY || cx == -INFINITY) {
        std::cout << "error: cx == INF" << std::endl;
    }

    return true;

}

/*//////////////////////////////////////////////////////////////////////////////
  HOUGH CIRCLE
  ////////////////////////////////////////////////////////////////////////////*/
vHoughCircleObserver::vHoughCircleObserver()
{

    qType = "Fixed";
    qlength = 2000;
    qduration = 200000;
    useFlow = false;
    r1 = 10;
    r2 = 36;
    rsize = r2 - r1;
    height = 128;
    width = 128;

    valid = false;
    xc = 0; yc = 0; rc = 0; valc = 0;

    for(int r = 0; r < rsize; r++) {
        H.push_back(new yarp::sig::Matrix(height, width));
        posThreshs.push_back(1.0 / (int)(6.2831853 * (r+r1) + 0.5));
        negThreshs.push_back(-1 * posThreshs.back());
    }

    x_max = 0; y_max = 0; r_max = 0;
    obs_max = &(*H[r_max])[y_max][x_max];

    rot.push_back(0); rot.push_back(M_PI);
    err.push_back(-5.0*M_PI/180.0); err.push_back(5.0*M_PI/180.0);


}

/******************************************************************************/
vHoughCircleObserver::~vHoughCircleObserver()
{
    for(int r = 0; r < rsize; r++) {
        delete H[r];
    }
}

/******************************************************************************/
void vHoughCircleObserver::addEvent(emorph::vEvent &event)
{
    if(qType == "Fixed")
        addEventFixed(event);
    else if(qType == "Lifetime")
        addEventLife(event);
}

/******************************************************************************/
void vHoughCircleObserver::addEventFixed(emorph::vEvent &event)
{
    valid = false;
    emorph::AddressEvent *v = event.getUnsafe<emorph::AddressEvent>();

    int cx = v->getX(); int cy = v->getY();
    emorph::vQueue::iterator i = FIFO.begin();
    while(i != FIFO.end()) {
        //v = (*i)->getAs<emorph::AddressEvent>();
        v = (*i)->getUnsafe<emorph::AddressEvent>();

        bool samelocation = v->getX() == cx && v->getY() == cy;

        if(samelocation) {
            updateH(*v, -1);
            i = FIFO.erase(i);
        } else {
            i++;
        }
    }

    //if successful add it to the FIFO and check to remove others
    FIFO.push_front(&event);
    while(FIFO.size() > qlength) {
        updateH(*FIFO.back(), -1);
        FIFO.pop_back();
    }
    //add this event to the hough space
    valid = updateH(event, 1);

}

/******************************************************************************/
void vHoughCircleObserver::addEventLife(emorph::vEvent &event)
{

    //lifetime requires a flow event only
    valid = false;
    emorph::FlowEvent *v = event.getAs<emorph::FlowEvent>();
    if(!v) return;

    int cts = v->getStamp();
    int cx = v->getX(); int cy = v->getY();
    emorph::vQueue::iterator i = FIFO.begin();
    while(i != FIFO.end()) {
        v = (*i)->getUnsafe<emorph::FlowEvent>();
        int modts = cts;
        if(cts < v->getStamp()) //we have wrapped
            modts += emorph::vtsHelper::maxStamp();

        bool samelocation = v->getX() == cx && v->getY() == cy;

        if(modts > v->getDeath() || samelocation) {
            updateH(*v, -1);
            i = FIFO.erase(i);
        } else {
            i++;
        }
    }

    //add to queue
    FIFO.push_front(&event);

    //add this event to the hough space
    valid = updateH(event, 1);
    if(!valid) return;



}

/******************************************************************************/
bool vHoughCircleObserver::updateH(emorph::vEvent &event, int val)
{

    if(useFlow) {
        emorph::FlowEvent *v = event.getAs<emorph::FlowEvent>();
        if(!v) return false;
        if(val > 0)
            updateHFlowAngle(v->getX(), v->getY(), posThreshs, v->getVx(),
                             v->getVy());
        else
            updateHFlowAngle(v->getX(), v->getY(), negThreshs, v->getVx(),
                             v->getVy());
    } else {
        emorph::AddressEvent *v = event.getAs<emorph::AddressEvent>();
        if(!v) return false;
        if(val > 0)
            updateHAddress(v->getX(), v->getY(), posThreshs);
        else
            updateHAddress(v->getX(), v->getY(), negThreshs);
    }
    return true;
}

/******************************************************************************/
void vHoughCircleObserver::updateHAddress(int xv, int yv,
                                          std::vector<double> &threshs)
{
    int P = 1;
    //if(val > 0) valc = 0; //val > 0 : we are looking for a circle
    for(int r = 0; r < rsize; r++) {
        int R = r+r1;
        double R2 = pow(R, 2.0);
        int xstart = std::max(0, xv - R);
        int xend = std::min(width-1, xv + R);
        for(int x = xstart; x <= xend; x++) {
            //(xv-xc)^2 + (yv - yc)^2 = R^2
            int deltay = (int)sqrt(R2 - pow(x - xv, 2.0));

            for(int s = -P; s <= P; s++) {
                int y = yv + deltay + s;
                if(y > 127 || y < 0) continue;
                (*H[r])[y][x] += threshs[r];
                if((*H[r])[y][x] > *obs_max) {
                    r_max = r+r1; y_max = y; x_max = x;
                    obs_max = &(*H[r])[y][x];
                }

                if(!deltay) continue; //don't double up on same space
                if(deltay - P <= -deltay + s) continue;


                y = yv - deltay + s;
                if(y > 127 || y < 0) continue;
                (*H[r])[y][x] += threshs[r];
                if((*H[r])[y][x] > *obs_max) {
                    r_max = r+r1; y_max = y; x_max = x;
                    obs_max = &(*H[r])[y][x];
                }
            }

        }
    }
}

/******************************************************************************/
void vHoughCircleObserver::updateHFlow(int xv, int yv,
                                       std::vector<double> &threshs,
                                       double dtdx, double dtdy)
{
    int P = 2;
    double theta1 = atan2(dtdx, dtdy);
    //if(val > 0) valc = 0;
    for(int r = 0; r < rsize; r++) {
        int R = r+r1;


        int x_base = R * cos(theta1) + xv;
        int y_base = R * sin(theta1) + yv;

        for(int y = y_base-P; y < y_base+P; y++) {
            for(int x = x_base-P; x < x_base+P; x++) {

                if(x < 0 || x > width-1 || y < 0 || y > height-1) continue;

                (*H[r])[y][x] += threshs[r];
                if((*H[r])[y][x] > *obs_max) {
                    r_max = r+r1; y_max = y; x_max = x;
                    obs_max = &(*H[r])[y][x];
                }
            }
        }

        theta1 = atan2(dtdx, dtdy) + M_PI;

        x_base = R * cos(theta1) + xv;
        y_base = R * sin(theta1) + yv;

        for(int y = y_base-P; y < y_base+P; y++) {
            for(int x = x_base-P; x < x_base+P; x++) {

                if(x < 0 || x > width-1 || y < 0 || y > height-1) continue;

                (*H[r])[y][x] += threshs[r];
                if((*H[r])[y][x] > *obs_max) {
                    r_max = r+r1; y_max = y; x_max = x;
                    obs_max = &(*H[r])[y][x];
                }
            }
        }



    }
}

/******************************************************************************/
void vHoughCircleObserver::updateHFlowAngle(int xv, int yv,
                                            std::vector<double> &threshs,
                                            double dtdx, double dtdy)
{
    std::vector<double>::iterator rotval;

    double thetaBase = atan2(dtdx, dtdy);
    int P = 1;
    //do for multiple scales
    for(int r = 0; r < rsize; r++) {
        int R = r+r1;

        int R2 = pow(R, 2.0);
        //do for 'forward' flow and flow rotated by 180
        for(rotval = rot.begin(); rotval != rot.end(); rotval++) {

            //calculate centre position in hough space
            double thetaExact = thetaBase + *rotval;
            if(thetaExact > M_PI) thetaExact -= 2 * M_PI; //keep between -pi->pi

            if(fabs(thetaExact - M_PI_2) < M_PI_4 ||
                    fabs(thetaExact + M_PI_2) < M_PI_4) {

                int ysign = thetaExact < 0 ? -1 : 1;

                int xStart = xv + R * cos(thetaExact+err[0]);
                int xEnd   = xv + R * cos(thetaExact+err[1]);

                if(xEnd < xStart) {
                    int temp = xStart;
                    xStart = xEnd;
                    xEnd = temp;
                }

                xEnd = std::min(width-1, xEnd);
                xStart = std::max(0, xStart);

                for(int x = xStart; x <= xEnd; x++) {

                    int deltay = (int)sqrt(R2 - pow(x - xv, 2.0)) * ysign;

                    for(int s = -P; s <= P; s++) {
                        int y = yv + deltay + s;
                        if(y > height-1 || y < 0) continue;
                        (*H[r])[y][x] += threshs[r];
                        if((*H[r])[y][x] > *obs_max) {
                            r_max = r+r1; y_max = y; x_max = x;
                            obs_max = &(*H[r])[y][x];
                        }
                    }
                }
            } else {
                int xsign = fabs(thetaExact) > M_PI_2 ? -1 : 1;

                int yStart = yv + R * sin(thetaExact + err[0]);
                int yEnd   = yv + R * sin(thetaExact + err[1]);

                if(yEnd < yStart) {
                    int temp = yStart;
                    yStart = yEnd;
                    yEnd = temp;
                }

                yEnd = std::min(height-1, yEnd);
                yStart = std::max(0, yStart);

                for(int y = yStart; y <= yEnd; y++) {

                    int deltax = (int)sqrt(R2 - pow(y - yv, 2.0)) * xsign;

                    for(int s = -P; s <= P; s++) {
                        int x = xv + deltax + s;
                        if(x > width-1 || x < 0) continue;
                        (*H[r])[y][x] += threshs[r];
                        if((*H[r])[y][x] > *obs_max) {
                            r_max = r+r1; y_max = y; x_max = x;
                            obs_max = &(*H[r])[y][x];
                        }
                    }
                }

            }
        }
    }
}

/******************************************************************************/
double vHoughCircleObserver::getMaximum(int &x, int &y, int &r)
{
    double val = 0;
    for(int ir = 0; ir < rsize; ir++) {
        for(int iy = 0; iy < height; iy++) {
            for(int ix = 0; ix < width; ix++) {
                if((*H[ir])[iy][ix] > val) {
                    x = ix;
                    y = iy;
                    r = ir;
                    val = (*H[ir])[iy][ix];
                }
            }
        }
    }

    if(val == 0) {
        x = 0; y = 0; r = 0;
    }

    r = r + r1;
    return val;
}

/******************************************************************************/

yarp::sig::ImageOf<yarp::sig::PixelMono> vHoughCircleObserver::makeDebugImage(
        int r)
{

    yarp::sig::ImageOf<yarp::sig::PixelMono> canvas;
    canvas.resize(width, height);
    canvas.zero();
    if(r < r1 || r > r2) return canvas;
    r = r - r1;

    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            if((*H[r])[y][x] > 0.3)
                canvas(y,127- x) = 255.0;
            else
                canvas(y,127- x) = 255.0 * (*H[r])[y][x] / 0.3;

        }
    }

    return canvas;
}

/******************************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelMono> vHoughCircleObserver::makeDebugImage2()
{

    int bx, by, br;
    getMaximum(bx, by, br);
    return makeDebugImage(br);

}



/******************************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelBgr> vHoughCircleObserver::makeDebugImage3(
        int s)
{

    emorph::AddressEvent a;
    yarp::sig::ImageOf<yarp::sig::PixelBgr> canvas;
    canvas.resize(width*s, height*s);\

    for(int v = 0; v < height*s; v++) {
        for(int u = 0; u < width*s; u++) {
            yarp::sig::PixelBgr &p = canvas.pixel(v, u);
            p.b = 255; p.g = 255; p.r = 255;
        }
    }

    emorph::vQueue::const_iterator qi;
    for(qi = FIFO.begin(); qi != FIFO.end(); qi++) {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;

        for(int sv = 0; sv < s; sv++) {
            for(int su = 0; su < s; su++) {

                yarp::sig::PixelBgr &p =
                        canvas.pixel(v->getY()*s+sv,(width - v->getX()-1)*s+su);

                //cv::Vec3b cpc = canvas.at<cv::Vec3b>(v->getX(), v->getY());

                if(!v->getPolarity())
                {
                    //blue
                    if(p.b == 1) p.b = 0;   //if positive and negative
                    else p.b = 160;            //if only positive
                    //green
                    if(p.g == 60) p.g = 255;
                    else p.g = 0;
                    //red
                    if(p.r == 0) p.r = 255;
                    else p.r = 160;
                }
                else
                {
                    //blue
                    if(p.b == 160) p.b = 0;   //negative and positive
                    else p.b = 1;                //negative only
                    //green
                    if(p.g == 0) p.g = 255;
                    else p.g = 60;
                    //red
                    if(p.r == 160) p.r = 255;
                    else p.r = 0;
                }
            }
        }

    }

    cv::Mat image = (IplImage *)canvas.getIplImage();
    for(qi = FIFO.begin(); qi != FIFO.end(); qi++) {
        emorph::FlowEvent *v = (*qi)->getAs<emorph::FlowEvent>();
        if(!v) continue;

        //Starting point of the line
        cv::Point p_start(v->getY()*s, (width - v->getX() - 1)*s);
        cv::Point p_end(p_start.x + v->getVx()*s, p_start.y - v->getVy()*s);
        cv::line(image, p_start, p_end, CV_RGB(255, 0, 0), 1, CV_AA);

        double angle = atan2(v->getVx(), v->getVy());

        //Draw the main line of the arrow


        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 7*sin(angle + M_PI/4));
        p_start.y = (int) (p_end.y + 7*cos(angle + M_PI/4));
        cv::line(image, p_start, p_end, CV_RGB(255, 0, 0), 1, CV_AA);

        p_start.x = (int) (p_end.x - 7*sin(angle - M_PI/4));
        p_start.y = (int) (p_end.y + 7*cos(angle - M_PI/4));
        cv::line(image, p_start, p_end, CV_RGB(255, 0, 0), 1, CV_AA);

    }

    if(*obs_max > 0.29) {
        cv::circle(image, cv::Point(y_max, height - x_max - 1)*s, r_max*s,
                   CV_RGB(0, 0, 255), CV_FILLED);
    }

    return canvas;

}

yarp::sig::ImageOf<yarp::sig::PixelBgr> vHoughCircleObserver::makeDebugImage4()
{

    int scale = 4;
    yarp::sig::ImageOf<yarp::sig::PixelMono>  h = makeDebugImage2();
    yarp::sig::ImageOf<yarp::sig::PixelBgr>   e = makeDebugImage3(scale);
    yarp::sig::ImageOf<yarp::sig::PixelBgr> final;
    final.resize(e.width(), e.height()*2);
    cv::Mat cvfinal = (IplImage *)final.getIplImage();
    cv::Mat cvh = (IplImage *)h.getIplImage();
    cv::Mat cve = (IplImage *)e.getIplImage();

    //resize the hough image
    cv::Mat hbig(cvh.size() * scale, CV_8U);
    cv::resize(cvh, hbig, cvh.size() * scale);
    //color the hough image
    cv::Mat hbigcol(hbig.size(), CV_8UC3);
    cv::cvtColor(hbig, hbigcol, CV_GRAY2BGR);

    //put the two images in the one
    //cv::resize(cve, (cv::Mat)cvfinal(cv::Rect(0, 0, e.width(), e.height())), cv::Size(e.width(), e.height()));
    //cv::resize(hbigcol, cvfinal(cv::Rect(0, e.height(), e.width(), e.height())), cv::Size(e.width(), e.height()));
    //cv::OutputArray *cvfinaloa = cvfinal(cv::Rect(0, 0, e.width(), e.height()));
    //cv::OutputArray a = cvfinal(cv::Rect(0, 0, e.width(), e.height()));
    //cv::OutputArray * tempoa = (cv::OutputArray *)&temp;

    //workaround for opencv3.1 rather than copyTo used below [untested]
    for(int j = 0; j < e.height(); j++) {
        for(int i = 0; i < e.width(); i++) {
            cvfinal.at<cv::Vec3b>(j, i) = cve.at<cv::Vec3b>(j, i);
            cvfinal.at<cv::Vec3b>(j, i+e.width()) = hbigcol.at<cv::Vec3b>(j, i);
        }
    }


    //cve.copyTo(cvfinal(cv::Rect(0, 0, e.width(), e.height())));
    //hbigcol.copyTo(cvfinal(cv::Rect(0, e.height(), e.width(), e.height())));

    return final;

}

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

vCircleTracker::vCircleTracker()
{
    svPos = 5;
    svSiz = 2;
    filter = 0;
    active = false;

}

/******************************************************************************/
vCircleTracker::~vCircleTracker()
{
    if(filter) delete filter;
}

/******************************************************************************/
void vCircleTracker::init(double svPos, double svSiz, double zvPos,
                          double zvSiz)
{
    this->svPos = svPos;
    this->svSiz = svSiz;
    active = false;

    //these two matrices are dependent on dt so we have to update them everytime
    yarp::sig::Matrix A(6, 6); A = 0;
    A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1;
    A(3, 3) = 1; A(4, 4) = 1; A(5, 5) = 1;
   // A(6, 6) = 1; A(7, 7) = 1; A(8, 8) = 1;
    //we need to update A: (0, 3), (1, 4) and (2, 5) based on delta t

    yarp::sig::Matrix Q(6, 6); Q = 0;
    //we update Q depending on delta t

    //these two are constant
    yarp::sig::Matrix H(6, 6); H = 0;
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;

    yarp::sig::Matrix R(6, 6); R = 0;
    R(0, 0) = zvPos; R(1, 1) = zvPos; R(2, 2) = zvSiz;

    filter = new iCub::ctrl::Kalman(A, H, Q, R);
}

/******************************************************************************/
bool vCircleTracker::startTracking(double xz, double yz, double rz)
{
    if(!filter) return false;
    yarp::sig::Vector x0(6, 0.0); x0[0] = xz; x0[1] = yz; x0[2] = rz;
    yarp::sig::Matrix P0 = filter->get_R();
    P0(3, 3) = P0(0, 0); P0(4, 4) = P0(1, 1); P0(5, 5) = P0(2, 2);
    //P0(4, 4) = P0(0, 0); P0(5, 5) = P0(1, 1); P0(6, 6) = P0(2, 2);
    filter->init(x0, P0);
    active = true;
    return true;
}

/*********************************s********************************************/
double vCircleTracker::predict(double dt)
{

    if(!active) return 0;
    //update the tracker position
    double dtmod = dt > 1.0 ? 0.0 : 1.0 - 0.5*pow(dt, 2.0);
    yarp::sig::Matrix A = filter->get_A();
    A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
    //A(3, 6) = dt; A(4, 7) = dt; A(5, 8) = dt;
    //A(0, 6) = dt2; A(1, 7) = dt2; A(2, 8) = dt2
    A(3, 3) = dtmod; A(4, 4) = dtmod; A(5, 5) = dtmod;
    filter->set_A(A);

    yarp::sig::Matrix Q = filter->get_Q();
    //Q(6, 6) = svPos*dt2; Q(7, 7) = svPos*dt2; Q(8, 8) = svSiz*dt2;
    //Q(0, 0) = svPos*dt; Q(1, 1) = svPos*dt; Q(2, 2) = svSiz*dt;
    Q(3, 3) = svPos*dt; Q(4, 4) = svPos*dt; Q(5, 5) = svSiz*dt;
    filter->set_Q(Q);

    filter->predict();

    return 0;
}

/******************************************************************************/
bool vCircleTracker::correct(double xz, double yz, double rz)
{
    if(!active) return false;
    yarp::sig::Vector z(6, 0.0); z[0]=xz; z[1]=yz; z[2]=rz;
    filter->correct(z);
    return true;
}

/******************************************************************************/
bool vCircleTracker::getState(double &x, double &y, double &r)
{
    if(!active) return false;
    yarp::sig::Vector state = filter->get_x();
    x = state[0]; y = state[1]; r = state[2];
    return true;
}

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

#include "vHough.h"


/******************************************************************************/
vHoughCircle::vHoughCircle()
{

    qlength = 2000;
    qduration = 200000;
    r1 = 8;
    r2 = 32;
    rsize = r2 - r1;
    height = 128;
    width = 128;

    found = false;
    xc = 0; yc = 0; rc = 0; valc = 0;

    for(int r = 0; r < rsize; r++) {
        H.push_back(new yarp::sig::Matrix(height, width));
    }

}

/******************************************************************************/
vHoughCircle::~vHoughCircle()
{
    for(int r = 0; r < rsize; r++) {
        delete H[r];
    }
}

/******************************************************************************/
void vHoughCircle::updateH(int xv, int yv, int val)
{
    valc = 0;
    for(int r = 0; r < rsize; r++) {
        int R = r+r1;
        int xstart = std::max(0, xv - R);
        int xend = std::min(width-1, xv + R);
        for(int x = xstart; x < xend; x++) {
            //(xv-xc)^2 + (yv - yc)^2 = R^2
            int deltay = (int)sqrt(pow(R, 2.0) - pow(x - xv, 2.0));

            for(int s = -1; s < 2; s++) {
                int y = yv + deltay + s;
                if(y > 127 || y < 0) continue;
                (*H[r])[y][x] += val;
                if(val > 0 && (*H[r])[y][x] > valc) {
                    valc = (*H[r])[y][x];
                    xc = x;
                    yc = y;
                    rc = R;
                }

                y = yv - deltay + s;
                if(y > 127 || y < 0) continue;
                (*H[r])[y][x] += val;
                if(val > 0 && (*H[r])[y][x] > valc) {
                    valc = (*H[r])[y][x];
                    xc = x;
                    yc = y;
                    rc = R;
                }
            }

        }
    }
}

/******************************************************************************/
void vHoughCircle::updateHwithFlow(int xv, int yv, int val, double dtdx, double dtdy)
{
    valc = 0;
    for(int r = 0; r < rsize; r++) {
        int R = r+r1;

        double theta1 = atan2(dtdy, dtdx);
        //double theta2 = 3.14 + theta1;

        int x_base = R * cos(theta1) + xv;
        int y_base = R * sin(theta1) + yv;

        for(int y = y_base-2; y < y_base+2; y++) {
            for(int x = x_base-2; x < x_base+2; x++) {

                if(x < 0 || x > width-1 || y < 0 || y > height-1) continue;

                (*H[r])[y][x] += val;
                if(val > 0 && (*H[r])[y][x] > valc) {
                    valc = (*H[r])[y][x];
                    xc = x;
                    yc = y;
                    rc = R;
                }
            }
        }

        theta1 = atan2(dtdy, dtdx) + 3.14;
        //double theta2 = 3.14 + theta1;

        x_base = R * cos(theta1) + xv;
        y_base = R * sin(theta1) + yv;

        for(int y = y_base-2; y < y_base+2; y++) {
            for(int x = x_base-2; x < x_base+2; x++) {

                if(x < 0 || x > width-1 || y < 0 || y > height-1) continue;

                (*H[r])[y][x] += val;
                if(val > 0 && (*H[r])[y][x] > valc) {
                    valc = (*H[r])[y][x];
                    xc = x;
                    yc = y;
                    rc = R;
                }
            }
        }



    }
}

/******************************************************************************/
void vHoughCircle::addEventFixed(emorph::vEvent &event)
{

    found = false;
    emorph::AddressEvent *v;
    while(FIFO.size() > qlength) {
        v = FIFO.back()->getAs<emorph::AddressEvent>();
        updateH(v->getX(), v->getY(), -1);
        FIFO.pop_back();
    }

    FIFO.push_front(&event);
    v = FIFO.front()->getAs<emorph::AddressEvent>();
    updateH(v->getX(), v->getY(), 1);
    found = true;


}

/******************************************************************************/
void vHoughCircle::addEventTime(emorph::vEvent &event)
{

    found = false;
    FIFO.push_front(&event);
    emorph::AddressEvent *v = event.getAs<emorph::AddressEvent>();
    updateH(v->getX(), v->getY(), 1);

    //remove any events falling out the back of the window
    int ctime = event.getStamp();
    int upper = ctime + emorph::vtsHelper::maxStamp() - qduration;
    int lower = ctime - qduration;

    while(true) {

        int vtime = FIFO.back()->getStamp();
        if((vtime > ctime && vtime < upper) || vtime < lower) {
            v = FIFO.back()->getAs<emorph::AddressEvent>();
            updateH(v->getX(), v->getY(), -1);
            FIFO.pop_back();
        } else {
            break;
        }
    }

    found = true;

}

/******************************************************************************/
void vHoughCircle::addEventLife(emorph::vEvent &event)
{
    found = false;
    emorph::FlowEvent *v = event.getAs<emorph::FlowEvent>();
    if(!v) return;
    updateHwithFlow(v->getX(), v->getY(), 1, v->getVx(), v->getVy());
    FIFO.push_front(&event);
    int cts = v->getStamp();

//    double flowmag = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0)) /
//            emorph::vtsHelper::tstosecs();
//    if(vr->getStamp() - v->getStamp() > flowmag)


    emorph::vQueue::iterator i = FIFO.begin();
    while(i != FIFO.end()) {
        v = (*i)->getAs<emorph::FlowEvent>();
        //int life = sqrt(pow(v->getVx(), 2.0) + pow(v->getVy(), 2.0)) /
        //        emorph::vtsHelper::tstosecs();
        //if(life > 1000000) life = 1000000;
        //int death = v->getStamp() + life;
        int death = v->getDeath();
        if(cts < v->getStamp())
            death -= emorph::vtsHelper::maxStamp();
        //if(death > emorph::vtsHelper::maxStamp())
        if(cts > death || (death >= emorph::vtsHelper::maxStamp() && cts+emorph::vtsHelper::maxStamp() > death)) {
            updateHwithFlow(v->getX(), v->getY(), -1, v->getVx(), v->getVy());
            i = FIFO.erase(i);
        } else {
            i++;
        }
    }

    found = true;

}

/******************************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelMono> vHoughCircle::makeDebugImage(int r)
{

    yarp::sig::ImageOf<yarp::sig::PixelMono> canvas;
    canvas.resize(width, height);
    if(r < r1 && r > r2) return canvas;
    r = r - r1;

    double maxval = 0;
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            maxval = std::max(maxval, (*H[r])[y][x]);
        }
    }

    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            canvas(y,127- x) = 255.0 * (*H[r])[y][x] / maxval;
        }
    }

    return canvas;
}

/******************************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelMono> vHoughCircle::makeDebugImage2()
{

    std::cout << FIFO.size() << std::endl;
    yarp::sig::ImageOf<yarp::sig::PixelMono> canvas;
    canvas.resize(width, height); canvas.zero();

    int bx, by, br, bval = 0;
    for(int r = 0; r < rsize; r++) {
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                if((*H[r])[y][x] > bval) {
                    bx = x;
                    by = y;
                    br = r;
                    bval = (*H[r])[y][x];
                }
            }
        }
    }

    int R = br+r1;
    int xstart = std::max(0, bx - R);
    int xend = std::min(width-1, bx + R);
    for(int x = xstart; x < xend; x++) {
        //(xv-xc)^2 + (yv - yc)^2 = R^2
        int deltay = (int)sqrt(pow(R, 2.0) - pow(x - bx, 2.0));

        int y = by + deltay;
        if(y > 127) continue;
        canvas(y, 127 - x) = 255.0;
        y = by - deltay;
        if(y < 0) continue;
        canvas(y, 127 - x) = 255.0;
    }

    return canvas;
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
    yarp::sig::Matrix A(9, 9); A = 0;
    A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1;
    A(3, 3) = 1; A(4, 4) = 1; A(5, 5) = 1;
    A(6, 6) = 1; A(7, 7) = 1; A(8, 8) = 1;
    //we need to update A: (0, 3), (1, 4) and (2, 5) based on delta t

    yarp::sig::Matrix Q(9, 9); Q = 0;
    //we update Q depending on delta t

    //these two are constant
    yarp::sig::Matrix H(9, 9); H = 0;
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;

    yarp::sig::Matrix R(9, 9); R = 0;
    R(0, 0) = zvPos; R(1, 1) = zvPos; R(2, 2) = zvSiz;

    filter = new iCub::ctrl::Kalman(A, H, Q, R);
}

/******************************************************************************/
bool vCircleTracker::startTracking(double xz, double yz, double rz)
{
    if(!filter) return false;
    yarp::sig::Vector x0(9, 0.0); x0[0] = xz; x0[1] = yz; x0[2] = rz;
    yarp::sig::Matrix P0 = filter->get_R();
    P0(3, 3) = P0(0, 0); P0(4, 4) = P0(1, 1); P0(5, 5) = P0(2, 2);
    P0(4, 4) = P0(0, 0); P0(5, 5) = P0(1, 1); P0(6, 6) = P0(2, 2);
    filter->init(x0, P0);
    active = true;
    return true;
}

/******************************************************************************/
double vCircleTracker::predict(double dt)
{

    if(!active) return 0;
    //update the tracker position
    yarp::sig::Matrix A = filter->get_A();
    double dt2 = 0.5 * pow(dt, 2.0);
    A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
    //A(3, 6) = dt; A(4, 7) = dt; A(5, 8) = dt;
    //A(0, 6) = dt2; A(1, 7) = dt2; A(2, 8) = dt2;
    filter->set_A(A);

    yarp::sig::Matrix Q = filter->get_Q();
    //Q(6, 6) = svPos*dt2; Q(7, 7) = svPos*dt2; Q(8, 8) = svSiz*dt2;
    Q(3, 3) = svPos*dt; Q(4, 4) = svPos*dt; Q(5, 5) = svSiz*dt;
    filter->set_Q(Q);

    filter->predict();

    return 0;
}

/******************************************************************************/
bool vCircleTracker::correct(double xz, double yz, double rz)
{
    if(!active) return false;
    yarp::sig::Vector z(9, 0.0); z[0]=xz; z[1]=yz; z[2]=rz;
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

/******************************************************************************/
double vCircleTracker::Pzgd(double xz, double yz, double rz)
{
    if(!active) return -1;

    yarp::sig::Vector x = filter->get_x();
    yarp::sig::Matrix P = filter->get_P();

    double Px = exp(-0.5 * pow(xz - x[0], 2.0) / pow(P(0, 0), 2.0));
    double Py = exp(-0.5 * pow(yz - x[1], 2.0) / pow(P(1, 1), 2.0));
    double Pr = exp(-0.5 * pow(rz - x[2], 2.0) / pow(P(2, 2), 2.0));

    return std::min(Px, std::min(Py, Pr));

}

//double vCircleTracker::Pvgd(double xv, double yv)
//{
//    if(!active) return 0;

//    yarp::sig::Vector c = filter->get_x();
//    double xc = c[0], yc = c[1], rc = c[2];

//    //calculate the distance to the mean of the distribution
//    double rv = sqrt(pow(yv-yc, 2.0) + pow(xv-xc, 2.0));
//    if(rv == 0) return 0;
//    double xd = xc + (rc / rv) * (xv - xc); //trouble here if rv == 0
//    double yd = yc + (rc / rv) * (yv - yc);

//    double x = fabs(xv - xd);
//    double y = fabs(yv - yd);

//    yarp::sig::Matrix P = filter->get_P();
//    double distance = P(0, 0) * pow(x, 2.0) + 2 * P(0, 1) * x * y +
//            P(1, 1) * pow(y, 2.0);
//    distance = pow(x, 2.0) + pow(y, 2.0);
//    double det = pow(P(0, 0), 2.0) + pow(P(1, 1), 2.0) + 2 * P(0, 0) * P(1, 1) *
//            P(0, 1) + pow(P(2, 2), 2.0);

//    double pvgd = exp(-0.5 * distance / det);
//    return pvgd;// / sqrt(det * 6.283);
//}



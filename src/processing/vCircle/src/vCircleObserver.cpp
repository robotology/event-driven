/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

/*////////////////////////////////////////////////////////////////////////////*/
//vCircleThread
/*////////////////////////////////////////////////////////////////////////////*/
vCircleThread::vCircleThread(int R, bool directed, bool parallel, int height, int width)
{
    this->threaded = parallel;
    this->R = R;
    this->Rsqr = pow(R, 2.0);
    this->directed = directed;
    this->height = height;
    this->width = width;

    H.resize(height, width);
    a = R * fabs(tan(20.0 * M_PI / 180.0));
    //Hstr = 1.0 / (int)(2 * M_PI * R + 0.5);
    Hstr = 1.0 / (2.0 * M_PI * R * 0.5);

    x_max = 0; y_max = 0;

    mstart.lock();
    mdone.lock();

    if(threaded)
        this->start();

}

void vCircleThread::process(emorph::vQueue &procQueue, std::vector<int> &procType) {

    this->procQueue = &procQueue;
    this->procType = &procType;

    if(threaded)
        mstart.unlock();
    else
        performHough();

}

void vCircleThread::waitfordone()
{
    if(threaded)
        mdone.lock();
}

void vCircleThread::updateHAddress(int xv, int yv, double strength)
{
    int P = 0;

    int xstart = std::max(0, xv - R);
    int xend = std::min(width-1, xv + R);

    for(int x = xstart; x <= xend; x++) {
        //(xv-xc)^2 + (yv - yc)^2 = R^2
        int deltay = (int)sqrt(Rsqr - pow(x - xv, 2.0));

        for(int s = -P; s <= P; s++) {
            int y = yv + deltay + s;
            if(y > height-1 || y < 0) continue;
            H[y][x] += strength;
            if(H[y][x] > H[y_max][x_max]) {
                y_max = y; x_max = x;
            }

            if(!deltay) continue; //don't double up on same space
            if(deltay - P <= -deltay + s) continue;

            y = yv - deltay + s;
            if(y > height-1 || y < 0) continue;
            H[y][x] += strength;
            if(H[y][x] > H[y_max][x_max]) {
                y_max = y; x_max = x;
            }
        }
    }
}

double vCircleThread::updateHFlowAngle(int xv, int yv, double strength,
                                     double dtdx, double dtdy)
{

    int P = 0;

    //this switch could be done when passing arguments to the function
    double temp = dtdx;
    dtdx = dtdy;
    dtdy = temp;

    bool horquad = fabs(dtdy/dtdx) < 1;

    //this is the same for all R try passing xn/yn to the function instead
    double velR = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0));
    double xn = dtdx / velR;
    double yn = dtdy / velR;

    //calculate the end position of the tangent to the arc
    double x2 = R * xn - yn * a;
    double y2 = R * yn + xn * a;

    double nonadjR = sqrt(pow(x2, 2.0) + pow(y2, 2.0));

    //then adjust to fit the radius R
    double x2h = R * x2 / nonadjR;
    double y2h = R * y2 / nonadjR;

    //also for the other end of the arc
    double x3 = R * xn + yn * a;
    double y3 = R * yn - xn * a;

    double x3h = R * x3 / nonadjR; //nonadjR is the same for both 2 and 3
    double y3h = R * y3 / nonadjR;

    //treat it differently depending on the angle
    if(horquad) {

        //we are mostly left or right of the centre
        int sign = xn < 0 ? -1 : 1;

        //get the starting position
        int yStart = y2h+0.5, yEnd = y3h+0.5;
        if(y2h > y3h) {
            yStart = y3h+0.5; yEnd = y2h+0.5;
        }

        //and then go through the y values
        for(int yd = yStart; yd <= yEnd; yd++) {

            //calculate the x value
            int xd = (int)(sqrt(Rsqr - pow(yd, 2.0)) * sign + 0.5);

            //for both forward and reverse directions
            for(int dir = 1; dir >= -1; dir -= 2) {

                //update the direction
                int xdd = xd * dir;
                int ydd = yd * dir;

                //calculate x pixel location and check limits
                int ypix = yv + ydd;
                if(!(ypix > 0 && ypix < width)) continue;

                //then for some error band add in the values
                for(int s = -P; s <= P; s++) {

                    //calculate the y value
                    int xpix = xv + xdd + s;
                    if(!(xpix > 0 && xpix < height)) continue;

                    //update and check the hough transform
                    H[ypix][xpix] += strength;
                    if(H[ypix][xpix] > H[y_max][x_max]) {
                        y_max = ypix; x_max = xpix;
                    }
                }
            }
        }
    } else {


        //we are mostly vertical either above or below the centre
        int sign = yn < 0 ? -1 : 1;

        //get the starting position
        int xStart = x2h+0.5, xEnd = x3h+0.5;
        if(x2h > x3h) {
            xStart = x3h+0.5; xEnd = x2h+0.5;
        }

        //and then go through the x values
        for(int xd = xStart; xd <= xEnd; xd++) {

            //calculate the y value
            int yd = (int)(sqrt(Rsqr - pow(xd, 2.0)) * sign + 0.5);

            //for both forward and reverse directions
            for(int dir = 1; dir >= -1; dir -= 2) {

                //update the direction
                int xdd = xd * dir;
                int ydd = yd * dir;

                //calculate x pixel location and check limits
                int xpix = xv + xdd;
                if(!(xpix > 0 && xpix < width)) continue;

                //then for some error band add in the values
                for(int s = -P; s <= P; s++) {

                    //calculate the y value
                    int ypix = yv + ydd + s;
                    if(!(ypix > 0 && ypix < height)) continue;

                    //update and check the hough transform
                    H[ypix][xpix] += strength;
                    if(H[ypix][xpix] > H[y_max][x_max]) {
                        y_max = ypix; x_max = xpix;
                    }
                }
            }
        }
    }

    return 0;

}

double vCircleThread::updateHFlowAngle2(int xv, int yv, double strength,
                                     double dtdx, double dtdy)
{

    double m = dtdx / dtdy;
    double b = yv - m * xv;
    int bmd = 0;
    int dr = 1;

    for(int x = 0; x < width; x++) {

        int y = m * x + b + 0.5;
        double delta = sqrt(pow(x - xv, 2.0) + pow(y - yv, 2.0));

        //if(delta > R - dr && delta < R + dr) {

            for(int bmod = -bmd; bmod <= bmd; bmod++) {
                y = m * x + b + bmod + 0.5;
                if(x < 0 || y < 0 || x >= width || y >= height) continue;
                //this delta is incorrect
                H[y][x] += strength;
                if(H[y][x] > H[y_max][x_max]) {
                    y_max = y; x_max = x;
                }
            }

//            y = m * x + b + 1.5;
//            if(x < 0 || y < 0 || x >= width || y >= height) continue;
//            //this delta is incorrect
//            H[y][x] += strength;
//            if(H[y][x] > H[y_max][x_max]) {
//                y_max = y; x_max = x;
//            }

//            y = m * x + b - 0.5;
//            if(x < 0 || y < 0 || x >= width || y >= height) continue;
//            //this delta is incorrect
//            H[y][x] += strength;
//            if(H[y][x] > H[y_max][x_max]) {
//                y_max = y; x_max = x;
//            }
        //}

    }

    return 0;

}

//double vCircleThread::updateHFlowAngle3(int xv, int yv, double strength,
//                                     double dtdx, double dtdy)
//{

//    double total = 0;
//    double temp = dtdx;
//    dtdx = dtdy;
//    dtdy = temp;

//    //calculate the non-exact energy per pixel (doesn't account for pixel
//    //rounding or image edges
//    double eval = strength / (M_PI * Rsqr * 2);

//    //this is the same for all R try passing it to the function instead
//    double velR = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0));
//    double xn = dtdx / velR;
//    double yn = dtdy / velR;



//    //for forward and backward flows
//    for(int dir = 1; dir >= -1; dir -= 2) {

//        //calculate the centre position
//        double x1 = dir * R * xn + xv;
//        double y1 = dir * R * yn + yv;

//        //and then go through the y values of a circle (this could actually be
//        //hardcoded fairly easily rather than calculated every time
//        for(int ydi = -R; ydi <= R; ydi++) {

//            int xd = (int)(sqrt(Rsqr - pow(ydi, 2.0))+0.5);

//            for(int xdi = -xd; xdi <= xd; xdi++) {

//                int xpix = x1 + xdi;
//                int ypix = y1 + ydi;

//                if(!(xpix > 0 && xpix < width)) continue;
//                if(!(ypix > 0 && ypix < height)) continue;

//                H[ypix][xpix] -= eval;
//                total -= eval;

//            }
//        }
//    }

//    return total;

//}

void vCircleThread::performHough()
{

    for(int i = 0; i < procQueue->size(); i++) {

        if(directed) {

            emorph::FlowEvent * v = (*procQueue)[i]->getAs<emorph::FlowEvent>();

            if(v) {
                updateHFlowAngle(v->getX(), v->getY(), (*procType)[i] * Hstr,
                                 v->getVx(), v->getVy());
            }


        } else {

            emorph::AddressEvent * v = (*procQueue)[i]->getAs<emorph::AddressEvent>();

            if(v) {
                updateHAddress(v->getX(), v->getY(), (*procType)[i] * Hstr);
            }

        }

    }


}

void vCircleThread::run()
{

    while(!isStopping()) {

        waitforstart();

        performHough();

        signalfinish();

    }
}

yarp::sig::ImageOf<yarp::sig::PixelBgr> vCircleThread::makeDebugImage(double refval)
{

    yarp::sig::ImageOf<yarp::sig::PixelBgr> canvas;
    canvas.resize(width, height);
    canvas.zero();

    if(refval < 0)
        refval = H[y_max][x_max];

    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            double I;
            if(H[y][x] >= refval*0.9) I = 255.0;
            else I = 255.0 * pow(H[y][x] / refval, 2.0);
            canvas(y, 127 - x) = yarp::sig::PixelBgr(I, I, I);
        }
    }

    return canvas;
}

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMULTISIZE
/*////////////////////////////////////////////////////////////////////////////*/
vCircleMultiSize::vCircleMultiSize(std::string qType, int qLength, int rLow, int rHigh,
                                   bool directed, bool parallel, int height, int width)
{
    this->qType = qType;
    this->qlength = qLength;

    for(int r = rLow; r <= rHigh; r++)
        htransforms.push_back(new vCircleThread(r, directed, parallel, height, width));

    best = htransforms.begin();
    dummy.referto();
}

vCircleMultiSize::~vCircleMultiSize()
{

    emorph::vQueue dummy1;
    std::vector<int> dummy2;
    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++) {
        (*i)->stop();
        (*i)->process(dummy1, dummy2);
        (*i)->waitfordone();
        delete *i;
    }
    dummy.destroy();


}

void vCircleMultiSize::addQueue(emorph::vQueue &additions) {

        if(qType == "Fixed")
            addFixed(additions);
        else if(qType == "Lifetime")
            addLife(additions);
        else if(qType == "surf")
            addSurf(additions);
        else if(qType == "edge")
            addEdge(additions);
        else
            std::cerr << "Did not understand qType" << std::endl;

}

void vCircleMultiSize::updateHough(emorph::vQueue &procQueue, std::vector<int> &procType)
{

    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++)
        (*i)->process(procQueue, procType);

    for(i = htransforms.begin(); i != htransforms.end(); i++)
        (*i)->waitfordone();

}

double vCircleMultiSize::getObs(int &x, int &y, int &r)
{
    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++)
        if((*i)->getScore() > (*best)->getScore())
            best = i;

    x = (*best)->getX();
    y = (*best)->getY();
    r = (*best)->getR();
    return (*best)->getScore();

}

void vCircleMultiSize::addFixed(emorph::vQueue &additions)
{

    emorph::vQueue procQueue;
    std::vector<int> procType;

    emorph::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        //GET THE EVENTS AS CORRECT TYPE
        emorph::AddressEvent *v = (*vi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel()) continue;

        procQueue.push_back(v);
        procType.push_back(1);


        bool removed = false;

        //CHECK TO REMOVE "SAME LOCATION EVENTS FIRST"
        int cx = v->getX(); int cy = v->getY();
        emorph::vQueue::iterator i = FIFO.begin();
        while(i != FIFO.end()) {
            //we only add Address Events therefore we can do an unsafe cast
            v = (*i)->getUnsafe<emorph::AddressEvent>();
            removed = v->getX() == cx && v->getY() == cy;
            if(removed) {
                procQueue.push_back(v);
                procType.push_back(-1);
                i = FIFO.erase(i);
                break;
            } else {
                i++;
            }
        }

        //ADD THE CURRENT EVENT
        FIFO.push_front(*vi);

        //KEEP FIFO TO LIMITED SIZE
        while(FIFO.size() > qlength) {
            procQueue.push_back(FIFO.back());
            procType.push_back(-1);
            FIFO.pop_back();
            removed = true;
        }

    }

    updateHough(procQueue, procType);

}

void vCircleMultiSize::addLife(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    std::vector<int> procType;

    emorph::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        //lifetime requires a flow event only
        emorph::FlowEvent *v = (*vi)->getAs<emorph::FlowEvent>();
        if(!v) return;

        //add this event
        procQueue.push_back(v);
        procType.push_back(1);

        //then find any events that should be removed
        int cts = v->getStamp();
        int cx = v->getX(); int cy = v->getY();
        emorph::FlowEvent * v2;
        emorph::vQueue::iterator i = FIFO.begin();
        while(i != FIFO.end()) {
            v2 = (*i)->getUnsafe<emorph::FlowEvent>();
            int modts = cts;
            if(cts < v2->getStamp()) //we have wrapped
                modts += emorph::vtsHelper::maxStamp();

            bool samelocation = v2->getX() == cx && v2->getY() == cy;

            if(modts > v2->getDeath() || samelocation) {
                procQueue.push_back(v2);
                procType.push_back(-1);
                i = FIFO.erase(i);
            } else {
                i++;
            }
        }

        //add to queue
        FIFO.push_front(v);
    }

    //add this event to the hough space
    updateHough(procQueue, procType);

}

void vCircleMultiSize::addSurf(emorph::vQueue &additions)
{

    emorph::vQueue procQueue;
    std::vector<int> procType;

    emorph::vQueue::iterator qi;
    for(qi = additions.begin(); qi != additions.end(); qi++) {
        emorph::AddressEvent * v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        procQueue.push_back(v);
        procType.push_back(1);

        emorph::vEvent * removed = surface.addEvent(*v);
        if(removed) {
            procQueue.push_back(removed);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);

}

void vCircleMultiSize::addEdge(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    std::vector<int> procType;

    emorph::vQueue::iterator qi;
    for(qi = additions.begin(); qi != additions.end(); qi++) {
        emorph::AddressEvent * v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel()) continue;

        emorph::FlowEvent * vf = v->getAs<emorph::FlowEvent>();
        //emorph::FlowEvent * vf = edge.upgradeEvent(v);

        if(vf) {
            procQueue.push_back(vf);
            procType.push_back(1);
        }

        emorph::vQueue removed = edge.addEventToEdge(v);
        for(int i = 0; i < removed.size(); i++) {
            if(removed[i]->getAs<emorph::FlowEvent>()) {
                procQueue.push_back(removed[i]);
                procType.push_back(-1);
            }
        }
    }

    updateHough(procQueue, procType);

}

yarp::sig::ImageOf<yarp::sig::PixelBgr> vCircleMultiSize::makeDebugImage()
{

    //update best
    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++)
        if((*i)->getScore() > (*best)->getScore())
            best = i;

    double score = (*best)->getScore();
    score = 0.2;

    i = htransforms.begin();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> imagebase =
            (*i)->makeDebugImage(score);

    for(i++; i != htransforms.end(); i++) {
        yarp::sig::ImageOf<yarp::sig::PixelBgr> image = (*i)->makeDebugImage(score);
        for(int y = 0; y < image.height(); y++) {
            for(int x = 0; x < image.width(); x++) {
                yarp::sig::PixelBgr pixbase = imagebase(x, y);
                yarp::sig::PixelBgr piximg  = image(x, y);
                if(piximg.b > pixbase.b)
                    imagebase(x, y) = piximg;
            }
        }
    }


    // = (*best)->makeDebugImage((*best)->getScore());
    //image.resize(128, 128);
    //image.zero();

    emorph::vQueue q;
    if(qType == "Fixed")
        q = this->FIFO;
    else if(qType == "Lifetime")
        q = this->FIFO;
    else if(qType == "surf")
        q = surface.getSURF(0, 127, 0, 127);
    else if(qType == "edge")
        q = edge.getSURF(0, 127, 0, 127);

    for(int i = 0; i < q.size(); i++) {
        emorph::AddressEvent *v = q[i]->getUnsafe<emorph::AddressEvent>();
        imagebase(v->getY(), 127 - v->getX()) = yarp::sig::PixelBgr(255, 0, 255);
    }



//    for(int x = 0; x < 128; x++) {
//        for(int y = 0; y < 128; y++) {
//            image(x+128, y) = image2(x, y);
//        }
//    }


    return imagebase;
}

/*////////////////////////////////////////////////////////////////////////////*/
//vCircleTracker
/*////////////////////////////////////////////////////////////////////////////*/
vCircleTracker::vCircleTracker()
{
    svPos = 5;
    svSiz = 2;
    filter = 0;
    active = false;

}

vCircleTracker::~vCircleTracker()
{
    if(filter) delete filter;
}

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

bool vCircleTracker::correct(double xz, double yz, double rz)
{
    if(!active) return false;
    yarp::sig::Vector z(6, 0.0); z[0]=xz; z[1]=yz; z[2]=rz;
    filter->correct(z);
    return true;
}

bool vCircleTracker::getState(double &x, double &y, double &r)
{
    if(!active) return false;
    yarp::sig::Vector state = filter->get_x();
    x = state[0]; y = state[1]; r = state[2];
    return true;
}

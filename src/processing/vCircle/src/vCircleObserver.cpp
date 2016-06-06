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
vCircleThread::vCircleThread(int R, bool directed, bool parallel, int height, int width, double arclength)
{
    this->threaded = parallel;
    this->R = R;
    this->Rsqr = pow(this->R, 2.0);
    this->directed = directed;
    this->height = height;
    this->width = width;

    H.resize(this->height, this->width); H.zero();
    double alr  = arclength * M_PI / 180.0;

    a = 0;
    double count = 0;
    int x = R; int y = 0;
    for(double th = 0; th <= 2 * M_PI; th+=0.01) {

        int xn = R * cos(th) + 0.5;
        int yn = R * sin(th) + 0.5;

        if((xn != x || yn != y) && (sqrt(pow(xn, 2.0)+pow(yn, 2.0))-R < 0.4)) {
            x = xn;
            y = yn;
            hy.push_back(y);
            hx.push_back(x);
            hang.push_back(th);
            count++;
        }

        if(!a && th > alr) a = hx.size();
    }
    Hstr = 0.05;

    x_max = 0; y_max = 0;

    canvas.resize(width, height);
    canvas.zero();

    mstart.lock();
    mdone.lock();

    if(threaded)
        this->start();

}

void vCircleThread::process(emorph::vQueue &procQueue, std::vector<int> &procType)
{

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

void vCircleThread::updateHAddress(int xv, int yv, int strength)
{

    //std::cout << "Updating with LUT" << std::endl;
    for(unsigned int i = 0; i < hx.size(); i++) {

        int x = xv + hx[i];
        int y = yv + hy[i];
        if(y > height - 1 || y < 0 || x > width -1 || x < 0) continue;

        //std::cout << y << " " << x << std::endl;

        H(y, x) += strength;
        if(H(y, x) > H(y_max, x_max)) {
            y_max = y; x_max = x;
        }
    }
    return;
}

double vCircleThread::updateHFlowAngle(int xv, int yv, int strength,
                                     double dtdx, double dtdy)
{

    //this is the same for all R try passing xn/yn to the function instead
    double velR = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0));
    double xr = R * dtdy / velR;
    double yr = R * dtdx / velR;

    double theta = acos(xr/R) / (2 * M_PI);
    if(yr < 0) theta = 1 - theta;
    int bir = theta * hx.size();

    //now fill in the pixels from that starting pixel for a pixels forward and
    //backward

    for(int i = bir - a; i <= bir + a; i++) {

        int modi =  i;
        if(i >= (int)hx.size())
            modi = i - hx.size();
        if(i < 0)
            modi = i + hx.size();

        int x = xv + hx[modi];
        int y = yv + hy[modi];

        if(y >= 0 && y < height && x >= 0 && x < width) {
            H(y, x) += strength;
            if(H(y, x) > H(y_max, x_max)) {
                y_max = y; x_max = x;
            }
        }

        x = xv - hx[modi];
        y = yv - hy[modi];

        if(y >= 0 && y < height && x >= 0 && x < width) {
            H(y, x) += strength;
            if(H(y, x) > H(y_max, x_max)) {
                y_max = y; x_max = x;
            }
        }

    }

    return 0;

}

void vCircleThread::performHough()
{

    for(unsigned int i = 0; i < procQueue->size(); i++) {

        if(directed) {

            emorph::FlowEvent * v = (*procQueue)[i]->getAs<emorph::FlowEvent>();

            if(v) {
                updateHFlowAngle(v->getX(), v->getY(), (*procType)[i],
                                 v->getVx(), v->getVy());
            }

        } else {

            emorph::AddressEvent * v = (*procQueue)[i]->getAs<emorph::AddressEvent>();

            if(v) {
                updateHAddress(v->getX(), v->getY(), (*procType)[i]);
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

int vCircleThread::findScores(std::vector<double> &values, double threshold)
{
    int c = 0;
    for(int y = 0; y < height; y += 1) {
        for(int x = 0; x < width; x += 1) {
            if(H(y, x) > threshold) {
                values.push_back(x);
                values.push_back(y);
                values.push_back(R);
                values.push_back(H(y, x)*Hstr);
                c++;
            }
        }
    }

    return c;

}

yarp::sig::ImageOf<yarp::sig::PixelBgr> vCircleThread::makeDebugImage(double refval)
{

    if(refval < 0)
        refval = H(y_max, x_max)*Hstr;

    for(int y = 0; y < height; y += 1) {
        for(int x = 0; x < width; x += 1) {

            if(H(y, x)*Hstr >= refval*0.9)
                canvas(y, width - 1 - x) = yarp::sig::PixelBgr(255, 255, 255);
            else {
                int I = 255.0 * pow(H(y, x)*Hstr / refval, 1.0);
                if(I > 254) I = 254;
                //I = 0;
                if(directed)
                    canvas(y, width - 1 - x) = yarp::sig::PixelBgr(0, I, 0);
                else
                    canvas(y, width - 1 - x) = yarp::sig::PixelBgr(0, 0, I);

            }
//            if(H[y][x] >= refval*0.9) {
//                canvas(y, width - 1 - x) = yarp::sig::PixelBgr(
//                I = 255.0;
//            else I =

        }
    }

    return canvas;
}

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMULTISIZE
/*////////////////////////////////////////////////////////////////////////////*/
vCircleMultiSize::vCircleMultiSize(double threshold, std::string qType,
                                   int rLow, int rHigh,
                                   bool directed, bool parallel,
                                   int height, int width, int arclength, double fifolength)
{
    this->qType = qType;
    this->threshold = threshold;
    this->fifolength = fifolength;
    this->directed = directed;

    for(int r = rLow; r <= rHigh; r++)
        htransforms.push_back(new vCircleThread(r, directed, parallel, height, width, arclength));

    best = htransforms.begin();
    fFIFO = emorph::fixedSurface(fifolength, width, height);
    tFIFO = emorph::temporalSurface(width, height, fifolength * fifolength * 7812.5);
    //eFIFO.setThickness(1);
    //fFIFO.setFixedWindowSize(fifolength);
    //tFIFO.setTemporalSize(fifolength * 7812.5);
    channel = 0;

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
}

void vCircleMultiSize::addQueue(emorph::vQueue &additions) {

        if(qType == "fixed")
            addFixed(additions);
        else if(qType == "life")
            addLife(additions);
        else if(qType == "time")
            addTime(additions);
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

std::vector<double> vCircleMultiSize::getPercentile(double p, double thMin)
{
    int x, y, r;
    double maxval = getObs(x, y, r);
    double threshold = std::max(p * maxval, (double)thMin);

    std::vector<double> values;
    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++)
        (*i)->findScores(values, threshold);

    return values;
}

void vCircleMultiSize::addFixed(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    procType.clear();
    emorph::AddressEvent *v;

    emorph::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        //GET THE EVENTS AS CORRECT TYPE
        //emorph::AddressEvent *v = (*vi)->getAs<emorph::AddressEvent>();
        if(directed)
            v = (*vi)->getAs<emorph::FlowEvent>();
        else
            v = (*vi)->getAs<emorph::AddressEvent>();

        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        emorph::vQueue removed = fFIFO.addEvent(*v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

void vCircleMultiSize::addTime(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    procType.clear();
    emorph::AddressEvent *v;

    emorph::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        v = (*vi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        emorph::vQueue removed = tFIFO.addEvent(*v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

void vCircleMultiSize::addLife(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    procType.clear();
    emorph::AddressEvent *v;

    emorph::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        v = (*vi)->getAs<emorph::FlowEvent>();
        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        emorph::vQueue removed = lFIFO.addEvent(*v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

//void vCircleMultiSize::addFixed(emorph::vQueue &additions)
//{

//    emorph::vQueue procQueue;
//    procType.clear();
//    emorph::AddressEvent *v;

//    emorph::vQueue::iterator vi;
//    for(vi = additions.begin(); vi != additions.end(); vi++) {

//        //GET THE EVENTS AS CORRECT TYPE
//        //emorph::AddressEvent *v = (*vi)->getAs<emorph::AddressEvent>();
//        if(directed)
//            v = (*vi)->getAs<emorph::FlowEvent>();
//        else
//            v = (*vi)->getAs<emorph::AddressEvent>();

//        if(!v || v->getChannel()) continue;

//        procQueue.push_back(v);
//        procType.push_back(1);


//        bool removed = false;

//        //CHECK TO REMOVE "SAME LOCATION EVENTS FIRST"
//        // <----------THIS IS THE SLOWEST PART OF BALL TRACKING -------->
//        int cx = v->getX(); int cy = v->getY();
//        emorph::vQueue::iterator i = FIFO.begin();
//        while(i != FIFO.end()) {
//            //we only add Address Events therefore we can do an unsafe cast
//            v = (*i)->getUnsafe<emorph::AddressEvent>(); //this may break now?
//            removed = v->getX() == cx && v->getY() == cy;
//            if(removed) {
//                procQueue.push_back(v);
//                procType.push_back(-1);
//                i = FIFO.erase(i);
//                break;
//            } else {
//                i++;
//            }
//        }
//        // <---------------------------------------------------- -------->

//        //ADD THE CURRENT EVENT
//        FIFO.push_front(*vi);

//        //KEEP FIFO TO LIMITED SIZE
//        while(FIFO.size() > fifolength) {
//            procQueue.push_back(FIFO.back());
//            procType.push_back(-1);
//            FIFO.pop_back();
//            removed = true;
//        }

//    }

//    updateHough(procQueue, procType);

//}

//void vCircleMultiSize::addLife(emorph::vQueue &additions)
//{
//    emorph::vQueue procQueue;
//    procType.clear();

//    emorph::vQueue::iterator vi;
//    for(vi = additions.begin(); vi != additions.end(); vi++) {

//        //lifetime requires a flow event only
//        emorph::FlowEvent *v = (*vi)->getAs<emorph::FlowEvent>();
//        if(!v) continue;

//        //add this event
//        procQueue.push_back(v);
//        procType.push_back(1);

//        //then find any events that should be removed
//        int cts = v->getStamp();
//        int cx = v->getX(); int cy = v->getY();
//        emorph::FlowEvent * v2;
//        emorph::vQueue::iterator i = FIFO.begin();
//        while(i != FIFO.end()) {
//            v2 = (*i)->getUnsafe<emorph::FlowEvent>();
//            int modts = cts;
//            if(cts < v2->getStamp()) //we have wrapped
//                modts += emorph::vtsHelper::maxStamp();

//            bool samelocation = v2->getX() == cx && v2->getY() == cy;

//            if(modts > v2->getDeath() || samelocation) {
//                procQueue.push_back(v2);
//                procType.push_back(-1);
//                i = FIFO.erase(i);
//            } else {
//                i++;
//            }
//        }

//        //add to queue
//        FIFO.push_front(v);
//    }

//    //add this event to the hough space
//    updateHough(procQueue, procType);

//}

void vCircleMultiSize::addEdge(emorph::vQueue &additions)
{
    emorph::vQueue procQueue;
    procType.clear();

    emorph::vQueue::iterator qi;
    for(qi = additions.begin(); qi != additions.end(); qi++) {
        emorph::AddressEvent * v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel() != channel) continue;

        emorph::FlowEvent * vf = v->getAs<emorph::FlowEvent>();
        //emorph::FlowEvent * vf = edge.upgradeEvent(v);

        if(vf) {
            procQueue.push_back(vf);
            procType.push_back(1);
        }

        emorph::vQueue removed = eFIFO.addEventToEdge(v);
        for(unsigned int i = 0; i < removed.size(); i++) {
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

    std::vector<vCircleThread *>::iterator i;
    //int dum1, dum2, dum3;
    double v;
    //v = this->getObs(dum1, dum2, dum3);
    v = threshold;

    i = htransforms.begin();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> imagebase =
            (*i)->makeDebugImage(v);

    for(i++; i != htransforms.end(); i++) {
        yarp::sig::ImageOf<yarp::sig::PixelBgr> image = (*i)->makeDebugImage(v);
        for(int y = 0; y < image.height(); y++) {
            for(int x = 0; x < image.width(); x++) {
                if(image(x, y).g > imagebase(x, y).g)
                    imagebase(x, y) = image(x, y);
            }
        }
    }

    emorph::vQueue q;
    if(qType == "fixed")
        q = fFIFO.getSurf();
    else if(qType == "life")
        q = lFIFO.getSurf();
    else if(qType == "time")
        q = tFIFO.getSurf();
    else if(qType == "edge")
        q = eFIFO.getSurf(0, imagebase.width(), 0, imagebase.height());

    for(unsigned int i = 0; i < q.size(); i++) {
        emorph::AddressEvent *v = q[i]->getUnsafe<emorph::AddressEvent>();
        imagebase(v->getY(), imagebase.width() - 1 - v->getX()) =
                yarp::sig::PixelBgr(255, 0, 255);
    }

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

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

using ev::event;
using ev::as_event;
using ev::AddressEvent;
using ev::FlowEvent;

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

void vCircleThread::process(ev::vQueue &procQueue, std::vector<int> &procType)
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

            event<FlowEvent> v = as_event<FlowEvent>((*procQueue)[i]);

            if(v) {
                updateHFlowAngle(v->x, v->y, (*procType)[i],
                                 v->vx, v->vy);
            }

        } else {

            event<AddressEvent> v = as_event<AddressEvent>((*procQueue)[i]);

            if(v) {
                updateHAddress(v->x, v->y, (*procType)[i]);
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
    fFIFO = ev::fixedSurface(fifolength, width, height);
    tFIFO = ev::temporalSurface(width, height, fifolength * 7812.5);
    //eFIFO.setThickness(1);
    //fFIFO.setFixedWindowSize(fifolength);
    //tFIFO.setTemporalSize(fifolength * 7812.5);
    channel = 0;

}

vCircleMultiSize::~vCircleMultiSize()
{

    ev::vQueue dummy1;
    std::vector<int> dummy2;
    std::vector<vCircleThread *>::iterator i;
    for(i = htransforms.begin(); i != htransforms.end(); i++) {
        (*i)->stop();
        (*i)->process(dummy1, dummy2);
        (*i)->waitfordone();
        delete *i;
    }
}

void vCircleMultiSize::addQueue(ev::vQueue &additions) {

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

void vCircleMultiSize::updateHough(ev::vQueue &procQueue, std::vector<int> &procType)
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

void vCircleMultiSize::addFixed(ev::vQueue &additions)
{
    ev::vQueue procQueue;
    procType.clear();
    event<AddressEvent> v = event<AddressEvent>(nullptr);

    ev::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        //GET THE EVENTS AS CORRECT TYPE
        //eventdriven::AddressEvent *v = (*vi)->getAs<eventdriven::AddressEvent>();
        if(directed)
            v = as_event<FlowEvent>(*vi);
        else
            v = as_event<AddressEvent>(*vi);

        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        ev::vQueue removed = fFIFO.addEvent(v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

void vCircleMultiSize::addTime(ev::vQueue &additions)
{
    ev::vQueue procQueue;
    procType.clear();
    event<AddressEvent> v = event<AddressEvent>(nullptr);

    ev::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        v = as_event<AddressEvent>(*vi);
        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        ev::vQueue removed = tFIFO.addEvent(v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

void vCircleMultiSize::addLife(ev::vQueue &additions)
{
    ev::vQueue procQueue;
    procType.clear();
    event<AddressEvent> v = event<AddressEvent>(nullptr);

    ev::vQueue::iterator vi;
    for(vi = additions.begin(); vi != additions.end(); vi++) {

        v = as_event<FlowEvent>(*vi);
        if(!v || v->getChannel() != channel) continue;

        procQueue.push_back(v);
        procType.push_back(1);

        ev::vQueue removed = lFIFO.addEvent(v);

        for(unsigned int i = 0; i < removed.size(); i++) {
            procQueue.push_back(removed[i]);
            procType.push_back(-1);
        }

    }

    updateHough(procQueue, procType);
}

//void vCircleMultiSize::addFixed(eventdriven::vQueue &additions)
//{

//    eventdriven::vQueue procQueue;
//    procType.clear();
//    eventdriven::AddressEvent *v;

//    eventdriven::vQueue::iterator vi;
//    for(vi = additions.begin(); vi != additions.end(); vi++) {

//        //GET THE EVENTS AS CORRECT TYPE
//        //eventdriven::AddressEvent *v = (*vi)->getAs<eventdriven::AddressEvent>();
//        if(directed)
//            v = (*vi)->getAs<eventdriven::FlowEvent>();
//        else
//            v = (*vi)->getAs<eventdriven::AddressEvent>();

//        if(!v || v->getChannel()) continue;

//        procQueue.push_back(v);
//        procType.push_back(1);


//        bool removed = false;

//        //CHECK TO REMOVE "SAME LOCATION EVENTS FIRST"
//        // <----------THIS IS THE SLOWEST PART OF BALL TRACKING -------->
//        int cx = v->X; int cy = v->y;
//        eventdriven::vQueue::iterator i = FIFO.begin();
//        while(i != FIFO.end()) {
//            //we only add Address Events therefore we can do an unsafe cast
//            v = (*i)->getUnsafe<eventdriven::AddressEvent>(); //this may break now?
//            removed = v->X == cx && v->y == cy;
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

//void vCircleMultiSize::addLife(eventdriven::vQueue &additions)
//{
//    eventdriven::vQueue procQueue;
//    procType.clear();

//    eventdriven::vQueue::iterator vi;
//    for(vi = additions.begin(); vi != additions.end(); vi++) {

//        //lifetime requires a flow event only
//        eventdriven::FlowEvent *v = (*vi)->getAs<eventdriven::FlowEvent>();
//        if(!v) continue;

//        //add this event
//        procQueue.push_back(v);
//        procType.push_back(1);

//        //then find any events that should be removed
//        int cts = v->getStamp();
//        int cx = v->X; int cy = v->y;
//        eventdriven::FlowEvent * v2;
//        eventdriven::vQueue::iterator i = FIFO.begin();
//        while(i != FIFO.end()) {
//            v2 = (*i)->getUnsafe<eventdriven::FlowEvent>();
//            int modts = cts;
//            if(cts < v2->getStamp()) //we have wrapped
//                modts += eventdriven::vtsHelper::maxStamp();

//            bool samelocation = v2->X == cx && v2->y == cy;

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

void vCircleMultiSize::addEdge(ev::vQueue &additions)
{
    ev::vQueue procQueue;
    procType.clear();

    ev::vQueue::iterator qi;
    for(qi = additions.begin(); qi != additions.end(); qi++) {
        event<AddressEvent> v = as_event<AddressEvent>(*qi);
        if(!v || v->getChannel() != channel) continue;

        event<FlowEvent> vf = as_event<FlowEvent>(v);

        if(vf) {
            procQueue.push_back(vf);
            procType.push_back(1);
        }

        ev::vQueue removed = eFIFO.addEventToEdge(v);
        for(unsigned int i = 0; i < removed.size(); i++) {
            if(as_event<FlowEvent>(removed[i])) {
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

    ev::vQueue q;
    if(qType == "fixed")
        q = fFIFO.getSurf();
    else if(qType == "life")
        q = lFIFO.getSurf();
    else if(qType == "time")
        q = tFIFO.getSurf();
    else if(qType == "edge")
        q = eFIFO.getSurf(0, imagebase.width(), 0, imagebase.height());

    for(unsigned int i = 0; i < q.size(); i++) {
        event<AddressEvent> v = as_event<AddressEvent>(q[i]);
        imagebase(v->y, imagebase.width() - 1 - v->x) =
                yarp::sig::PixelBgr(255, 0, 255);
    }

    return imagebase;
}



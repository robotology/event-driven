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


/******************************************************************************/
vCircleObserver::vCircleObserver()
{
    spatialRadius = 32;
    inlierThreshold = 5;
    angleThreshold = 0.1;
    radiusThreshold = 1.5;

    window = 0;

}

/******************************************************************************/
vCircleObserver::~vCircleObserver()
{
    if(window) delete window;
}

/******************************************************************************/
void vCircleObserver::init(int width, int height, int temporalRadius,
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
void vCircleObserver::addEvent(emorph::vEvent &event) {
    window->addEvent(event);
}

//double vCircleObserver::RANSAC(double &cx, double &cy, double &cr)
//{
//    //emorph::vEvent *v = window->getMostRecent();
//    //if(!v) return -1;
//    //emorph::AddressEvent *av = v->getAs<emorph::AddressEvent>();

//    emorph::vQueue q = window->getSMARTSURF(sRadius);
//    q.sort();

//    if(q.size() < minVsReq4RANSAC) return -1;

//    int pi1, pi2, pi3;
//    int max_inliers = 0;

//    //this will only work for asynchronous windows
//    emorph::AddressEvent *v1 = window->getMostRecent()->getAs<emorph::AddressEvent>();
//    for(pi1 = 0; pi1 < q.size(); pi1++)
//        if(q[pi1] == v1) break;
//    //emorph::AddressEvent *v1 = q[pi1]->getAs<emorph::AddressEvent>();
//    for(int i = 0; i < iterations; i++) {
//        //choose three random points

//        //pi1 = rand() % q.size();
//        pi2 = pi1;
//        while(pi2 == pi1) {
//            pi2 = rand() % q.size();
//        }
//        pi3 = pi1;
//        while(pi3 == pi1 || pi3 == pi2) {
//           pi3 = rand() % q.size();
//        }

//        emorph::AddressEvent *v2 = q[pi2]->getAs<emorph::AddressEvent>();
//        emorph::AddressEvent *v3 = q[pi3]->getAs<emorph::AddressEvent>();

//        double tx, ty, tr;
//        bool madeCircle = calculateCircle(v1->getX(), v2->getX(), v3->getX(),
//                v1->getY(), v2->getY(), v3->getY(), tx, ty, tr);
//        if(!madeCircle) continue;

//        if(tr < 2*inlierThreshold) continue;
//        int inliers = 0;
//        for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
//            emorph::AddressEvent *vp = (*qi)->getAs<emorph::AddressEvent>();
//            double sqerror = fabs(pow(vp->getX() - tx, 2.0) + pow(vp->getY() - ty, 2.0)
//                                 - pow(tr, 2.0));
//            if(sqerror < inlierThreshold) {
//                inliers++;
//            }
//        }

//        //if(error < min_error) {
//        if(inliers > max_inliers) {
//            max_inliers = inliers;
//            cx = tx; cy = ty; cr = tr;
//        }
//    }

//    return max_inliers;

//}

//double vCircleObserver::oneShotObserve(double &cx, double &cy, double &cr)
//{
//    //emorph::vEvent *v = window->getMostRecent();
//    //if(!v) return -1;
//    //emorph::AddressEvent *av = v->getAs<emorph::AddressEvent>();

//    emorph::vQueue q = window->getSMARTSURF(sRadius);
//    q.sort();

//    if(q.size() < minVsReq4RANSAC) return -1;

//    int pi1, pi2, pi3;

//    //this will only work for asynchronous windows
//    emorph::AddressEvent *v1 = window->getMostRecent()->getAs<emorph::AddressEvent>();
//    for(pi1 = 0; pi1 < q.size(); pi1++)
//        if(q[pi1] == v1) break;

//    if(pi1 == 0) {
//        pi2 = q.size()-1;
//        pi3 = q.size()-2;
//    } else if(pi1 == 1) {
//        pi2 = 0;
//        pi3 = q.size()-1;
//    } else {
//        pi2 = pi1 - 1;
//        pi3 = pi1 - 2;
//    }

//    emorph::AddressEvent *v2 = q[pi2]->getAs<emorph::AddressEvent>();
//    emorph::AddressEvent *v3 = q[pi3]->getAs<emorph::AddressEvent>();


//    bool madeCircle = calculateCircle(v1->getX(), v2->getX(), v3->getX(),
//                                      v1->getY(), v2->getY(), v3->getY(),
//                                      cx, cy, cr);

//    if(!madeCircle) return 0;
//    if(cr < 2*inlierThreshold) return 0;

//    int inliers = 0;
//    for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
//        emorph::AddressEvent *vp = (*qi)->getAs<emorph::AddressEvent>();
//        double sqerror = fabs(pow(vp->getX() - cx, 2.0) + pow(vp->getY() - cy, 2.0)
//                              - pow(cr, 2.0));
//        if(sqerror < inlierThreshold) {
//            inliers++;
//        }
//    }

//    return inliers;

//}


//int vCircleObserver::flowView()
//{
//    if(!window) return -1;
//    emorph::FlowEvent *vr = window->getMostRecent()->
//            getAs<emorph::FlowEvent>();
//    if(!vr) return -1;

//    //get the recent event and the surface
//    emorph::vQueue q = window->getSMARTSURF(spatialRadius);
//    if(q.size() < inlierThreshold) return -1;

//    int RX = -(vr->getX()-spatialRadius); int RY = -(vr->getY()-spatialRadius);
//    int S = 10;

//    //plot the surface
//    double cts = vr->getStamp();
//    cv::Mat G(spatialRadius*2+1, spatialRadius*2+1, CV_8UC1); G.setTo(255);
//    for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
//        emorph::AddressEvent *vp = (*qi)->getAs<emorph::AddressEvent>();

//        double tts = vp->getStamp();
//        if(tts > cts) tts = tts - emorph::vtsHelper::maxStamp();
//        if(cts - tts > 190000) tts = cts - 190000;
//        int val = 254.0 * (cts - tts)/ 200000;
//        G.at<char>(vp->getX()+RX, vp->getY()+RY) = val;
//    }
//    cv::Mat image(G.size(), CV_8UC3);
//    cv::cvtColor(G, image, CV_GRAY2BGR);
//    cv::resize(image, image, image.size()*S, 0, 0, cv::INTER_NEAREST);

//    double main_m = vr->getVy() / vr->getVx();
//    double main_b = vr->getY() - main_m * vr->getX();

//    double xc, yc, rc;
//    double max_inliers = 0;
//    //step through the eigen areas
//    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
//        emorph::FlowEvent * v = (*qi)->getAs<emorph::FlowEvent>();
//        if(!v) continue;
//        if(v == vr) continue;

//        int y = v->getY();
//        int x = v->getX();
//        double dtdy = v->getVy();
//        double dtdx = v->getVx();

//        double flowmag = sqrt(pow(dtdx, 2.0) + pow(dtdy, 2.0)) /
//                emorph::vtsHelper::tstosecs();
//        if(vr->getStamp() - v->getStamp() > flowmag) continue;

//        double vm = dtdy / dtdx;
//        double vb = y - vm * x;
//        if(vm == main_m) continue;

//        double xX = (vb - main_b) / (main_m - vm);
//        double yX = (main_m*vb - vm*main_b) / (main_m - vm);

//        //it can only be a valid cross if both gradients point towards or away
//        if(vr->getVx() * (xX - vr->getX()) * dtdx * (xX - x) < 0) continue;
//        if(vr->getVy() * (yX - vr->getY()) * dtdy * (yX - y) < 0) continue;

//        //if the radius doesn't agree between the two points
//        double main_rad = sqrt(pow(vr->getX()-xX, 2.0) + pow(vr->getY() - yX, 2.0));
//        double rad1 = sqrt(pow(x-xX, 2.0) + pow(y - yX, 2.0));
//        if(fabs(main_rad - rad1) < radiusThreshold) continue;


//        int inliers = 0;
//        for(emorph::vQueue::iterator qi2 = q.begin(); qi2 != q.end(); qi2++) {
//            emorph::FlowEvent * v2 = (*qi2)->getAs<emorph::FlowEvent>();
//            if(!v2) continue;

//            //only allow points with the same side of xX to main_x
//            double vm2 = v2->getVy() / v2->getVx();
//            double vb2 = v2->getY() - vm2 * v2->getX();
//            if(vm2 == main_m) continue;
//            double xX2 = (vb2 - main_b) / (main_m - vm2);
//            double yX2 = (main_m*vb2 - vm2*main_b) / (main_m - vm2);

//            if(vr->getVx() * (xX2 - vr->getX()) * v2->getVx() * (xX2 - v2->getX()) < 0) continue;
//            if(vr->getVy() * (yX2 - vr->getY()) * v2->getVy() * (yX2 - v2->getY()) < 0) continue;

//            //angle1
//            double angle1 = atan2(v2->getVy(), v2->getVx());
//            double angle2 = atan2(yX - v2->getY(), xX - v2->getX());
//            if(v2->getVx()*(xX-v2->getX()) < 0 || v2->getVy()*(yX-v2->getY()) < 0)
//                angle2 = atan2(v2->getY() - yX, v2->getX() - xX);

//            //this needs to account for wrap
//            double adiff = fabs(angle1 - angle2);
//            adiff = std::min(adiff, fabs(angle1 - angle2 - 6.18));
//            adiff = std::min(adiff, fabs(angle1 - angle2 + 6.18));

//            double rad2 = sqrt(pow(v2->getX()-xX, 2.0) + pow(v2->getY() - yX, 2.0));
//            double rdiff = fabs(main_rad - rad2);

//            if(adiff < angleThreshold && rdiff < radiusThreshold) inliers++;

//        }

//        if(inliers > max_inliers) {
//            max_inliers = inliers;
//            xc = xX;
//            yc = yX;
//        }
//    }


//    double main_rad = sqrt(pow(vr->getX()-xc, 2.0) + pow(vr->getY() - yc, 2.0));
//    for(emorph::vQueue::iterator qi2 = q.begin(); qi2 != q.end(); qi2++) {
//        emorph::FlowEvent * v2 = (*qi2)->getAs<emorph::FlowEvent>();
//        if(!v2) continue;

//        //only allow points with the same side of xX to main_x
//        double vm2 = v2->getVy() / v2->getVx();
//        double vb2 = v2->getY() - vm2 * v2->getX();
//        if(vm2 == main_m) continue;
//        double xX2 = (vb2 - main_b) / (main_m - vm2);
//        double yX2 = (main_m*vb2 - vm2*main_b) / (main_m - vm2);

//        if(vr->getVx() * (xX2 - vr->getX()) * v2->getVx() * (xX2 - v2->getX()) < 0) continue;
//        if(vr->getVy() * (yX2 - vr->getY()) * v2->getVy() * (yX2 - v2->getY()) < 0) continue;

//        //angle1
//        double angle1 = atan2(v2->getVy(), v2->getVx());
//        double angle2 = atan2(yc - v2->getY(), xc - v2->getX());
//        if(v2->getVx()*(xc-v2->getX()) < 0 || v2->getVy()*(yc-v2->getY()) < 0)
//            angle2 = atan2(v2->getY() - yc, v2->getX() - xc);

//        //this needs to account for wrap
//        double adiff = fabs(angle1 - angle2);
//        adiff = std::min(adiff, fabs(angle1 - angle2 - 6.18));
//        adiff = std::min(adiff, fabs(angle1 - angle2 + 6.18));

//        double rad2 = sqrt(pow(v2->getX()-xc, 2.0) + pow(v2->getY() - yc, 2.0));
//        double rdiff = fabs(main_rad - rad2);

//        if(adiff < angleThreshold && rdiff < radiusThreshold) {
//            double GS = sqrt(2000.0 / (pow(v2->getVx(), 2.0) + pow(v2->getVy(), 2.0)));

//            cv::Point gp((v2->getY()+RY+1)*S, (v2->getX()+RX+1)*S);
//            cv::Point gvend = gp + cv::Point(S * v2->getVy() * GS, S * v2->getVx() *GS);
//            cv::line(image, gp, gvend, CV_RGB(255, 0, 0));
//            //cv::imshow("FLOW VECTORS", image);
//            //cv::waitKey();
//        }

//    }

//    rc = sqrt(pow(xc - vr->getX(), 2.0) + pow(yc - vr->getY(), 2.0));
//    std::cout << "max_inliers: " << max_inliers << std::endl;


//    double GS = sqrt(2000.0 / (pow(vr->getVx(), 2.0) + pow(vr->getVy(), 2.0)));
//    cv::Point gp((vr->getY()+RY+1)*S, (vr->getX()+RX+1)*S);
//    cv::Point gvend = gp + cv::Point(S * vr->getVy() * GS, S * vr->getVx() *GS);
//    cv::line(image, gp, gvend, CV_RGB(255, 0, 255));

//    cv::circle(image, cv::Point((yc+RY+1)*S, (xc+RX+1)*S), rc*S, CV_RGB(0, 255, 0), 2);
//    cv::flip(image, image, 0);
//    cv::imshow("FLOW VECTORS", image);


//    return max_inliers;
//}

/******************************************************************************/
int vCircleObserver::flowcircle(double &cx, double &cy, double &cr)
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
        double main_rad = sqrt(pow(vr->getX()-xX, 2.0) + pow(vr->getY() - yX, 2.0));
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

            if(vr->getVx() * (xX2 - vr->getX()) * v2->getVx() * (xX2 - v2->getX()) < 0) continue;
            if(vr->getVy() * (yX2 - vr->getY()) * v2->getVy() * (yX2 - v2->getY()) < 0) continue;


            //angle1
            double angle1 = atan2(v2->getVy(), v2->getVx());
            double angle2 = atan2(yX - v2->getY(), xX - v2->getX());
            if(v2->getVx()*(xX-v2->getX()) < 0 || v2->getVy()*(yX-v2->getY()) < 0)
                angle2 = atan2(v2->getY() - yX, v2->getX() - xX);

            //this needs to account for wrap
            double adiff = fabs(angle1 - angle2);
            adiff = std::min(adiff, fabs(angle1 - angle2 - 6.18));
            adiff = std::min(adiff, fabs(angle1 - angle2 + 6.18));

            double rad2 = sqrt(pow(v2->getX()-xX, 2.0) + pow(v2->getY() - yX, 2.0));
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
bool vCircleObserver::calculateCircle(double x1, double x2, double x3,
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



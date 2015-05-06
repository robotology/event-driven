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
#include <vector>
#include <limits>

void vCircleObserver::createLocalSearch(int x, int y)
{
    localActivity.clear();
    int tminx = std::max(x-sRadius, 0);
    int tmaxx = std::min(x+sRadius, width -1);
    int tminy = std::max(y-sRadius, 0);
    int tmaxy = std::min(y+sRadius, height-1);
    for(int v = tminy; v <= tmaxy; v++) {
        for(int u = tminx; u <= tmaxx; u++) {
            double a = activity.queryActivity(u, v);
            if(a > 1)
                localActivity.push_back(act_unit(u, v, a));
        }
    }
}

void vCircleObserver::pointTrim(int x, int y)
{
    std::vector<act_unit>::iterator i = localActivity.begin();
    while(i != localActivity.end()) {
        if(sqrt(std::pow(x - i->x, 2.0) + std::pow(y - i->y, 2)) < tRadius)
            localActivity.erase(i);
        else
            i++;
    }
}

void vCircleObserver::linearTrim(int x1, int y1, int x2, int y2)
{
    std::vector<act_unit>::iterator i = localActivity.begin();
    if(x2 == x1) {
        //if we have a vertical line just check x value
        while(i != localActivity.end()) {
            if(std::fabs(i->x - x1) < tRadius)
                localActivity.erase(i);
            else
                i++;
        }
    } else {
        //otherwise we have to calculate the line parameters
        double m = (y2 - y1) / (double)(x2 - x1);
        double b = y1 - m * x1;
        while(i != localActivity.end()) {
            if(std::fabs(i->y - m*i->x - b) < tRadius)
                localActivity.erase(i);
            else
                i++;
        }
    }
}

bool vCircleObserver::localCircleEstimate(emorph::AddressEvent &event, double &cx,
                                  double &cy, double &cr, bool showDebug)
{
    act_unit p1(event.getX(), event.getY(), 0);
    //update the activity here
    p1.a = activity.addEvent(event);

    //create our search locations
    createLocalSearch(p1.x, p1.y);

    //remove points too close to centre
    pointTrim(p1.x, p1.y);
    if(localActivity.size() < 3) return false;

    //find the best score
    int bi = 0;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > localActivity[bi].a) bi = i;
    act_unit p2 = localActivity[bi];

    //remove activity close-by the first point
    linearTrim(p1.x, p1.y, p2.x, p2.y);
    if(localActivity.empty()) return false;

    //find the second best score
    bi = 0;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > localActivity[bi].a) bi = i;
    act_unit p3 = localActivity[bi];

    //if we are all on the same y line (should be impossible after linearTrim)
    if(p1.y == p2.y && p1.y == p3.y) return false;

    //make sure x2 is different to x1 and x3 (else we divide by 0 later)
    act_unit * ex = 0;

    if(p2.x == p1.x) {
        if(p2.x == p3.x)
            return 0;
        ex = &p3;
    } else if(p2.x == p3.x) {
        ex = &p1;
    }

    if(ex) {
        act_unit temp = p2;
        p2 = *ex;
        *ex = temp;
    }

    //calculate the circle from the 3 points
    double ma = (p2.y - p1.y) / (double)(p2.x - p1.x);
    double mb = (p3.y - p2.y) / (double)(p3.x - p2.x);

    cx = (ma * mb * (p1.y - p3.y) + mb * (p1.x + p2.x) -
                        ma * (p2.x + p3.x)) / (2 * (mb - ma));
    if(ma)
        cy = -1 * (cx - (p1.x+p2.x)/2.0)/ma + (p1.y+p2.y)/2.0;
    else
        cy = -1 * (cx - (p2.x+p3.x)/2.0)/mb + (p2.y+p3.y)/2.0;

    cr = sqrt(pow(cx - p1.x, 2.0) + pow(cy - p1.y, 2.0));



    if(cx < 0 || cx > width-1 || cy < 0 || cy > height-1) return false;


    if(showDebug && stepbystep) {

        //%%% display centre and radius lines.
        static int divider = 0;
        if(++divider % 1 == 0) {

            double mact = 0;
            cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
            for(int x = 0; x < 128; x++) {
                for(int y = 0; y < 128; y++) {
                    double a = activity.queryActivity(x, y);
                    mact = std::max(a, mact);
                    if(a > 0.01)
                        image2.at<float>(x, y) = a;
                }
            }

            image2 = image2 * (2/mact);
            cv::Mat i8u(128, 128, CV_8U); image2.copyTo(i8u);
            cv::Mat image(128, 128, CV_8UC3); cv::cvtColor(i8u, image, CV_GRAY2BGR);
            cv::resize(image, image, cv::Size(512, 512), 0, 0, CV_INTER_NN);


            cv::circle(image, cv::Point(p1.y, p1.x)*4, 12, CV_RGB(255, 255, 255));
            cv::circle(image, cv::Point(p2.y, p2.x)*4, 12, CV_RGB(255, 255, 255));
            cv::circle(image, cv::Point(p3.y, p3.x)*4, 12, CV_RGB(255, 255, 255));
            cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(p1.y, p1.x)*4, CV_RGB(255, 255, 255));
            cv::line(image, cv::Point(p1.y, p1.x)*4, cv::Point(p3.y, p3.x)*4, CV_RGB(255, 255, 255));
            cv::line(image, cv::Point(p1.y, p1.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
            cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
            cv::line(image, cv::Point(p3.y, p3.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
            cv::circle(image, cv::Point(cy, cx)*4, 5, CV_RGB(255, 0, 0), CV_FILLED);



            if(divider % 1  == 0) {
                int xsm = std::max(p1.x-sRadius, 0)*4;
                int ysm = std::max(p1.y-sRadius, 0)*4;
                int wsm = std::min(128 - xsm/4, 2*sRadius)*4;
                int hsm = std::min(128 - ysm/4, 2*sRadius)*4;

                cv::Mat submat;
                image(cv::Rect(ysm, xsm, hsm, wsm)).copyTo(submat);
                cv::flip(submat, submat, 0);
                cv::resize(submat, submat, cv::Size(0, 0), 4, 4, CV_INTER_NN);
                cv::imshow("Local", submat);
            }

            cv::flip(image, image, 0);
            cv::imshow("Activity", image);
            char c = cv::waitKey(0);
            if(c == 27) stepbystep = false;
        }
    }


    return true;

}

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

    cr = sqrt(pow(cx - x1, 2.0) + pow(cy - y1, 2.0));

    if(cx != cx) {
        std::cout << "error: cx == NaN" << std::endl;
    }
    if(cx == INFINITY || cx == -INFINITY) {
        std::cout << "error: cx == INF" << std::endl;
    }

    return true;

}

void vCircleObserver::addEvent(emorph::vEvent &event) {
    window.addEvent(event);
}

double vCircleObserver::RANSAC(double &cx, double &cy, double &cr, bool debug, cv::Mat *image)
{
    emorph::vEvent *v = window.getMostRecent();
    if(!v) return -1;
    emorph::AddressEvent *av = v->getAs<emorph::AddressEvent>();

    const emorph::vQueue& q =
            window.getSpatialWindow(av->getX(), av->getY(), sRadius);

    if(q.size() < minVsReq4RANSAC) return -1;
    int pi1, pi2, pi3;
    double min_error = std::numeric_limits<double>::max();
    int max_inliers = 0;
    for(int i = 0; i < iterations; i++) {
        //choose three random points
        pi1 = q.size()-1;
        //pi1 = rand() % q.size();
        pi2 = pi1;
        while(pi2 == pi1) {
            pi2 = rand() % q.size();
        }
        pi3 = pi1;
        while(pi3 == pi1 || pi3 == pi2) {
           pi3 = rand() % q.size();
        }
        emorph::AddressEvent *v1 = q[pi1]->getAs<emorph::AddressEvent>();
        emorph::AddressEvent *v2 = q[pi2]->getAs<emorph::AddressEvent>();
        emorph::AddressEvent *v3 = q[pi3]->getAs<emorph::AddressEvent>();

        double tx, ty, tr;
        bool madeCircle = calculateCircle(v1->getX(), v2->getX(), v3->getX(),
                v1->getY(), v2->getY(), v3->getY(), tx, ty, tr);
        if(!madeCircle) continue;

        if(tr < 8 || tr > 32) continue;

        double error = 0;
        int inliers = 0;
        for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
            emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
            double terror = fabs(pow(v->getX() - tx, 2.0) + pow(v->getY() - ty, 2.0)
                                 - pow(tr, 2.0));
            error += terror;
            if(sqrt(terror) < inlierThreshold) {
                inliers++;
            }
        }

        //if(error < min_error) {
        if(inliers > max_inliers) {
            min_error = error;
            max_inliers = inliers;
            cx = tx; cy = ty; cr = tr;
        }
    }

    if(debug && image && max_inliers >= minVsReq4RANSAC) {

        //cv::Mat image(128, 128, CV_8UC3); image.setTo(0);

        //if(max_inliers > minVsReq4RANSAC) {
            //cv::circle(*image, cv::Point(cy, cx), cr, CV_RGB(255, 0, 0), inlierThreshold * 2);
            cv::circle(*image, cv::Point(cy, cx), cr-inlierThreshold, CV_RGB(0, 0, 255));
            cv::circle(*image, cv::Point(cy, cx), cr+inlierThreshold, CV_RGB(0, 0, 255));
        //}

//        for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
//            emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
//            image.at<cv::Vec3b>(v->getX(), v->getY()) = cv::Vec3b(255, 255, 255);
//        }
        //cv::rectangle(image, cv::Rect(), CV_RGB(0, 0, 255));


        //cv::flip(image, image, 0);
        //cv::resize(image, image, image.size()*2, 0, 0, CV_INTER_NN);
        //cv::imshow("RANSAC", image);
        //cv::waitKey(1);

        //std::cout << q.size() << " " << min_error / q.size() << " "
        //          << max_inliers << std::endl;

\
    }

    return max_inliers;

}

double vCircleObserver::globalInlierCount(double cx, double cy, double cr)
{
    const emorph::vQueue& q = window.getSpatialWindow(cx, cy, cr+inlierThreshold);
    int inliers = 0;
    for(emorph::vQueue::const_iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        double terror = fabs(pow(v->getX() - cx, 2.0) + pow(v->getY() - cy, 2.0)
                             - pow(cr, 2.0));
        if(sqrt(terror) < inlierThreshold) {
            inliers++;
        }
    }
    return inliers;
}

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

vCircleTracker::vCircleTracker(double svPos, double svSiz, double zvPos, double zvSiz)
{
    this->svPos = svPos;
    this->svSiz = svSiz;
    active = false;
    pTS = 0;

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

vCircleTracker::~vCircleTracker()
{
    delete filter;
}

//add event will:
//1. check closeness to distribution
//2. update zq if < c
//3. perform correction based on observation from zq
//4. destroy if validation gate too high
//5. return whether, updated, non-updated, needs destroying
double vCircleTracker::addEvent(emorph::AddressEvent v, double cT)
{

    //get our delta t
    double ts = unwrap(v.getStamp());
    double dt = 0;
    if(!pTS) {
        pTS = ts;
    } else {
        dt = (ts - pTS) * unwrap.tstosecs();
        pTS = ts;
    }

    //predict
    predict(dt);
    yarp::sig::Vector state = filter->get_x();
    if(state[0] != state[0]) {
        std::cout << "error" << std::endl;
    }

    //assume it is part of the tracker (probability = 1)
    double cV = 1;
    if(active) {
        //calculate probability observation belongs to this tracker
        cV = Pvgd(v.getX(), v.getY());

    } else {
        //some other measure whether this belongs to a circle otherwise we
        //would make circles from any three random points

    }

    std::cout << cV << " <--> " << cT << std::endl;
    if(cV < cT) {
        return 0;
    }



    //either it's not active or the z fits the distribution
    zq.push_front(v.clone());

    //still not active
    if(zq.size() < 3)
        return 0;

    //or we throw out old observations
    if(zq.size() > 3) {
        delete zq.back();
        zq.pop_back();
    }


    //calculate the circle
    double cx, cy, cr;
    bool madecircle = makeObservation(cx, cy, cr);


    //if we didn't make a circle just exit?
    if(!madecircle)
        return 0;

    //adding observation
    std::cout << "NEW OBSERVATION" << std::endl;

    //either correct our position or start tracking
    yarp::sig::Vector z(6, 0.0); z[0]=cx; z[1]=cy; z[2]=cr;
    if(active) {
        filter->correct(z);
        yarp::sig::Vector state = filter->get_x();
        if(state[0] != state[0]) {
            std::cout << "error" << std::endl;
        }
    } else {
        yarp::sig::Matrix P0 = filter->get_R();
        P0(3, 3) = P0(0, 0); P0(4, 4) = P0(1, 1); P0(5, 5) = P0(2, 2);
        filter->init(z, P0);
        active = true;
    }

    //return our tracking strength
    //this might not be a good measure if we only add observations that agree
    //with the distribution in the first place
    return filter->get_ValidationGate();

}

double vCircleTracker::correct(double xz, double yz, double rz, double dt)
{
    if(!active) return 0;
    yarp::sig::Vector z(9, 0.0); z[0]=xz; z[1]=yz; z[2]=rz;
    filter->correct(z);
    return 0;
}

double vCircleTracker::init(double xz, double yz, double rz)
{
    yarp::sig::Vector x0(9, 0.0); x0[0] = xz; x0[1] = yz; x0[2] = rz;
    yarp::sig::Matrix P0 = filter->get_R();
    P0(3, 3) = P0(0, 0); P0(4, 4) = P0(1, 1); P0(5, 5) = P0(2, 2);
    P0(4, 4) = P0(0, 0); P0(5, 5) = P0(1, 1); P0(6, 6) = P0(2, 2);
    filter->init(x0, P0);
    active = true;
    return 0;
}

//update state
//also update zq values based on velocity
double vCircleTracker::predict(double dt)
{

    if(!active) return 0;
    //update the tracker position
    yarp::sig::Matrix A = filter->get_A();
    double dt2 = 0.5 * pow(dt, 2.0);
    A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
    A(3, 6) = dt; A(4, 7) = dt; A(5, 8) = dt;
    A(0, 6) = dt2; A(1, 7) = dt2; A(2, 8) = dt2;
    filter->set_A(A);

    yarp::sig::Matrix Q = filter->get_Q();
    Q(6, 6) = svPos*dt2; Q(7, 7) = svPos*dt2; Q(8, 8) = svSiz*dt2;
    filter->set_Q(Q);

    filter->predict();

    //update the observations according to tracker movement
//    yarp::sig::Vector x = filter->get_x();
//    emorph::vQueue::iterator vi;
//    for(vi = zq.begin(); vi != zq.end(); vi++) {
//        emorph::AddressEvent *v = (*vi)->getAs<emorph::AddressEvent>();
//        if(!v) continue;
//        v->setX(v->getX() + x(0) * dt);
//        v->setY(v->getY() + x(1) * dt);
//    }


    return 0;
}

double vCircleTracker::Pvgd(double xv, double yv)
{
    if(!active) return 0;

    yarp::sig::Vector c = filter->get_x();
    double xc = c[0], yc = c[1], rc = c[2];

    //calculate the distance to the mean of the distribution
    double rv = sqrt(pow(yv-yc, 2.0) + pow(xv-xc, 2.0));
    if(rv == 0) return 0;
    double xd = xc + (rc / rv) * (xv - xc); //trouble here if rv == 0
    double yd = yc + (rc / rv) * (yv - yc);

    double x = fabs(xv - xd);
    double y = fabs(yv - yd);

    yarp::sig::Matrix P = filter->get_P();
    double distance = P(0, 0) * pow(x, 2.0) + 2 * P(0, 1) * x * y +
            P(1, 1) * pow(y, 2.0);
    distance = pow(x, 2.0) + pow(y, 2.0);
    double det = pow(P(0, 0), 2.0) + pow(P(1, 1), 2.0) + 2 * P(0, 0) * P(1, 1) *
            P(0, 1) + pow(P(2, 2), 2.0);

    double pvgd = exp(-0.5 * distance / det);
    return pvgd;// / sqrt(det * 6.283);
}

double vCircleTracker::Pzgd(double xz, double yz, double rz)
{
    if(!active) return 0;

    yarp::sig::Vector x = filter->get_x();
    yarp::sig::Matrix P = filter->get_P();

    double Px = exp(-0.5 * pow(xz - x[0], 2.0) / pow(P(0, 0), 2.0));
    double Py = exp(-0.5 * pow(yz - x[1], 2.0) / pow(P(1, 1), 2.0));
    double Pr = exp(-0.5 * pow(rz - x[2], 2.0) / pow(P(2, 2), 2.0));

    return std::min(Px, std::min(Py, Pr));

}

bool vCircleTracker::makeObservation(double &cx, double &cy, double &cr)
{
    //extract the points
    if(zq.size() < 3) return false;
    emorph::AddressEvent *v1 = zq[0]->getAs<emorph::AddressEvent>();
    emorph::AddressEvent *v2 = zq[1]->getAs<emorph::AddressEvent>();
    emorph::AddressEvent *v3 = zq[2]->getAs<emorph::AddressEvent>();
    double x1 = v1->getX(), y1 = v1->getY();
    double x2 = v2->getX(), y2 = v2->getY();
    double x3 = v3->getX(), y3 = v3->getY();

    //if we are all on the same line we can't compute a circle
    if(y1 == y2 && y1 == y3) return false;
    if(x1 == x2 && x1 == x3) return false;
    if(x1 == x2 && y1 == y2) return false;
    if(x1 == x3 && y1 == y3) return false;
    if(x2 == x3 && y2 == y3) return false;

    //make sure x2 is different to x1 and x3 (else we divide by 0 later)
    if(x2 == x1) {
        x3 = x2; y3 = x2;
        x2 = v3->getX(); y2 = v3->getY();
    } else if(x2 == x3) {
        x1 = x2; y1 = y2;
        x2 = v1->getX(); y2 = v1->getY();
    }


    //calculate the circle from the 3 points
    double ma = (y2 - y1) / (x2 - x1);
    double mb = (y3 - y2) / (x3 - x2);

    if(ma == mb) return false;
    if(ma != ma || mb != mb) {
        std::cout << "error" << std::endl;
    }

    cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) -
                        ma * (x2 + x3)) / (2 * (mb - ma));
    if(ma)
        cy = -1 * (cx - (x1+x2)/2.0)/ma + (y1+y2)/2.0;
    else
        cy = -1 * (cx - (x2+x3)/2.0)/mb + (y2+y3)/2.0;

    cr = sqrt(pow(cx - x1, 2.0) + pow(cy - y1, 2.0));

    if(cx != cx) {
        std::cout << "error" << std::endl;
    }
    if(cx == INFINITY || cx == -INFINITY) {
        std::cout << "error" << std::endl;
    }

    return true;

}




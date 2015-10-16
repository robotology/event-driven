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

#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <iCub/ctrl/kalman.h>

/*////////////////////////////////////////////////////////////////////////////*/
//VHOUGHCIRCLEOBSERVER
/*////////////////////////////////////////////////////////////////////////////*/
class vHoughCircleObserver
{

private:

    //data
    std::vector<yarp::sig::Matrix *> H;
    std::vector<double> posThreshs;
    std::vector<double> negThreshs;
    std::vector<double> rot;
    std::vector<double> err;
    emorph::vQueue FIFO;

    //parameters
    int qlength;
    int qduration;

    int r1;
    int r2;
    int rsize;
    int height;
    int width;

    bool updateH(emorph::vEvent &event, int val);
    //two methods of general function above
    void updateHAddress(int xv, int yv, std::vector<double> &threshs);
    void updateHFlow(int xv, int yv, std::vector<double> &threshs,
                     double dtdx, double dtdy);
    void updateHFlowAngle(int xv, int yv, std::vector<double> &threshs,
                          double dtdx, double dtdy);


public:

    bool valid;
    bool useFlow;
    std::string qType;
    int xc, yc, rc;
    double valc;

    double * obs_max;
    int x_max, y_max, r_max;

    vHoughCircleObserver();
    ~vHoughCircleObserver();

    void addEvent(emorph::vEvent &event);
    //three methods of general function above
    void addEventFixed(emorph::vEvent &event);
    void addEventTime(emorph::vEvent &event);
    void addEventLife(emorph::vEvent &event);
    double getMaximum(int &x, int &y, int &r);


    yarp::sig::ImageOf<yarp::sig::PixelMono> makeDebugImage(int r);
    yarp::sig::ImageOf<yarp::sig::PixelMono> makeDebugImage2();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> makeDebugImage3(int s = 4);
    yarp::sig::ImageOf<yarp::sig::PixelBgr> makeDebugImage4();


};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLETHREAD
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleThread : public yarp::os::Thread
{

private:

    //parameters
    int R;
    double Rsqr;
    bool directed;
    int height;
    int width;

    //data
    yarp::sig::Matrix H;
    std::vector<double> rot;
    std::vector<double> err;
    double normedStrength;
    int x_max, y_max;
    bool valid;

    //current data
    emorph::vEvent cEvent;
    double signedStrength;

    void updateHAddress(int xv, int yv, double strength);
    void updateHFlowAngle(int xv, int yv, double strength, double dtdx,
                          double dtdy);

    virtual void run();

public:

    vCircleThread(int R, bool directed, int height = 128, int width = 128);

    void addEvent(emorph::vEvent &event);
    void removeEvent(emorph::vEvent &event);

    bool wasUpdated() { return valid; }
    double getScore() { return H[y_max][x_max]; }
    int getX() { return x_max; }
    int getY() { return y_max; }
    int getR() { return R; }

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMULTISIZE
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleMultiSize
{

private:

    std::vector<vCircleThread *> htransforms;

public:

    vCircleMultiSize(int rLow = 8, int rHigh = 38, bool directed = true,
                     int height = 128, int width = 128);

    void addEvent(emorph::vEvent &event);
    bool getObs(int &x, int &y, int &r);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleTracker
{
private:

    iCub::ctrl::Kalman *filter;
    bool active;

    //system variance
    double svPos;
    double svSiz;

    //private functions
    //double Pvgd(double xv, double yv);

public:

    vCircleTracker();
    ~vCircleTracker();

    void init(double svPos, double svSiz, double zvPos, double zvSiz);

    bool startTracking(double xz, double yz, double rz);

    double predict(double dt);
    bool correct(double xz, double yz, double rz);
    bool getState(double &x, double &y, double &r);
    double Pzgd(double xz, double yz, double rz);

    bool isActive() { return active; }

};

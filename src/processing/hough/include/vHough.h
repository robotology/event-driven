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

#include <iCub/emorph/all.h>
//#include <opencv2/opencv.hpp>
#include <iCub/ctrl/kalman.h>

class vHoughCircle
{

private:

    //data
    std::vector<yarp::sig::Matrix *> H;
    emorph::vQueue FIFO;

    //parameters
    int qlength;
    int qduration;

    int r1;
    int r2;
    int rsize;
    int height;
    int width;

    void updateH(int xv, int yv, int val);
    void updateHwithFlow(int xv, int yv, int val, double dtdx, double dtdy);


public:

    bool found;
    int xc, yc, rc, valc;

    vHoughCircle();
    ~vHoughCircle();

    void addEventFixed(emorph::vEvent &event);
    void addEventTime(emorph::vEvent &event);
    void addEventLife(emorph::vEvent &event);

    yarp::sig::ImageOf<yarp::sig::PixelMono> makeDebugImage(int r);
    yarp::sig::ImageOf<yarp::sig::PixelMono> makeDebugImage2();


};

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

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



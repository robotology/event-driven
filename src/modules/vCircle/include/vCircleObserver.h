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

#ifndef __VCIRCLE_OBS__
#define __VCIRCLE_OBS__

#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLETHREAD
/*////////////////////////////////////////////////////////////////////////////*/
///
/// \brief The vCircleThread class performs a circular Hough transform
///
/// The class gives the maximal location and strength of a circular shape of a
/// single given radius. The class can use the directed transform and can be
/// threaded for use on multi-core systems.
///
class vCircleThread : public yarp::os::Thread
{

private:

    //parameters
    int R; /// the Hough Radius (pixels)
    bool directed; /// use the directed Hough transform
    bool threaded; /// perform calculations in a separate thread
    int height; /// sensor height
    int width; /// sensor width

    //data
    //yarp::sig::Matrix H; /// stores the Hough strength over the sensor plane
    yarp::sig::ImageOf<yarp::sig::PixelMono16> H;
    double a; /// length of tangent line to the directed Hough arc
    double Rsqr; /// precompute the square of the radius
    double Hstr; /// normalised Hough strength given the radius
    int x_max; /// strongest response along the x axis
    int y_max; /// strongest response along the y axis
    yarp::sig::ImageOf<yarp::sig::PixelBgr> canvas;
    std::vector<int> hx;
    std::vector<int> hy;
    std::vector<int> hang;

    yarp::os::Mutex mstart; /// for thread safety when starting computation
    yarp::os::Mutex mdone; /// for thread safety when computation is finished

    //current data
    ev::vQueue * procQueue; /// pointer to list of events to add to Hough space
    std::vector<int> * procType; /// pointer to list of events to remove from Hough

    /// update the Hough space using standard method
    void updateHAddress(int xv, int yv, int strength);

    /// update the Hough space using directed method
    double updateHFlowAngle(int xv, int yv, int strength, double dtdx,
                          double dtdy);
    double updateHFlowAngle2(int xv, int yv, double strength,
                                         double dtdx, double dtdy);

    /// update the Hough space given adds and subs
    void performHough();

    /// calls performHough in separate thread
    virtual void run();

    /// wait for call to perform Hough when threaded
    void waitforstart() { mstart.lock(); }

    /// tell the calling function that the computation has ended
    void signalfinish() { mdone.unlock(); }

public:

    ///
    /// \brief vCircleThread constructor
    /// \param R circle radius
    /// \param directed use directed Hough transform
    /// \param parallel use a separate thread for computation
    /// \param height sensor height
    /// \param width sensor width
    ///
    vCircleThread(int R, bool directed, bool parallel = false, int height = 128, int width = 128, double arclength = 15);

    ///
    /// \brief getScore get the maximum strength in Hough space
    /// \return the maximum strength in Hough space
    ///
    double getScore() { return H(y_max, x_max) * Hstr; }
    ///
    /// \brief getX get the maximum strength location
    /// \return maximum strength location along x axis
    ///
    int getX() { return x_max; }
    ///
    /// \brief getY get the maximum strength location
    /// \return maximum strength location along y axis
    ///
    int getY() { return y_max; }
    ///
    /// \brief getR return the radius of the circle to be detected
    /// \return the radius R
    ///
    int getR() { return R; }

    ///
    /// \brief process update the Hough transform (threaded or non-threaded)
    /// \param adds list of events to add
    /// \param subs list of events to remove
    ///
    void process(ev::vQueue &procQueue, std::vector<int> &procType);

    ///
    /// \brief waitfordone wait for computation to finish if threaded
    ///
    void waitfordone();

    int findScores(std::vector<double> &values, double threshold);

    ///
    /// \brief makeDebugImage create an image visualising the Hough space
    /// \return a BGR yarp image
    ///
    yarp::sig::ImageOf<yarp::sig::PixelBgr> makeDebugImage(double refval = -1);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMULTISIZE
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleMultiSize
{

private:

    //parameters
    std::string qType;
    int qduration;
    double threshold;
    int fifolength;
    bool directed;
    int channel; //circle detection shouldn't really care about channel!
    //channel splitting should be done at a higher level

    //internal data
    ev::vEdge eFIFO;
    ev::fixedSurface fFIFO;
    ev::temporalSurface tFIFO;
    ev::lifetimeSurface lFIFO;
    ev::event<> dummy;
    std::vector<vCircleThread *> htransforms;
    std::vector<vCircleThread *>::iterator best;
    std::vector<int> procType;

    void addHough(ev::event<> event);
    void remHough(ev::event<> event);
    void updateHough(ev::vQueue &procQueue, std::vector<int> &procType);

    void addFixed(ev::vQueue &additions);
    void addTime(ev::vQueue &additions);
    void addLife(ev::vQueue &additions);
    void addEdge(ev::vQueue &additions);

public:

    vCircleMultiSize(double threshold, std::string qType = "edge",
                     int rLow = 8, int rHigh = 38,
                     bool directed = true, bool parallel = false,
                     int height = 128, int width = 128, int arclength = 20,
                     double fifolength = 2000);
    ~vCircleMultiSize();

    void setChannel(int channelNumber) { channel = channelNumber; }
    void addQueue(ev::vQueue &additions);
    double getObs(int &x, int &y, int &r);
    std::vector<double> getPercentile(double p, double thMin);
    yarp::sig::ImageOf<yarp::sig::PixelBgr> makeDebugImage();

};

#endif

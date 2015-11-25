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
    int Hscale;

    //data
    yarp::sig::Matrix H; /// stores the Hough strength over the sensor plane
    double a; /// length of tangent line to the directed Hough arc
    double Rsqr; /// precompute the square of the radius
    double Hstr; /// normalised Hough strength given the radius
    int x_max; /// strongest response along the x axis
    int y_max; /// strongest response along the y axis

    yarp::os::Mutex mstart; /// for thread safety when starting computation
    yarp::os::Mutex mdone; /// for thread safety when computation is finished

    //current data
    emorph::vQueue * procQueue; /// pointer to list of events to add to Hough space
    std::vector<int> * procType; /// pointer to list of events to remove from Hough

    /// update the Hough space using standard method
    void updateHAddress(int xv, int yv, double strength);

    /// update the Hough space using directed method
    double updateHFlowAngle(int xv, int yv, double strength, double dtdx,
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
    vCircleThread(int R, bool directed, bool parallel = false, int height = 128, int width = 128, int scale = 1, double arclength = 20);

    ///
    /// \brief getScore get the maximum strength in Hough space
    /// \return the maximum strength in Hough space
    ///
    double getScore() { return H[y_max][x_max]; }
    ///
    /// \brief getX get the maximum strength location
    /// \return maximum strength location along x axis
    ///
    int getX() { return x_max / Hscale; }
    ///
    /// \brief getY get the maximum strength location
    /// \return maximum strength location along y axis
    ///
    int getY() { return y_max / Hscale; }
    ///
    /// \brief getR return the radius of the circle to be detected
    /// \return the radius R
    ///
    int getR() { return R / Hscale; }

    ///
    /// \brief process update the Hough transform (threaded or non-threaded)
    /// \param adds list of events to add
    /// \param subs list of events to remove
    ///
    void process(emorph::vQueue &procQueue, std::vector<int> &procType);

    ///
    /// \brief waitfordone wait for computation to finish if threaded
    ///
    void waitfordone();

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
    int qlength;
    int qduration;

    //internal data
    emorph::vSurface surface;
    emorph::vEdge edge;
    emorph::vQueue FIFO;
    emorph::vEvent dummy;
    std::vector<vCircleThread *> htransforms;
    std::vector<vCircleThread *>::iterator best;

    void addHough(emorph::vEvent &event);
    void remHough(emorph::vEvent &event);
    void updateHough(emorph::vQueue &procQueue, std::vector<int> &procType);

    void addFixed(emorph::vQueue &additions);
    void addTime(emorph::vQueue &additions);
    void addLife(emorph::vQueue &additions);
    void addSurf(emorph::vQueue &additions);
    void addEdge(emorph::vQueue &additions);

public:

    vCircleMultiSize(std::string qType = "edge", int qLength = 2000, int rLow = 8, int rHigh = 38,
                     bool directed = true, bool parallel = false, int height = 128, int width = 128);
    ~vCircleMultiSize();

    void addQueue(emorph::vQueue &additions);
    double getObs(int &x, int &y, int &r);
    yarp::sig::ImageOf<yarp::sig::PixelBgr> makeDebugImage();

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

#ifndef __FLOWVIEWER_H__
#define __FLOWVIEWER_H__

#include "VelocityBuffer.h"
#include "VelocityGrabber.h"

#include <cmath>
#include <iostream>

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageDraw.h>

#define XDIM 128
#define YDIM 128
#define FILTER_NGHBRHD 3
#define MEDIAN_NGHBRHD 2

using namespace std;
using namespace yarp::os;

class VelocityGrabber;

class flowViewer : public RateThread{
    yarp::sig::ImageOf<yarp::sig::PixelMono16> vecBaseImg;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> normBaseImg;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * outImagePort;
	BufferedPort<Bottle> * outDataPort;
	double data_x [XDIM][YDIM];
	double data_y [XDIM][YDIM];

    VelocityGrabber * inPort;

    yarp::sig::Matrix xVels;
    yarp::sig::Matrix yVels;
    yarp::sig::Matrix timeRec;

    int visMthd;

    double kth_smallest(double * a, int n, int k);

    void medianFilter2D(VelocityBuffer &);
    void medianFilterSeprabale(VelocityBuffer & );

//    double velR, velL, avgR, avgL;
//    int cntrR, cntrL;
//    int zeroFlag;

public:

    flowViewer(int visMethod = 1);
    void setPorts(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * oPort, VelocityGrabber * iPort, BufferedPort<Bottle> * oDataPort);
    void setVisMethod(int method){visMthd = method;}
    virtual ~flowViewer();

    virtual void run();

    void avrageFilter(VelocityBuffer& data);


    void velVect(VelocityBuffer &);
    void velNorm(VelocityBuffer& data);


};



#endif //OPTICALFLOWVIEWER

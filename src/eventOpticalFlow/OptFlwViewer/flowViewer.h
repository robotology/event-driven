#ifndef __FLOWVIEWER_H__
#define __FLOWVIEWER_H__

#include "VelocityBuffer.h"


#include <cmath>
#include <iostream>

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

class flowViewer {
    yarp::sig::ImageOf<yarp::sig::PixelMono16> vecBaseImg;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> normBaseImg;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * outPort;

    yarp::sig::Matrix xVels;
    yarp::sig::Matrix yVels;
    yarp::sig::Matrix timeRec;


    double kth_smallest(double * a, int n, int k);

    void medianFilter2D(VelocityBuffer &);
    void medianFilterSeprabale(VelocityBuffer & );

//    double velR, velL, avgR, avgL;
//    int cntrR, cntrL;
//    int zeroFlag;

public:


    flowViewer();
    void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * );
    ~flowViewer();

    void avrageFilter(VelocityBuffer& data);


    void run(VelocityBuffer &);
    void velNorm(VelocityBuffer& data);


};

#endif //OPTICALFLOWVIEWER

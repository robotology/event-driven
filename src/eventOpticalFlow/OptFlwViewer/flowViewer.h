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
#define MEDIAN_NGHBRHD 1

using namespace std;
using namespace yarp::os;

class flowViewer {
    yarp::sig::ImageOf<yarp::sig::PixelMono16> mBaseImg;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * outPort;

    yarp::sig::Matrix xVels;
    yarp::sig::Matrix yVels;
    int zeroFlag;

    double kth_smallest(double * a, int n, int k);

    void medianFilter2D(VelocityBuffer &);
    void medianFilterSeprabale(VelocityBuffer & );

    void resetVelMtrx();

public:
    flowViewer();
    void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * );
    ~flowViewer();
    void run(VelocityBuffer &);


};

#endif //OPTICALFLOWVIEWER

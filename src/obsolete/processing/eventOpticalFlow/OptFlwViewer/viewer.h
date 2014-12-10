#ifndef __VIEWER_H__
#define __VIEWER_H__

#include "VelocityBuffer.h"

#include <cmath>

#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/os/BufferedPort.h>
#include  <yarp/os/Time.h>

#define TNK_TIME 2    // 25 fps


#define XDIM 128
#define YDIM 128

class viewer : public yarp::os::BufferedPort<VelocityBuffer> {
    double mTimeStart;
    double mTimeCurrent;

    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > mPort;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> mBaseImg;


public:
    viewer();
    ~viewer();
    virtual void onRead(VelocityBuffer &);


};

#endif //OPTICALFLOWVIEWER

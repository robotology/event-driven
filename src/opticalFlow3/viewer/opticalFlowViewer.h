#ifndef __OPTICALFLOWVIEWER_H__
#define __OPTICALFLOWVIEWER_H__

#include <cmath>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/ImageDraw.h>

#include "vecBuffer.h"

#define TNK_TIME 0.04    // 25 fps
//#define TNK_TIME 0.02 // 50 fps

#define XDIM 128
#define YDIM 128

class opticalFlowViewer:public yarp::os::BufferedPort<vecBuffer>
{
public:
    opticalFlowViewer();
    ~opticalFlowViewer();
    virtual void onRead(vecBuffer &);

private:
    double mTimeStart;
    double mTimeCurrent;

    double mMapVx[128][128];
    double mMapVy[128][128];

    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > mPort;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> mBaseImg;
};

#endif //OPTICALFLOWVIEWER

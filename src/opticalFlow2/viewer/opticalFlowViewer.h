#ifndef __OPTICALFLOWVIEWER_H__
#define __OPTICALFLOWVIEWER_H__

#include <cmath>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/ImageDraw.h>

#include "vecBuffer.h"

#define TNK_TIME 0.04    // 25 fps
//#define TNK_TIME 0.01666 // 60 fps
//#define TNK_TIME 0.005   // 200 fps

#define HEIGHT 128
#define WIDTH  128

class opticalFlowViewer:public yarp::os::BufferedPort<vecBuffer>
{
public:
    opticalFlowViewer();
    ~opticalFlowViewer();
    virtual void onRead(vecBuffer &);

private:
    double mTimeStart;
    double mTimeCurrent;

    yarp::sig::Matrix mVx;
    yarp::sig::Matrix mVy;

    int indQuiv;

    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > mPort;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> mBaseImg;
    yarp::sig::PixelMono16 mBlack;
};

#endif //OPTICALFLOWVIEWER

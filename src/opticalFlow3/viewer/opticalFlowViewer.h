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
    opticalFlowViewer(double gain);
    ~opticalFlowViewer();
    virtual void onRead(vecBuffer &);

private:
    double mGain;
    double mTimeStart;
    double mTimeCurrent;

    double mMapVx[128][128];
    double mMapVy[128][128];

    yarp::sig::PixelRgb mPalette[256][256];

    //yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono> > mPort;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> mBaseImg;
    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > mPort;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> mBaseImg;
};

#endif //OPTICALFLOWVIEWER

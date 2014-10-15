#ifndef OPTICALFLOWVIEWER
#define OPTICALFLOWVIEWER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cmath>

#include <yarp/os/Network.h>
//#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include "vecBuffer.h"

#define TNK_TIME 40000000	//25fps
//#define TNK_TIME 16700000 //60fps
//#define TNK_TIME 5000000	//200fps
#define HEIGHT 128
#define WIDTH  128
//#define _DEBUG

class opticalFlowViewer:public yarp::os::BufferedPort<vecBuffer>
{
public:
    opticalFlowViewer();
    ~opticalFlowViewer();
    virtual void onRead(vecBuffer &);
    void send_frame(yarp::sig::ImageOf<yarp::sig::PixelMono16>);

private:
    yarp::sig::Matrix findNz(yarp::sig::Matrix&, double);


    timespec start;
    timespec current;

    yarp::sig::Vector x;
    yarp::sig::Vector y;

    yarp::sig::Matrix vx;
    yarp::sig::Matrix vy;

    std::stringstream ssBuffer;
    int indQuiv;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > port;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> base_img;
    yarp::sig::PixelMono16 black;
};

#endif //OPTICALFLOWVIEWER

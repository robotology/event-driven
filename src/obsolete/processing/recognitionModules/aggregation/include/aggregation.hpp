#ifndef AGGREGATION_HPP
#define AGGREGATION_HPP

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_statistics.h>

#include "iCub/emorph/objDistBuffer.h"

#define _DEBUG

class aggregation : public yarp::os::BufferedPort<emorph::reco::objDistBuffer>
{
public:
    aggregation();
    aggregation(std::string&, yarp::os::BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > *);
    ~aggregation();

    void onRead(emorph::reco::objDistBuffer&);
    void aggregate();

private:
    double sum(yarp::sig::Vector&);
    int loadPictures();
    void sendImg(int&);
#ifdef _DEBUG
    void printVector(yarp::sig::Vector*);
#endif
    bool rcvLeft;
    bool rcvRight;

    unsigned int szLeft;
    unsigned int szRight;

    yarp::sig::Vector posLeftEye;
    yarp::sig::Vector negLeftEye;
    yarp::sig::Vector posRightEye;
    yarp::sig::Vector negRightEye;

    std::string imgFile;
    yarp::os::BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > *port;

    cv::Mat **knowledgeImg;
};

#endif //AGGREGATION_HPP

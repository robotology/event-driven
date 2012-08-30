#ifndef RECOGNITION_HPP
#define RECOGNITION_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_statistics.h>

#include "iCub/emorph/eventHistBuffer.h"
#include "iCub/emorph/objDistBuffer.h"

class recognition:public yarp::os::BufferedPort<emorph::ehist::eventHistBuffer>
{
public:
    recognition();
    recognition(std::string, unsigned int, unsigned int, std::string);
    ~recognition();

    virtual void onRead(emorph::ehist::eventHistBuffer&);

    void set_hist(unsigned int*, unsigned int, unsigned int*, unsigned int);
    void setOutPort(yarp::os::BufferedPort< emorph::reco::objDistBuffer > *_port ){outPort=_port;};
private:
    void init();
    void loadKnowledgeBase();
    int loadFile(std::string);
    int loadFileFromPol(std::string&, unsigned int**, unsigned int**, unsigned int&, unsigned int&, unsigned int&);

    int createDenseHistogram(unsigned int*, unsigned int, yarp::sig::Matrix*);
    void max(yarp::sig::Matrix*, double&);
    void min(yarp::sig::Matrix*, double&);
    void mean(yarp::sig::Vector*, double&);
    int cov(yarp::sig::Vector*, yarp::sig::Vector*, double&);
    int mahalanobisDist(yarp::sig::Vector*, yarp::sig::Vector*, double&);    
    int euclideanDist(yarp::sig::Vector*, yarp::sig::Vector*, double&);    
    void histDist(yarp::sig::Matrix*, yarp::sig::Matrix*, double&);
    void printVector(yarp::sig::Vector*);
    void countElemInHist(yarp::sig::Matrix*, unsigned int&);   

    void compare(); 

    //yarp::sig::ImageOf<yarp::sig::PixelMono16> resImg;
    yarp::os::BufferedPort< emorph::reco::objDistBuffer > * outPort;

    std::string polarity;
    std::string knowledgeFileList;

    unsigned int saccd; //duration of a saccade.
    unsigned int htwin; //Histogram time window precision.
    unsigned int nBin;  //number of bin computed using saccd and htwin

    unsigned int nposk;
    unsigned int sznposh;
    unsigned int szptrposh;
    unsigned int *nposh;
    unsigned int *ptr_posh;
    
    yarp::sig::Matrix **ptr_posdh;

    unsigned int nnegk;
    unsigned int sznnegh;
    unsigned int szptrnegh;
    unsigned int *nnegh;
    unsigned int *ptr_negh;

    yarp::sig::Matrix **ptr_negdh;

    unsigned int knowledgeSize;

    unsigned int nunkposdh;
    yarp::sig::Matrix *ptr_unkposdh;
    unsigned int nunknegdh;
    yarp::sig::Matrix *ptr_unknegdh;
    
    bool eyeSel;
};

#endif //RECOGNITION_HPP

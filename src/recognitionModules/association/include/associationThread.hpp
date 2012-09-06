#ifndef ASSOCIATIONTHREAD_HPP
#define ASSOCIATIONTHREAD_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>

#include <yarp/os/Network.h>
//#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "iCub/emorph/eventBuffer.h"
#include "iCub/emorph/eventHistBuffer.h"
#include "iCub/emorph/eventUnmask.h"
#include "iCub/emorph/eventUnmaskDVS128.h"
#include "iCub/emorph/eventUnmaskICUB.h"

#include "iCub/emorph/eventSpatiotemporalVolumeStack.h"

#define RATETH 1

//class associationThread:public yarp::os::RateThread
class associationThread:public yarp::os::Thread
{
public:
    associationThread(); 
    associationThread(std::string, unsigned int, std::string, std::string, unsigned int, unsigned int, double, double, unsigned int, bool, yarp::os::BufferedPort<emorph::ehist::eventHistBuffer>*);
    ~associationThread();

    unsigned int get_sizeh(std::string);
    unsigned int* get_ptrh(std::string);

    //void setBuffer(char*, unsigned int);
    void setBuffer(emorph::ebuffer::eventBuffer&);

    void run();
private:
    int loadFeatures(std::string, double**, unsigned int&);
    int associate();
    int initialize(unsigned int&, unsigned int&, int&, unsigned int&);
    /**
    *   void feed(unsigned int&, unsigned int&, int&, unsigned int&)
    *   Create and fill the spatiotemporal volumes under the spatiotemporal volume stack
    *   @param[in]  _x, x address of the current event
    *   @param[in]  _y, y address of the current event
    *   @param[in]  _p, polarity of the current event
    *   @param[in]  _t, timestamp of the current event
    *   @return     N/A
    */
    void feed(unsigned int&, unsigned int&, int&, unsigned int&);
    /**
    *   void pca()
    *   
    */
    void pca();
    int computePCA(double*, unsigned int&, int);
    void computeDistances();
    int computeDistance(double*, unsigned int&, double*, unsigned int&, unsigned int*, unsigned int&);
    
    double mean(double*, unsigned int);
    void save();
    void splitFilename(const std::string&, std::string&);
    bool verifIfExist(std::string, std::string);
    void sendhist();
    void createPreview(unsigned int, unsigned int, int);

    std::string source;
    std::string featuresFile;
    std::string featuresFolder;

    int lastUpperBound;

    unsigned int refts;
    unsigned int szSpace;
    unsigned int szTemp;
    unsigned int dim;
    double eigenvalSim;
    double eigenvecSim;

    bool learn;

    unsigned int npose;
    double *ptr_pose;
    
    unsigned int nnege;
    double *ptr_nege;

    unsigned int nposk;
    double *ptr_posk;

    unsigned int nnegk;
    double *ptr_negk;

    unsigned int nposh;
    unsigned int *ptr_posh;

    unsigned int nnegh;
    unsigned int *ptr_negh;

    emorph::evolume::eventSpatiotemporalVolumeStack stvStack;
//    jaerParser jaer; 
    emorph::eunmask::eventUnmask *target;

    yarp::os::BufferedPort<emorph::ehist::eventHistBuffer> *outputPort;

    bool initialized;
    unsigned int eyeSel;

    cv::Mat imgPreview;
};

#endif //ASSOCIATIONTHREAD_HPP

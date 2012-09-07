#ifndef EXTRACTIONTHREAD_HPP
#define EXTRACTIONTHREAD_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdio>

#include <yarp/os/Network.h>
//#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "iCub/emorph/eventUnmask.h"
#include "iCub/emorph/eventUnmaskDVS128.h"
#include "iCub/emorph/eventUnmaskICUB.h"
#include "iCub/emorph/eventUnmaskICUBcircBuf.h"
#include "iCub/emorph/eventSpatiotemporalVolumeStack.h"

#define THRATE 1

//typedef unsigned int unsigned int;
//class extractionThread:public yarp::os::RateThread
class extractionThread:public yarp::os::Thread
{
public:
    extractionThread();
    extractionThread(std::string, unsigned int, std::string, unsigned int, unsigned int, double, double, unsigned int);
    ~extractionThread();
    
    /**
    *   double* get()
    *   @return pointer on the set of features 
    *           ____________________
    *           | nb of features    | 0
    *           | nb of princ comp  | 1
    *           | eigenval_11       | .
    *           |   ...             | .
    *           | eigenval_1n       | .
    *           | eigenvec_111      | 
    *           |   ...             |
    *           | eigenvec_11n      |
    *           |   ...             |
    *           | eigenvec_1n1      |
    *           |   ...             |
    *           | eigenvec_1nn      |
    *           |   ...             |
    *           | eigenval_n1       |
    *           |   ...             |
    */
    //inline double* get(){return ptr_features;};
    virtual void run();
    void setBuffer(char*, unsigned int); 
private:
    /**
    *   int extract()
    *   Extraction of the features of the given record. Those features are stored in memory pointed by ptr_features.
    *   @return 0,  if extractionThread terminates without error, -1 otherwise
    */
    int extract();
    /**
    *   void initialize()
    *   Look for the event of synchronization which indicates the beginning of the saccade.
    */
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
    void createKernels();
    int bruteForce(double*, unsigned int&, double*, unsigned int&);
    
    double mean(double*, unsigned int);
    void save();
    void splitFilename(const std::string&, std::string&);

    unsigned int szSpace;
    unsigned int szTemp;
    unsigned int dim;

    double eigenvalSim;
    double eigenvecSim;

    unsigned int npose;
    double *ptr_pose;
    
    unsigned int nnege;
    double *ptr_nege;

    unsigned int nposk;
    double *ptr_posk;

    unsigned int nnegk;
    double *ptr_negk;

    emorph::evolume::eventSpatiotemporalVolumeStack stvStack;
    emorph::eunmask::eventUnmask *source;

    bool initialized;
    unsigned int eyeSel;
};

#endif //EXTRACTIONTHREAD_HPP

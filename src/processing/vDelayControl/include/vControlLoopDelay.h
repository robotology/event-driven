#ifndef __VCONTROLLOOPDELAYPF__
#define __VCONTROLLOOPDELAYPF__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/eventdriven/all.h>
#include "vParticle.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/
class roiq
{
public:

    vQueue q;
    unsigned int n;
    yarp::sig::Vector roi;
    bool use_TW;

    roiq();
    void setSize(unsigned int value);
    void setROI(int xl, int xh, int yl, int yh);
    int add(event<AE> &v);

};

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

class delayControl : public yarp::os::Thread
{
private:

    //data structures and ports
    queueAllocator inputPort;
    roiq qROI;
    vParticlefilter vpf;
    yarp::os::BufferedPort<vBottle> outputPort;

    //variables
    resolution res;
    double avgx, avgy, avgr;
    double gain;
    double minEvents;
    int detectionThreshold;

    //diagnostics
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;
    yarp::os::BufferedPort< yarp::sig::ImageOf< yarp::sig::PixelBgr> > debugPort;


public:

    delayControl() {}

    bool open(std::string name, unsigned int qlimit = 0);
    void initFilter(int width, int height, int nparticles,
                    int bins, bool adaptive, int nthreads,
                    double minlikelihood, double inlierThresh, double randoms);
    void setFilterInitialState(int x, int y, int r);
    void initDelayControl(double gain, int maxtoproc, int positiveThreshold, int mindelay);

    //bool threadInit();
    void onStop();
    void run();
    //void threadRelease();


};





#endif

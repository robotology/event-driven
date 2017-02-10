#ifndef __VFIXEDRATEPF__
#define __VFIXEDRATEPF__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include "vParticle.h"
#include <queue>

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEREADER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticleReader : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //debug stuff
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > debugOut;
    yarp::os::Bottle weights;
    yarp::os::Stamp pstamp;

    //event reps
    ev::temporalSurface surfaceLeft;
    ev::vtsHelper unwrap;

    //particle storage and variables
    std::priority_queue<vParticle> sortedlist;
    std::vector<vParticle> indexedlist;
    vParticle pmax;
    double pwsum;
    double pwsumsq;
    double avgx;
    double avgy;
    double avgr;
    double avgtw;

    //parameters
    ev::resolution res;
    bool strict;
    int nparticles;
    int rate;
    double nRandomise;
    bool adaptive;

public:

    vParticleReader();
    void initialise(unsigned int width , unsigned int height, unsigned int nParticles, unsigned int rate, double nRands, bool adaptive);

    bool    open(const std::string &name, bool strictness = false);
    void    onRead(ev::vBottle &inBot);
    void    close();
    void    interrupt();

};

#endif

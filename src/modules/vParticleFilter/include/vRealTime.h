#ifndef __VREALTIMEPF__
#define __VREALTIMEPF__

#include "vParticle.h"

/*////////////////////////////////////////////////////////////////////////////*/
//vTemporalHandler
/*////////////////////////////////////////////////////////////////////////////*/
class vSurfaceHandler : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //parameters
    ev::resolution res;
    bool strict;

    //data
    ev::vQueue queriedQ;
    ev::temporalSurface surfaceLeft;
    ev::temporalSurface surfaceRight;
    yarp::os::Stamp pstamp;
    ev::vtsHelper unwrap;
    unsigned long int tnow;
    unsigned int condTime;
    unsigned int tw;
    bool eventsQueried;
    yarp::os::Semaphore waitsignal;
    yarp::os::Mutex mutexsignal;
    double ptime;
    double eventrate;
    double bottletime;

public:

    vSurfaceHandler(unsigned int width = 128, unsigned int height = 128);

    void resize(unsigned int width, unsigned int height);
    ev::vQueue queryEvents(unsigned long int conditionTime, unsigned int temporalWindow);
    ev::vQueue queryEventList(std::vector<vParticle> &ps);
    void queryEvents(ev::vQueue &fillq, unsigned int temporalwindow);
    double geteventrate() { return eventrate; }

    bool    open(const std::string &name, bool strictness = false);
    void    onRead(ev::vBottle &inBot);
    void    close();
    void    interrupt();

};

/*////////////////////////////////////////////////////////////////////////////*/
// vParticleObserver
/*////////////////////////////////////////////////////////////////////////////*/
class vPartObsThread : public yarp::os::Thread
{
private:

    int pStart;
    int pEnd;

    double normval;

    std::vector<vParticle> *particles;
    std::vector<int> *deltats;
    ev::vQueue *stw;

public:

    vPartObsThread(int pStart, int pEnd);
    void setDataSources(std::vector<vParticle> *particles,
                        std::vector<int> *deltats, ev::vQueue *stw);
    double getNormVal() { return normval; }
    bool threadInit() { return true; }
    void run();
    void threadRelease() {}
};

/*////////////////////////////////////////////////////////////////////////////*/
//particleProcessor
/*////////////////////////////////////////////////////////////////////////////*/
class particleProcessor : public yarp::os::Thread
{
private:

    vSurfaceHandler eventhandler;
    std::vector<vPartObsThread *> computeThreads;
    int nThreads;
    ev::resolution res;
    double ptime, ptime2;

    yarp::os::BufferedPort<ev::vBottle> vBottleOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > debugOut;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    ev::vtsHelper unwrap;
    std::vector<vParticle> indexedlist;
    double avgx;
    double avgy;
    double avgr;
    double avgtw;
    double maxtw;
    double maxlikelihood;
    double pwsumsq;
    double particleVariance;
    int rate;
    std::string name;
    bool strict;

    int nparticles;
    double nRandomise;
    bool adaptive;
    double pVariance;

    double obsThresh;
    double obsInlier;
    double obsOutlier;

    bool inbounds(vParticle &p);

public:

    void setFilterParameters(int nParticles, double nRandomise, bool adaptive, double variance) {
        nparticles = nParticles; this->nRandomise = 1.0 + nRandomise; this->adaptive = adaptive; this->pVariance = variance;}
    void setObservationParameters(double minLikelihood, double inlierPar, double outlierPar) {
        obsThresh = minLikelihood; obsInlier = inlierPar; obsOutlier = outlierPar; }

    particleProcessor(unsigned int height, unsigned int width, std::string name, bool strict);
    bool threadInit();
    void run();
    void threadRelease();
};

#endif

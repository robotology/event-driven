#ifndef __VREALTIMEPF__
#define __VREALTIMEPF__

#include "vParticle.h"
#include <iCub/eventdriven/vSurfaceHandlerTh.h>

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

    surfaceThread eventhandler2;
    preComputedBins pcb;
    std::vector<vPartObsThread *> computeThreads;
    int nThreads;
    ev::resolution res;
    double ptime, ptime2;
    double pytime;
    int rbound_min;
    int rbound_max;

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
    int camera;
    bool useroi;

    double seedx;
    double seedy;
    double seedr;

    int nparticles;
    double nRandomise;
    bool adaptive;
    double pVariance;

    double obsThresh;
    double obsInlier;
    double obsOutlier;

    bool inbounds(vParticle &p);

public:

    void setComputeOptions(int camera, int threads, bool useROI) {
        this->camera = camera; nThreads = threads; useroi = useROI; }
    void setFilterParameters(int nParticles, double nRandomise, bool adaptive, double variance) {
        nparticles = nParticles; this->nRandomise = 1.0 + nRandomise; this->adaptive = adaptive; this->pVariance = variance; }
    void setObservationParameters(double minLikelihood, double inlierPar, double outlierPar) {
        obsThresh = minLikelihood; obsInlier = inlierPar; obsOutlier = outlierPar; }
    void setSeed(double x, double y, double r) {
        seedx = x; seedy = y; seedr = r;
    }

    particleProcessor(unsigned int height, unsigned int width, std::string name, bool strict);
    bool threadInit();
    void run();
    void threadRelease();
};

#endif

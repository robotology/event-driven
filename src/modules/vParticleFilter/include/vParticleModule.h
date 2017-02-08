/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/// \defgroup Modules Modules
/// \defgroup vParticleFilter vParticleFilter
/// \ingroup Modules
/// \brief tracks targets using a particle filter

#ifndef __V_PARTICLEMODULE__
#define __V_PARTICLEMODULE__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>
#include <queue>

#include "vParticleFilter.h"

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

public:

    vParticleReader(unsigned int width = 128, unsigned int height = 128);

    bool    open(const std::string &name, bool strictness = false);
    void    onRead(ev::vBottle &inBot);
    void    close();
    void    interrupt();

};

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

    double obsThresh;
    double obsInlier;
    double obsOutlier;

    bool inbounds(vParticle &p);

public:

    void setFilterParameters(int nParticles, double nRandomise, bool adaptive) {
        nparticles = nParticles; this->nRandomise = 1.0 + nRandomise; this->adaptive = adaptive; }
    void setObservationParameters(double minLikelihood, double inlierPar, double outlierPar) {
        obsThresh = minLikelihood; obsInlier = inlierPar; obsOutlier = outlierPar; }

    particleProcessor(unsigned int height, unsigned int width, std::string name, bool strict);
    bool threadInit();
    void run();
    void threadRelease();
};




/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class vParticleModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vParticleReader *particleCallback;
    particleProcessor *particleThread;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif

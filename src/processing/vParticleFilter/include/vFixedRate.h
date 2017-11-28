/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
    yarp::os::BufferedPort<yarp::os::Bottle> resultOut;
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;
    yarp::os::Bottle weights;
    yarp::os::Stamp pstamp;

    //event reps
    ev::temporalSurface surfaceLeft;
    ev::vtsHelper unwrap;
    preComputedBins pcb;

    //particle storage and variables
    //std::priority_queue<vParticle> sortedlist;
    std::vector<vParticle> indexedlist;
    vParticle pmax;
    double pwsum;
    double pwsumsq;
    double avgx;
    double avgy;
    double avgr;
    double avgtw;

    double seedx;
    double seedy;
    double seedr;

    int rbound_min;
    int rbound_max;

    //parameters
    ev::resolution res;
    bool strict;
    int nparticles;
    int rate;
    double nRandomise;
    bool adaptive;
    int camera;
    bool useroi;

    double obsThresh;
    double obsInlier;
    double obsOutlier;

    bool inbounds(vParticle &p);

public:

    vParticleReader();
    void initialise(unsigned int width , unsigned int height, unsigned int nParticles, unsigned int rate, double nRands, bool adaptive, double pVariance, int camera, bool useROI);
    void setObservationParameters(double minLikelihood, double inlierPar, double outlierPar) {
        obsThresh = minLikelihood; obsInlier = inlierPar; obsOutlier = outlierPar; }

    void setSeed(int x, int y, int r)
    {
        seedx = x; seedy = y; seedr = r;
    }

    bool    open(const std::string &name, bool strictness = false);
    void    onRead(ev::vBottle &inBot);
    void    close();
    void    interrupt();

};

#endif

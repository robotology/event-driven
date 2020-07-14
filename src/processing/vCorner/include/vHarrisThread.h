/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: valentina.vasco@iit.it
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

/// \defgroup Modules Modules
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VHARRISTHREAD__
#define __VHARRISTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <event-driven/all.h>
#include <event-driven/deprecated.h>
#include <filters.h>
#include <fstream>
#include <math.h>

class vComputeHarrisThread : public yarp::os::Thread
{
private:

    int sobelsize;
    int windowRad;
    double sigma;
    double thresh;
    unsigned int qlen;
    ev::vQueue patch;
    filters convolution;
    ev::collectorPort *outthread;
    yarp::os::Stamp *ystamp_p;
    ev::temporalSurface *cSurf_p;

    yarp::os::Semaphore *semaphore;

    std::mutex *mutex_writer;
    std::mutex *mutex_reader;
    int *readcount;

    ev::event<ev::AddressEvent> aep;

    bool suspended;
    bool detectcorner(int x, int y);

public:

    vComputeHarrisThread(int sobelsize, int windowRad, double sigma, double thresh, unsigned int qlen, ev::collectorPort *outthread,
                    std::mutex *mutex_writer, std::mutex *mutex_reader, int *readcount);
    void assignTask(ev::event<ev::AddressEvent> ae, ev::temporalSurface *cSurf, yarp::os::Stamp *ystamp);
    void suspend();
    void wakeup();
    bool available();
    ev::event<ev::LabelledAE> getResponse();
    bool threadInit() { return true; }
    void run();
    void threadRelease() {}
    void onStop();
};

class vHarrisThread : public yarp::os::Thread
{
private:

    //thread for queues of events
    ev::queueAllocator inputPort;

    //data structures
    ev::temporalSurface *surfaceleft;
    ev::temporalSurface *surfaceright;

    //port for debugging
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //list of thread for processing
    std::vector<vComputeHarrisThread *> computeThreads;
    std::mutex semaphore;

    //to protect the writing
    std::mutex *mutex_writer;
    std::mutex *mutex_reader;
    int readcount;

    //thread for the output
    ev::collectorPort outthread;

    //synchronising value
    yarp::os::Stamp yarpstamp;

    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    int qlen;
    int temporalsize;
    int windowRad;
    int sobelsize;
    double sigma;
    double thresh;
    int nthreads;
    double gain;

    bool detectcorner(ev::vQueue patch, int x, int y);

public:

    vHarrisThread(unsigned int height, unsigned int width, std::string name, bool strict, int qlen,
                  double temporalsize, int windowRad, int sobelsize, double sigma, double thresh,
                  int nthreads, double gain);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();

};


#endif
//empty line to make gcc happy

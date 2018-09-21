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

#include "vHarrisThread.h"

using namespace ev;

vHarrisThread::vHarrisThread(unsigned int height, unsigned int width, std::string name, bool strict, int qlen,
                             double temporalsize, int windowRad, int sobelsize, double sigma, double thresh,
                             int nthreads, double gain)
{
    std::cout << "Using HARRIS implementation..." << std::endl;

    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->qlen = qlen;
    this->temporalsize = temporalsize / ev::vtsHelper::tsscaler;
    this->windowRad = windowRad;
    this->sobelsize = sobelsize;
    this->sigma = sigma;
    this->thresh = thresh;
    this->nthreads = nthreads;
    this->gain = gain;

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    //data structure
    surfaceleft  = new temporalSurface(width, height, this->temporalsize);
    surfaceright = new temporalSurface(width, height, this->temporalsize);

    //mutex to protect the writing
    mutex_writer = new yarp::os::Mutex();
    mutex_reader = new yarp::os::Mutex();
    readcount = 0;

    //start the threads
    for(int i = 0; i < nthreads; i ++) {
        computeThreads.push_back(new vComputeHarrisThread(sobelsize, windowRad, sigma, thresh, qlen,
                                                          &outthread, mutex_writer, mutex_reader, &readcount));
        computeThreads[i]->start();
    }
    std::cout << "...with " << nthreads << " threads for computation " << std::endl;

}

bool vHarrisThread::threadInit()
{

    if(!inputPort.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

    std::string debugPortName = "/" + name + "/debug:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vHarrisThread::onStop()
{
    debugPort.close();
    inputPort.close();
    inputPort.releaseDataLock();

    for(int i = 0; i < nthreads; i++) {
        computeThreads[i]->stop();
        delete computeThreads[i];
    }

    delete surfaceleft;
    delete surfaceright;

    delete mutex_writer;
    delete mutex_reader;

}

void vHarrisThread::run()
{
    int maxV = 10000;
    //int minAcceptableDelay =  51200;
    while(!isStopping()) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.read(yarpstamp);
        }
        if(isStopping()) break;

        //skip events based on delay in the queue
        unsigned int delay_n = inputPort.queryDelayN();
//        double increment = gain * (delay_n - q->size()) / minAcceptableDelay;
        double increment = ((double)delay_n)/maxV;
        if(increment < 1.0)
            increment = 1;

        double currCount;
        unsigned int currSkip,lastSkip = 0;
        currCount = 0.0;
        currSkip = (unsigned int)currCount;

        int countProcessed = 0;
        bool firstChecked = false;
        ev::vQueue::iterator qi;
        while(currSkip < q->size())  {

            if(!firstChecked) {
                qi = q->begin();
                firstChecked = true;
            } else {
                qi = qi + ((int)currSkip - lastSkip);
                lastSkip = currSkip;
            }

            lastSkip = currSkip;
            currCount += increment;
            currSkip = (unsigned int)currCount;

            //get current event and add it to the surface
            auto ae = ev::is_event<ev::AE>(*qi);
            ev::temporalSurface *cSurf;
            if(ae->getChannel() == 0)
                cSurf = surfaceleft;
            else
                cSurf = surfaceright;

            mutex_writer->lock();
            cSurf->fastAddEvent(*qi);
            mutex_writer->unlock();

            int k = 0;
            while(true) {

                //assign a task to a thread that is not managing a task
                if(computeThreads[k]->available()) {
                    computeThreads[k]->assignTask(ae, cSurf, &yarpstamp);
                    countProcessed++;
                    break;
                }
                if(++k == nthreads)
                    k = 0;
            }
        }

        static double prevtime = yarp::os::Time::now();
        if(debugPort.getOutputCount()) {

            double time = yarp::os::Time::now();

            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(inputPort.queryRate());
            scorebottleout.addDouble(countProcessed/(time-prevtime));
            scorebottleout.addDouble((double)countProcessed/q->size());
            scorebottleout.addDouble(delay_n);
            scorebottleout.addDouble(inputPort.queryDelayT());
            debugPort.write();

            prevtime = time;
        }

        //inputPort.scrapQ();

    }

}

/*////////////////////////////////////////////////////////////////////////////*/
//threaded computation
/*////////////////////////////////////////////////////////////////////////////*/
vComputeHarrisThread::vComputeHarrisThread(int sobelsize, int windowRad, double sigma, double thresh, unsigned int qlen, collectorPort *outthread,
                               yarp::os::Mutex *mutex_writer, yarp::os::Mutex *mutex_reader, int *readcount)
{
    this->sobelsize = sobelsize;
    this->windowRad = windowRad;
    this->sigma = sigma;
    this->thresh = thresh;
    this->qlen = qlen;
    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);
    this->outthread = outthread;

    semaphore = new yarp::os::Semaphore(0);
    suspended = true;

    this->mutex_writer = mutex_writer;
    this->mutex_reader = mutex_reader;
    this->readcount = readcount;

}

void vComputeHarrisThread::assignTask(ev::event<AddressEvent> ae, ev::temporalSurface *cSurf, yarp::os::Stamp *ystamp)
{
    cSurf_p = cSurf;
    ystamp_p = ystamp;
    aep = ae;
    wakeup();
}

void vComputeHarrisThread::suspend()
{
    suspended = true;
}

void vComputeHarrisThread::wakeup()
{
    suspended = false;
    semaphore->post();
}

bool vComputeHarrisThread::available()
{
    return suspended;
}

void vComputeHarrisThread::run()
{
    while(!isStopping()) {

        //if no task is assigned, wait
        if(suspended) {
            semaphore->wait();
        }
        else {

            mutex_reader->lock();
            (*readcount)++;
            if(*readcount == 1)
                mutex_writer->lock();
            mutex_reader->unlock();

            //get patch from the surface
            patch = cSurf_p->getSurf_Clim(qlen, aep->x, aep->y, windowRad);

            mutex_reader->lock();
            (*readcount)--;
            if(*readcount == 0)
                mutex_writer->unlock();
            mutex_reader->unlock();

            //detect corner and send to output
            if(detectcorner(aep->x, aep->y)) {
                auto ce = make_event<LabelledAE>(aep);
                ce->ID = 1;
                outthread->pushevent(ce, *ystamp_p);
            }
            suspend();

        }

    }
}

void vComputeHarrisThread::onStop()
{
    wakeup();
}

bool vComputeHarrisThread::detectcorner(int x, int y)
{

    if(patch.size() == 0) return false;

    //set the final response to be centred on the current event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < patch.size(); i++)
    {
        //events the patch
        auto vi = is_event<AE>(patch[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();
    double score = convolution.getScore();

    //reset responses
    convolution.reset();

    //if score > thresh tag ae as ce
    return score > thresh;

}

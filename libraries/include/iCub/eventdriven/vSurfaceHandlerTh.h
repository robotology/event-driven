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

#ifndef __VSURFACEHANDLER__
#define __VSURFACEHANDLER__

#include <yarp/os/all.h>
#include <iCub/eventdriven/vBottle.h>
#include <iCub/eventdriven/vCodec.h>
#include <iCub/eventdriven/vWindow_basic.h>
#include <iCub/eventdriven/vWindow_adv.h>
#include <iCub/eventdriven/vFilters.h>
#include <deque>
#include <string>
#include <map>

namespace ev {

/// \brief an asynchronous reading port that accepts vBottles and decodes them
class queueAllocator : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    std::deque<ev::vQueue *> qq;
    std::deque<yarp::os::Stamp> sq;
    yarp::os::Mutex m;
    yarp::os::Mutex dataready;

    unsigned int qlimit;
    unsigned int delay_nv;
    long unsigned int delay_t;
    double event_rate;

public:

    /// \brief constructor
    queueAllocator()
    {
        qlimit = 0;
        delay_nv = 0;
        delay_t = 0;
        event_rate = 0;

        dataready.lock();

        useCallback();
        setStrict();
    }

    /// \brief desctructor
    ~queueAllocator()
    {
        m.lock();
        for(std::deque<ev::vQueue *>::iterator i = qq.begin(); i != qq.end(); i++)
            delete *i;
        qq.clear();
        m.unlock();
    }

    /// \brief the callback decodes the incoming vBottle and adds it to the
    /// list of received vBottles. The yarp, and event timestamps are updated.
    void onRead(ev::vBottle &inputbottle)
    {
        //make a new vQueue
        m.lock();

        if(qlimit && qq.size() >= qlimit) {
            m.unlock();
            return;
        }
        qq.push_back(new vQueue);
        yarp::os::Stamp yarpstamp;
        getEnvelope(yarpstamp);
        sq.push_back(yarpstamp);

        m.unlock();


        //and decode the data
        inputbottle.addtoendof<ev::AddressEvent>(*(qq.back()));

        //update the meta data
        m.lock();
        delay_nv += qq.back()->size();
        int dt = qq.back()->back()->stamp - qq.back()->front()->stamp;
        if(dt < 0) dt += vtsHelper::max_stamp;
        delay_t += dt;
        if(dt)
            event_rate = qq.back()->size() / (double)dt;
        m.unlock();

        //if getNextQ is blocking - let it get the new data
        dataready.unlock();
    }

    /// \brief ask for a pointer to the next vQueue. Blocks if no data is ready.
    ev::vQueue* getNextQ(yarp::os::Stamp &yarpstamp)
    {
        dataready.lock();
        if(qq.size()) {
            yarpstamp = sq.front();
            return qq.front();
        }  else {
            return 0;
        }

    }

    /// \brief remove the most recently read vQueue from the list and deallocate
    /// the memory
    void scrapQ()
    {
        m.lock();

        delay_nv -= qq.front()->size();
        int dt = qq.front()->back()->stamp - qq.front()->front()->stamp;
        if(dt < 0) dt += vtsHelper::max_stamp;
        delay_t -= dt;

        delete qq.front();
        qq.pop_front();
        sq.pop_front();
        m.unlock();
    }

    /// \brief set the maximum number of qs that can be stored in the buffer.
    /// A value of 0 keeps all qs.
    void setQLimit(unsigned int number_of_qs)
    {
        qlimit = number_of_qs;
    }

    /// \brief unBlocks the blocking call in getNextQ. Useful to ensure a
    /// graceful shutdown. No guarantee the return of getNextQ will be valid.
    void releaseDataLock()
    {
        dataready.unlock();
    }

    /// \brief ask for the number of vQueues currently allocated.
    int queryunprocessed()
    {
        return qq.size();
    }

    /// \brief ask for the number of events in all vQueues.
    unsigned int queryDelayN()
    {
        return delay_nv;
    }

    /// \brief ask for the total time spanned by all vQueues.
    double queryDelayT()
    {
        return delay_t * vtsHelper::tsscaler;
    }

    /// \brief ask for the high precision event rate
    double queryRate()
    {
        return event_rate * vtsHelper::vtsscaler;
    }

};

/// \brief asynchronously read events and push them in a vSurface
class surfaceThread : public yarp::os::Thread
{
private:

    ev::temporalSurface surfaceLeft;
    ev::temporalSurface surfaceRight;

    queueAllocator allocatorCallback;

    yarp::os::Mutex m;
    yarp::os::Stamp yarpstamp;
    unsigned int ctime;

    int vcount;


public:

    surfaceThread()
    {
        vcount = 0;
        ctime = 0;
    }

    void configure(int height, int width)
    {
        surfaceLeft = ev::temporalSurface(width, height);
        surfaceRight = ev::temporalSurface(width, height);
    }

    bool open(std::string portname)
    {
        if(!allocatorCallback.open(portname))
            return false;
        start();
        return true;
    }

    void onStop()
    {
        allocatorCallback.close();
        allocatorCallback.releaseDataLock();
    }

    void run()
    {
        while(true) {

            ev::vQueue *q = 0;
            while(!q && !isStopping()) {
                q = allocatorCallback.getNextQ(yarpstamp);
            }
            if(isStopping()) break;

            for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

                m.lock();

                vcount++;

                ctime = (*qi)->stamp;

                if((*qi)->getChannel() == 0)
                    surfaceLeft.fastAddEvent(*qi);
                else if((*qi)->getChannel() == 1)
                    surfaceRight.fastAddEvent(*qi);
                else
                    std::cout << "Unknown channel" << std::endl;

                m.unlock();

            }

            allocatorCallback.scrapQ();

        }

    }

    yarp::os::Stamp queryROI(ev::vQueue &fillq, int c, unsigned int t, int x, int y, int r)
    {

        //if(!vcount) return false;

        m.lock();
        if(c == 0)
            fillq = surfaceLeft.getSurf_Tlim(t, x, y, r);
        else
            fillq = surfaceRight.getSurf_Tlim(t, x, y, r);
        vcount = 0;
        m.unlock();
        return yarpstamp;
    }

    yarp::os::Stamp queryWindow(ev::vQueue &fillq, int c, unsigned int t)
    {

        m.lock();
        if(c == 0)
            fillq = surfaceLeft.getSurf_Tlim(t);
        else
            fillq = surfaceRight.getSurf_Tlim(t);
        vcount = 0;
        m.unlock();

        return yarpstamp;
    }

    unsigned int queryVTime()
    {
        return ctime;
    }

};

/// \brief asynchronously read events and push them in a historicalSurface
class hSurfThread : public yarp::os::Thread
{
private:

    int maxcpudelay; //maximum delay between v time and cpu time (in v time)

    queueAllocator allocatorCallback;
    historicalSurface surfaceleft;
    historicalSurface surfaceright;
    yarp::os::Mutex m;

    //current stamp to propagate
    yarp::os::Stamp ystamp;
    unsigned int vstamp;

    //synchronising value (add to it when stamps come in, subtract from it
    // when querying events).
    double cputimeL;
    int cpudelayL;
    double cputimeR;
    int cpudelayR;

public:

    hSurfThread()
    {
        vstamp = 0;
        cpudelayL = cpudelayR = 0;
        cputimeL = cputimeR = yarp::os::Time::now();
        maxcpudelay = 0.05 * vtsHelper::vtsscaler;
    }

    void configure(int height, int width, double maxcpudelay)
    {
        this->maxcpudelay = maxcpudelay * vtsHelper::vtsscaler;
        surfaceleft.initialise(height, width);
        surfaceright.initialise(height, width);
    }

    bool open(std::string portname)
    {
        if(!allocatorCallback.open(portname))
            return false;

        start();
        return true;
    }

    void onStop()
    {
        allocatorCallback.close();
        allocatorCallback.releaseDataLock();
    }


    void run()
    {
        static int maxqs = 4;
        bool allowproc = true;

        while(true) {

            ev::vQueue *q = 0;
            while(!q && !isStopping()) {
                q = allocatorCallback.getNextQ(ystamp);
            }
            if(isStopping()) break;


            int nqs = allocatorCallback.queryunprocessed();

            if(allowproc)
                m.lock();

            if(nqs >= maxqs)
                allowproc = false;
            else if(nqs < maxqs)
                allowproc = true;

            int dt = q->back()->stamp - vstamp;
            if(dt < 0) dt += vtsHelper::max_stamp;
            cpudelayL += dt;
            cpudelayR += dt;
            vstamp = q->back()->stamp;

            for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

                if((*qi)->getChannel() == 0)
                    surfaceleft.addEvent(*qi);
                else if((*qi)->getChannel() == 1)
                    surfaceright.addEvent(*qi);

            }

            if(allowproc)
                m.unlock();

            allocatorCallback.scrapQ();

        }

    }

    vQueue queryROI(int channel, unsigned int querySize, int x, int y, int r)
    {
        vQueue q;

        m.lock();

        double cpunow = yarp::os::Time::now();

        if(channel == 0) {

            cpudelayL -= (cpunow - cputimeL) * vtsHelper::vtsscaler * 1.01;
            cputimeL = cpunow;

            if(cpudelayL < 0) cpudelayL = 0;
            if(cpudelayL > maxcpudelay) {
                yWarning() << "CPU delay hit maximum";
                cpudelayL = maxcpudelay;
            }

            q = surfaceleft.getSurface(cpudelayL, querySize, r, x, y);
        } else {

            cpudelayR -= (cpunow - cputimeR) * vtsHelper::vtsscaler * 1.01;
            cputimeR = cpunow;

            if(cpudelayR < 0) cpudelayR = 0;
            if(cpudelayR > maxcpudelay) {
                yWarning() << "CPU delay hit maximum";
                cpudelayR = maxcpudelay;
            }

            q = surfaceright.getSurface(cpudelayR, querySize, r, x, y);
        }

        m.unlock();

        return q;
    }

    vQueue queryWindow(int channel, unsigned int querySize)
    {
        vQueue q;

        m.lock();

        double cpunow = yarp::os::Time::now();

        if(channel == 0) {

            cpudelayL -= (cpunow - cputimeL) * vtsHelper::vtsscaler * 1.01;
            cputimeL = cpunow;

            if(cpudelayL < 0) cpudelayL = 0;
            if(cpudelayL > maxcpudelay) {
                yWarning() << "CPU delay hit maximum";
                cpudelayL = maxcpudelay;
            }

            q = surfaceleft.getSurface(cpudelayL, querySize);
        }
        else {

            cpudelayR -= (cpunow - cputimeR) * vtsHelper::vtsscaler * 1.01;
            cputimeR = cpunow;

            if(cpudelayR < 0) cpudelayR = 0;
            if(cpudelayR > maxcpudelay) {
                yWarning() << "CPU delay hit maximum";
                cpudelayR = maxcpudelay;
            }

            q = surfaceright.getSurface(cpudelayR, querySize);
        }

        m.unlock();

        return q;
    }

    double queryDelay(int channel = 0)
    {
        if(channel) {
            return cpudelayR * vtsHelper::tsscaler;
        } else {
            return cpudelayL * vtsHelper::tsscaler;
        }
    }

    yarp::os::Stamp queryYstamp()
    {
        return ystamp;
    }

    int queryVstamp(int channel = 0)
    {
        int modvstamp;
        m.lock();
        if(channel) {
            modvstamp = vstamp - cpudelayR;
        } else {
            modvstamp = vstamp - cpudelayL;
        }
        m.unlock();

        if(modvstamp < 0) modvstamp += vtsHelper::max_stamp;
        return modvstamp;

    }

    int queryQDelay()
    {
        return allocatorCallback.queryunprocessed();
    }

};

/// \brief automatically accept events from a port and push them into a
/// vTempWindow
class tWinThread : public yarp::os::Thread
{
private:

    queueAllocator allocatorCallback;
    vTempWindow windowleft;
    vTempWindow windowright;

    yarp::os::Mutex safety;

    int strictUpdatePeriod;
    int currentPeriod;
    yarp::os::Mutex waitforquery;
    yarp::os::Stamp yarpstamp;
    unsigned int ctime;

public:

    tWinThread()
    {
        ctime = 0;
        strictUpdatePeriod = 0;
        currentPeriod = 0;
    }

    bool open(std::string portname, int period = 0)
    {
        strictUpdatePeriod = period;
        if(strictUpdatePeriod) yInfo() << "Forced update every" << period * vtsHelper::tsscaler <<"s, or"<< period << "event timestamps";
        if(!allocatorCallback.open(portname))
            return false;

        return start();
    }

    void onStop()
    {
        allocatorCallback.close();
        allocatorCallback.releaseDataLock();
        waitforquery.unlock();
    }

    void run()
    {
        if(strictUpdatePeriod) {
            safety.lock();
            waitforquery.lock();
        }

        while(!isStopping()) {

            ev::vQueue *q = 0;
            while(!q && !isStopping()) {
                q = allocatorCallback.getNextQ(yarpstamp);
            }
            if(isStopping()) break;

            for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

                if(!strictUpdatePeriod) safety.lock();

                if(strictUpdatePeriod) {
                    int dt = (*qi)->stamp - ctime;
                    if(dt < 0) dt += vtsHelper::max_stamp;
                    currentPeriod += dt;
                    if(currentPeriod > strictUpdatePeriod) {
                        safety.unlock();
                        waitforquery.lock();
                        safety.lock();
                        currentPeriod = 0;
                    }

                }
                ctime = (*qi)->stamp;

                if((*qi)->getChannel() == 0)
                    windowleft.addEvent(*qi);
                else if((*qi)->getChannel() == 1)
                    windowright.addEvent(*qi);

                if(!strictUpdatePeriod) safety.unlock();

            }

            allocatorCallback.scrapQ();

        }
        if(strictUpdatePeriod)
            safety.unlock();
    }

    vQueue queryWindow(int channel)
    {
        vQueue q;

        safety.lock();
        if(channel == 0)
            q = windowleft.getWindow();
        else
            q = windowright.getWindow();
        waitforquery.unlock();
        safety.unlock();
        return q;
    }

    void queryStamps(yarp::os::Stamp &yStamp, int &vStamp)
    {
        yStamp = yarpstamp;
        vStamp = ctime;
    }

};

/// \brief automatically accept multiple event types from different ports
/// (e.g. as in the vFramer)
class syncvstreams
{
private:

    std::map<std::string, ev::tWinThread> iPorts;
    //std::deque<tWinThread> iPorts;
    yarp::os::Stamp yStamp;
    int vStamp;
    int strictUpdatePeriod;
    //std::map<std::string, int> labelMap;

public:

    syncvstreams(void)
    {
        strictUpdatePeriod = 0;
        vStamp = 0;
    }

    bool open(std::string moduleName, std::string eventType)
    {
        //check already have an input of that type
        if(iPorts.count(eventType))
            return true;

        //otherwise open a new port
        if(!iPorts[eventType].open(moduleName + "/" + eventType + ":i", strictUpdatePeriod))
            return false;

        return true;
    }

    vQueue queryWindow(std::string vType, int channel)
    {
        yarp::os::Stamp ys;
        int vs;
        iPorts[vType].queryStamps(ys, vs);

        if(ys.getTime() > yStamp.getTime()) {
            yStamp = ys;
            vStamp = vs;
        }

        return iPorts[vType].queryWindow(channel);
    }

    void close()
    {
        std::map<std::string, ev::tWinThread>::iterator i;
        for(i = iPorts.begin(); i != iPorts.end(); i++)
            i->second.stop();
    }

    yarp::os::Stamp getystamp()
    {
        return yStamp;
    }

    int getvstamp()
    {
        return vStamp;
    }

    void setStrictUpdatePeriod(int period)
    {
        strictUpdatePeriod = period;
    }

};

}

#endif

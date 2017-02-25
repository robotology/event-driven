#ifndef __VSURFACEHANDLER__
#define __VSURFACEHANDLER__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <deque>
#include <string>

class queueAllocator : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    std::deque<ev::vQueue *> qq;
    yarp::os::Mutex m;
    yarp::os::Mutex dataready;

public:

    queueAllocator()
    {
        useCallback();
        setStrict();
    }
    ~queueAllocator()
    {
        for(std::deque<ev::vQueue *>::iterator i = qq.begin(); i != qq.end(); i++)
            delete *i;
    }

    void configure() {}

    void onRead(ev::vBottle &inputbottle)
    {

        //make a new vQueue
        m.lock();
        qq.push_back(new vQueue);
        m.unlock();
        //and decode the data
        inputbottle.addtoendof<ev::AddressEvent>(*(qq.back()));
        dataready.unlock();
    }

    ev::vQueue* getNextQ()
    {
        dataready.lock();
        if(qq.size() > 1) {
            return qq.front();
        }
        else
            return 0;

    }

    void scrapQ()
    {
        m.lock();
        delete qq.front();
        qq.pop_front();
        m.unlock();
    }

};

class surfaceThread : public yarp::os::Thread
{
private:

    ev::temporalSurface surfaceLeft;
    ev::temporalSurface surfaceRight;

    queueAllocator allocatorCallback;

    yarp::os::Mutex m;

    int vcount;


public:

    surfaceThread()
    {
        vcount = 0;
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


    void run()
    {
        while(!isStopping()) {

            ev::vQueue *q = 0;
            while(!q)
                q = allocatorCallback.getNextQ();

            for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

                m.lock();

                vcount++;

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

    void queryROI(ev::vQueue &fillq, int c, unsigned int t, int x, int y, int r)
    {

        if(!vcount) return;

        m.lock();
        if(c == 0)
            fillq = surfaceLeft.getSurf_Tlim(t, x, y, r);
        else
            fillq = surfaceRight.getSurf_Tlim(t, x, y, r);
        vcount = 0;
        m.unlock();

    }

};



#endif

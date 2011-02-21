#ifndef __IKARTNAV_LASERTHREAD_H__
#define __IKARTNAV_LASERTHREAD_H__

#include <string>

#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>

#include "NavThread.h"

class LaserThread : public yarp::os::Thread
{
public:
    LaserThread(yarp::os::ResourceFinder *rf,NavThread *navThread)
        : mNavThread(navThread),mRF(rf)
    {
    }

    virtual bool threadInit()
    {
        std::string remote=mRF->check("remote",yarp::os::Value("/ikart")).asString().c_str();
        std::string local=mRF->check("local",yarp::os::Value("/ikartnav")).asString().c_str();
        
        mLaserPortI.open((local+"/laser:i").c_str());
        if (!yarp::os::Network::connect(mLaserPortI.getName(),(remote+"/laser:o").c_str()))
        {
            fprintf(stderr,"ERROR: can't connect to iKartCtrl laser port\n");
            return false;
        }

        return true;
    }
    
    virtual void run()
    {
        while (isRunning())
        {
            yarp::sig::Vector *rangeData=mLaserPortI.read();

            if (rangeData!=NULL && !isStopping())
            {   
                mNavThread->readLaser(*rangeData);
            }

            static unsigned int cycle=0;
            if (!(++cycle%100))
            {
                printf("laser\n");
                fflush(stdout);
            }
        }
    }
    
    virtual void threadRelease()
    {
        mLaserPortI.close();
    }

    virtual void onStop()
    {
        mLaserPortI.interrupt();
        mLaserPortI.close();
    }

protected:
    NavThread *mNavThread;
    yarp::os::ResourceFinder *mRF;
    yarp::os::BufferedPort<yarp::sig::Vector> mLaserPortI;
};

#endif
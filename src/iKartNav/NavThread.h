#ifndef __IKARTNAV_NAVTHREAD_H__
#define __IKARTNAV_NAVTHREAD_H__

#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Vector.h>

#include "Vec2D.h"

class NavThread : public yarp::os::RateThread
{
public:
    NavThread(unsigned int period,yarp::os::ResourceFinder *rf);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    void readLaser(yarp::sig::Vector& rangeData);

protected:
    bool setOmega(double omega);
    bool setVel(const Vec2D& vel);
    int abs(int x){ return x>=0?x:-x; }

// data
    // from config
    double mPeriod;
    double mRadius;

    double mMaxSpeed;
    double mMaxOmega;
    double mLinAcc;
    double mRotAcc;

    Vec2D mRFpos;
    int mNumSamples;
    int mNumSamplesByTwo;
    int mNumData;

    double mRangeMax;
    double mAngularRes;
    double mRFAngleMax;

    double mDangerZone;
    double mInfluenceZone;

    ///////////////
    ///////////////

    double mA,mB,mC;

    Vec2D mVel;
    double mOmega;

    Vec2D mVelRef;
    double mOmegaRef;

    double mTimeLastCycle;
    Vec2D mOdoPos;
    double mOdoRot;

    // scan data
    Vec2D *mObjects;
    bool *mIsValid;
    Vec2D *mMemBuffOld; 
    Vec2D *mMemBuffNew;
    int mMemBuffSize;
    int mMemBuffNum;

    int miMinOld;
    bool mFreeSpace;
    bool mBug;
    double mBugRange2;

    bool mHaveTarget;
    Vec2D mTarget;

    yarp::os::ResourceFinder *mRF;
    yarp::os::BufferedPort<yarp::os::Bottle> mTargetPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mCommandPortO;

    yarp::os::Semaphore mLaserSem;
};

#endif


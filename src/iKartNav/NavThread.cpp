#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

#include "NavThread.h"

NavThread::NavThread(unsigned int period,yarp::os::ResourceFinder *rf) 
    : RateThread(period),
      mRF(rf),
      mVel(0.0,0.0),
      mOmega(0.0),
      mVelRef(0.0,0.0),
      mOmegaRef(0.0),
      mOdoPos(0.0,0.0),
      mOdoRot(0.0),
      miMinOld(-1),
      mFreeSpace(true),
      mBug(false),
      mLaserSem(1)
{
    mObjects=NULL;
    mMemBuffOld=mMemBuffNew=NULL;

    mIsValid=NULL;
}

bool NavThread::threadInit()
{
    std::string local=mRF->check("local",yarp::os::Value("/ikartnav")).asString().c_str();
    std::string remote=mRF->check("remote",yarp::os::Value("/ikart")).asString().c_str();
    mPeriod=mRF->check("period",yarp::os::Value(0.02)).asDouble();
    mRadius=mRF->check("radius",yarp::os::Value(0.3575)).asDouble();
    mRFpos.x=mRF->check("laser_pos",yarp::os::Value(0.245)).asDouble();
    mDangerZone=mRF->check("danger_zone",yarp::os::Value(0.4)).asDouble();
    mInfluenceZone=mRF->check("field_zone",yarp::os::Value(1.2)).asDouble();
    mMaxSpeed=mRF->check("max_speed",yarp::os::Value(0.2)).asDouble();
    mMaxOmega=mRF->check("max_ang_speed",yarp::os::Value(30.0)).asDouble();
    mLinAcc=mRF->check("linear_acc",yarp::os::Value(0.5)).asDouble();
    mRotAcc=mRF->check("rot_acc",yarp::os::Value(90.0)).asDouble();
    mNumSamples=mRF->check("num_range_samples",yarp::os::Value(1080)).asInt();
    mRangeMax=mRF->check("range_max_dist",yarp::os::Value(30.0)).asDouble();
    mAngularRes=mRF->check("range_ang_res",yarp::os::Value(0.25)).asDouble();

    mNumSamplesByTwo=mNumSamples/2;
    mRFAngleMax=double(mNumSamplesByTwo)*mAngularRes;
    mMemBuffSize=(int)(360.0/mAngularRes)-mNumSamples;

    double R0=(mRadius+mDangerZone);
    double R1=(mRadius+mInfluenceZone);
    double R2=(mRangeMax);
    double K1=0.1;

    mB=((1.0-2.0*K1)*R0*R1+2.0*K1*R1*R2-R2*R0)/((1.0-2.0*K1)*R2+2.0*K1*R0-R1);
    mC=-1.0/(R2+mB);
    mA=K1*(R1+mB)*(R2+mB)/(R2-R1);

    ///////////////////////////////////

    mObjects=new Vec2D[mNumSamples];

    mIsValid=new bool[mNumSamples];
    for (int i=0; i<mNumSamples; ++i) mIsValid[i]=false;

    mMemBuffOld=new Vec2D[mMemBuffSize];
    mMemBuffNew=new Vec2D[mMemBuffSize];

    mMemBuffNum=0;

    mCommandPortO.open((local+"/control:o").c_str());

    if (!yarp::os::Network::connect(mCommandPortO.getName(),(remote+"/control:i").c_str()))
    {
        fprintf(stderr,"ERROR: can't connect to iKartCtrl command port\n");
        return false;
    }

    mTargetPortI.open((local+"/target:i").c_str());

    mTimeLastCycle=yarp::os::Time::now();

    return true;
}

void NavThread::run()
{
    static double dVersus=1.0;

    // (very poor) ODOMETRY
    // only used for target memory
    double timeNow=yarp::os::Time::now();
    double timeElapsed=timeNow-mTimeLastCycle;
    mTimeLastCycle=timeNow;
    mOdoPos+=timeElapsed*mVel.rot(mOdoRot);
    mOdoRot+=timeElapsed*mOmega;
    while (mOdoRot>=180.0) mOdoRot-=360.0;
    while (mOdoRot<-180.0) mOdoRot+=360.0;

    // set new target if it is received
    for (yarp::os::Bottle* bot; bot=mTargetPortI.read(false);)
    {
        yarp::os::ConstString cmd=bot->get(0).asString();

        if (cmd=="set_target")
        { 
            double heading=-bot->get(1).asDouble();
            double distance=bot->get(2).asDouble();
            Vec2D target=mOdoPos+distance*Vec2D(mOdoRot+heading);

            if ((mTarget-target).mod()>0.5)
            {
                mBug=false;
                mFreeSpace=true;

                mBugRange2=distance*distance;
            }

            mTarget=target;
            mHaveTarget=true;

            printf("NEW TARGET H=%lf D=%lf\n",heading,distance);
            fflush(stdout);
        }
        else if (cmd=="stop")
        {
            printf("STOP\n");
            fflush(stdout);
            mHaveTarget=false;
        }
    }

    // CONTROL

    if (mHaveTarget)
    {
        double Umax=0.0;
        double rMin=1000000.0;
        int iMin=-1;

        // target direction
        Vec2D T=mTarget-mOdoPos;
        double distance=T.mod();
        T.normalize();
        double Thead=T.rot(-mOdoRot).arg();
        double beta=(distance>=mRadius)?atan2(mRadius,sqrt(distance*distance-mRadius*mRadius)):0.0;

        bool bFreeWay=false;

        if (fabs(Thead)<mRFAngleMax-beta)
        {
            bFreeWay=true;

            for (int i=0; i<mNumSamples; ++i)
            {
                if (mIsValid[i])
                {
                    Vec2D R=(mObjects[i]-mOdoPos);

                    if (T*R>0.0 && R.mod()<distance && (R-T*(T*R)).mod()<mRadius+0.2)
                    {
                        bFreeWay=false;
                        break;
                    }
                }
            }
        }

        // obstacles correction
        Vec2D Rres;

        mLaserSem.wait();

        for (int i=0; i<mNumSamples; ++i)
        {
            if (mIsValid[i])
            {
                Vec2D R=mOdoPos-mObjects[i];
                double r=R.mod();
                double U=mA*(mC+1.0/((r)+mB));

                if (U<0.0) U=0.0; else if (U>1.0) U=1.0;

                Rres+=R.norm(U);

                if (U>Umax) Umax=U;

                if (r<rMin)
                {
                    rMin=r;
                    iMin=i;
                }
            }
        }

        for (int i=0; i<mMemBuffNum; ++i)
        {
            Vec2D R=mOdoPos-mMemBuffOld[i];
            double r=R.mod();
            double U=mA*(mC+1.0/((r)+mB));

            if (U<0.0) U=0.0; else if (U>1.0) U=1.0;

            Rres+=R.norm(U);

            if (U>Umax) Umax=U;

            if (r<rMin) rMin=r;
        }

        mLaserSem.post();

        Rres.normalize();

        Vec2D H;

        if (bFreeWay)
        {
            mBug=false;
            mFreeSpace=true;
            H=T;
        }
        else if (Rres*T>=0.0 && (bFreeWay || !mBug || (mBug && (mBugRange2>(mTarget-mOdoPos).mod2()))))
        {
            mBug=false;
            mFreeSpace=true;
            H=Rres.norm(Umax)+T.norm(1.0-Umax);
        }
        else 
        {
            if ((rMin-=mRadius)<0.0) rMin=0.0;

            if (rMin<mInfluenceZone)
            {
                Vec2D L=Rres.rotLeft();

                if (!mBug && (mFreeSpace || abs(iMin-miMinOld)>mNumSamples/3))
                {
                    dVersus=L*T>0.0?1.0:-1.0;
                    mFreeSpace=false;
                    mBug=false;
                }

                if (!mBug && dVersus*(L*T)<0.0)
                {
                    //mBugRange2=(mTarget-mOdoPos).mod2();
                    double range2=(mTarget-mOdoPos).mod2();        
                    if (range2<mBugRange2)
                    {
                        mBugRange2=range2;
                    }

                    mBug=true;
                }
                    
                L=dVersus*L.norm(sqrt(1.0-rMin/mInfluenceZone));

                if (mBug)
                {
                    H=Rres.norm()*(2.0*Umax-1.0)+L;
                }
                else
                {
                    H=Rres.norm(Umax)+T.norm(1.0-Umax)+L;
                }
            }
            else
            {
                mBug=false;
                mFreeSpace=true;
                H=Rres.norm(Umax)+T.norm(1.0-Umax);
            }
        }

        miMinOld=iMin;

        H.normalize();
        H=H.rot(-mOdoRot);

        double heading=H.arg();

        if (distance<0.05)
        {
            setVel(Vec2D::zero);
            setOmega(0.0);
        }
        else if (distance<0.25)
        {
            setVel(distance*H);
            setOmega(0.5*heading);
        }
        else
        {
            if (fabs(heading)<=45.0)
            {
                setVel(mMaxSpeed*H);
                setOmega(0.5*heading);
            }
            else
            {
                setVel(Vec2D::zero);
                setOmega(0.5*heading);
            }    
        }
    }
    else
    {
        setVel(Vec2D::zero);
        setOmega(0.0);
    }

    // SMOOTHING
    if (mVelRef!=mVel)
    {
        Vec2D Verr=mVelRef-mVel;
        if (Verr.mod()>mPeriod*mLinAcc)
            mVel+=Verr.norm(mPeriod*mLinAcc);
        else
            mVel=mVelRef;
    }

    if (mOmegaRef!=mOmega)
    {
        double Werr=mOmegaRef-mOmega;

        if (Werr>mPeriod*mRotAcc)
            mOmega+=mPeriod*mRotAcc;
        else if (Werr<-mPeriod*mRotAcc)
            mOmega-=mPeriod*mRotAcc;
        else
            mOmega=mOmegaRef;
    }

    // SEND COMMANDS
    yarp::os::Bottle& cmd=mCommandPortO.prepare();
    cmd.clear();
    cmd.addInt(1);
    cmd.addDouble(-mVel.arg());
    cmd.addDouble(75000.0*mVel.mod());
    cmd.addDouble(-1000.0*mOmega);
    cmd.addDouble(65000.0); // pwm %
    mCommandPortO.write();

    static unsigned int cycle=0;
    if (!(++cycle%25))
    {
        printf("commands S=%lf H=%lf W=%lf\n",mVel.mod(),-mVel.arg(),-mOmega);
    }
}

void NavThread::threadRelease()
{
    mCommandPortO.close();
    mTargetPortI.close();

    if (mObjects) delete [] mObjects;
    
    if (mIsValid) delete [] mIsValid;

    if (mMemBuffOld) delete [] mMemBuffOld;
    if (mMemBuffNew) delete [] mMemBuffNew;
}

void NavThread::readLaser(yarp::sig::Vector& rangeData)
{
    static const double cosAngleMaxL=cos(Vec2D::DEG2RAD*(mRFAngleMax-mAngularRes));
    static const double cosAngleMaxR=cos(Vec2D::DEG2RAD*mRFAngleMax);

    int memBuffNum=0;

    Vec2D rfPos=mOdoPos+mRFpos.rot(mOdoRot);
    Vec2D dir(mOdoRot);

    mLaserSem.wait();

    for (int i=mNumSamples-1; i>mNumSamplesByTwo; --i)
    {
        if (mIsValid[i])
        {
            if (dir*(mObjects[i]-rfPos).norm()<=cosAngleMaxL)
            {
                mMemBuffNew[memBuffNum++]=mObjects[i];
            }
            else
            {
                break;
            }
        }
    }

    for (int i=0; i<mNumSamplesByTwo; ++i)
    {
        if (mIsValid[i])
        {
            if (dir*(mObjects[i]-rfPos).norm()<=cosAngleMaxR)
            {
                mMemBuffNew[memBuffNum++]=mObjects[i];
            }
            else
            {
                break;
            }
        }
    }

    for (int i=0; i<mMemBuffNum && memBuffNum<mMemBuffSize; ++i)
    {
        if (dir*(mMemBuffOld[i]-rfPos).norm()<cosAngleMaxL)
        {
            mMemBuffNew[memBuffNum++]=mMemBuffOld[i];
        }
    }

    mMemBuffNum=memBuffNum;
    Vec2D *swap=mMemBuffOld;
    mMemBuffOld=mMemBuffNew;
    mMemBuffNew=swap;

    double range;

    for (int i=0; i<mNumSamples; ++i)
    {
        range=0.001*rangeData[i];

        if (range<mRangeMax)
        {
            mObjects[i]=rfPos+0.001*range*Vec2D(mOdoRot+double(i-mNumSamplesByTwo)*mAngularRes);
            mIsValid[i]=true;
        }
        else
        {
            mIsValid[i]=false;
        }
    }

    mLaserSem.post();
}

bool NavThread::setOmega(double omega)
{
    if (omega>mMaxOmega)
    {
        mOmegaRef=mMaxOmega;
        return false;
    }

    if (omega<-mMaxOmega)
    {
        mOmegaRef=-mMaxOmega;
        return false;
    }

    mOmegaRef=omega;

    return true;
}

bool NavThread::setVel(const Vec2D& vel)
{
    if (vel.mod()>mMaxSpeed)
    {
        mVelRef=vel.norm(mMaxSpeed);
        return false;
    }

    mVelRef=vel;

    return true;
}



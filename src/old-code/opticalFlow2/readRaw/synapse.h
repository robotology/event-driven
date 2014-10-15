#ifndef __SYNAPSE_H__
#define __SYNAPSE_H__

#include "eLucasKanade.h"

class Synapse
{
public:
    Synapse(int xdim,int ydim,int ndim,double sigma,double threshold,double alpha,double tau,int accTime);
	~Synapse();

    void filter(int x,int y,int p,unsigned int t);
    inline void openPort(std::string& str){ objFlow->openPort(str); };
    inline void closePort(){ objFlow->closePort(); };

private:
    int mXdim;
    int mYdim;
    int mNdim;

    double mThreshold;
    double mAlpha;
    double mOneByTau;
    int mAccTime;

	unsigned int mTimeStamp;

    yarp::sig::Matrix mImage0;
    yarp::sig::Matrix mImage1;
    yarp::sig::Matrix mPotential;
    yarp::sig::Matrix mTime;

    yarp::sig::Matrix mGaussianMask;

	short* X;
	short* Y;

	eLucasKanade* objFlow;
};

#endif //SYNAPSE_H

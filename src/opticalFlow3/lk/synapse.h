#ifndef __SYNAPSE_H__
#define __SYNAPSE_H__

#include <string>
#include "vecBuffer.h"

#define XDIM 128
#define YDIM 128
#define NEIGH 2

class Synapse
{
public:
    Synapse(int xdim,int ydim,int ndim,double sigma,double threshold,double alpha,double tau,int accTime);
	~Synapse();

    void filter(int x,int y,int p,unsigned int t);
    inline void openPort(std::string& name){ mPort.open(name.c_str()); }
    inline void closePort(){ mPort.close(); }

private:
    yarp::os::BufferedPort<vecBuffer> mPort;

    //unsigned int mTimeStamp;
    
    //double Vtx[9],Vty[9];

    double mMapGaussian[2*NEIGH+1][2*NEIGH+1];

    //int mMapN[XDIM][YDIM];
    int mMapTime[XDIM][YDIM];
    double mImageH[XDIM][YDIM];
    double mImageL[XDIM][YDIM];
    //double mMapVx[XDIM][YDIM];
    //double mMapVy[XDIM][YDIM];
    double It[128][XDIM];
    int mMapFired[XDIM][YDIM];

    int mNumber;
    int mSize;
    //int mExcluded;
};

#endif //SYNAPSE_H

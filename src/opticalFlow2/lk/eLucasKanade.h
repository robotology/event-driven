#ifndef ELUCASKANADE_H
#define ELUCASKANADE_H

#include <string>
#include <yarp/sig/Matrix.h>

#include "vecBuffer.h"

class eLucasKanade
{
public:
	eLucasKanade(int xdim,int ydim,int ndim,double sigma);
	~eLucasKanade();

    inline void openPort(std::string& name){ mPort.open(name.c_str()); };
    inline void closePort(){ mPort.close(); };

    void compute(short *X,short *Y,int N,yarp::sig::Matrix &img1,yarp::sig::Matrix &img0);

private:
    yarp::os::BufferedPort<vecBuffer> mPort;

    double mLambda;

    int mXdim;
    int mYdim;
    int mNdim;

    yarp::sig::Matrix mW;
    
	void send(int x,int y,double vx,double vy);
};

#endif //ELUCASKANADE_H

/*
 * FOEFinder.h
 *
 *  Created on: Feb 29, 2012
 *      Author: fuozhan
 */

#ifndef FOEFINDER_H_
#define FOEFINDER_H_


#include "VelocityBuffer.h"

#include <cmath>
#include <vector>
#include <iostream>

#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageDraw.h>

#define X_DIM 128
#define Y_DIM 128
#define LEAK_RATE .9
#define BIN_NO  4  // used for "bin" function
#define NGHBR_RADIAN (M_PI / 10)

///For FOE in development
#define MAX_REG_NGHBR 7
#define WEIGHT_FACTOR .05


using namespace std;
using namespace yarp::os;

class FOEFinder{
    //yarp::sig::ImageOf<yarp::sig::PixelInt> wrldStatus;
    //BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelFloat> > * outPort;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb > > * outPort;

    unsigned int crnTS;
    yarp::sig::Matrix tsStatus;
    yarp::sig::Matrix wrldStatus;
    yarp::sig::Matrix objMap;

    int foeX;
    int foeY;
    float foeProb;

    vector< float > binSlopes;

    yarp::sig::Matrix vxVels;
    yarp::sig::Matrix vyVels;

    int sobelx(yarp::sig::Matrix & mtx, int stR, int stC);
    int sobely(yarp::sig::Matrix & mtx, int stR, int stC);

    void initBins(int binNo);//used for "bin" function

    void populateBins(int idx1, int idx2, int x, int y, double vx, double vy, double weight);

    void makeObjMap(VelocityBuffer &, int foeX, int foeY);

public:
    FOEFinder();
    void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb > > * );
    ~FOEFinder();

    /*To compute FOE, the retina is divided to a predefined number of bins (BIN_NO)*/
    void bin(VelocityBuffer &);

    /*To compute FOE, we consider the neighborhood around each flow (NGHBR_RADIAN) */
    void bin2(VelocityBuffer &);


    void velDivergance(VelocityBuffer &);
    void velNormal(VelocityBuffer &);

};



#endif /* FOEFINDER_H_ */

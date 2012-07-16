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
#define BIN_NO  4  // used for "bin" function
#define NGHBR_RADIAN (M_PI / 10)  //used by "bin2" function  //The neihboring arc

///For FOE in development
#define FOEMAP_MAXREG_SIZE 7
#define WEIGHT_FACTOR .05
#define LEAK_RATE (- 0.00001)


#define FILTER_NGHBRHD 3

using namespace std;
using namespace yarp::os;

class FOEFinder{
    //BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelFloat> > * outPort;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb > > * outPort;

    unsigned int crnTS;
    yarp::sig::Matrix tsStatus;
    yarp::sig::Matrix foeMap;
    yarp::sig::Matrix objMap;
    yarp::sig::Matrix objMapTS;

    int foeX;
    int foeY;
    float foeProb;

    vector< float > binSlopes;
    void initBins(int binNo);//used for "bin" function
    void populateBins(int idx1, int idx2, int x, int y, double vx, double vy, double weight); //

    /*To compute FOE, the retina is divided to a predefined number of bins (BIN_NO)*/
    void bin(VelocityBuffer &);

    /*To compute FOE, we consider the neighborhood around each flow (NGHBR_RADIAN) */
    void bin2(VelocityBuffer &);

    void getFOEMapMax(int & centerX, int & centerY, double & foeMaxValue);

public:
    FOEFinder();
    void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb > > * );
    ~FOEFinder();

    void computeFoE(VelocityBuffer & , bool vis = true);
    void makeObjMap(VelocityBuffer &);
    void makeObjMap2(VelocityBuffer &);



};



#endif /* FOEFINDER_H_ */

/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
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
#ifndef __FLOWVIEWER_H__
#define __FLOWVIEWER_H__

#include "VelocityBuffer.h"

#include <cmath>
#include <iostream>
#include <string>
#include <cmath>

#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageDraw.h>

#include <gsl/gsl_statistics.h>

#define XDIM 128
#define YDIM 128
#define MEDIAN_NGHBRHD 2

typedef unsigned char uchar;

class tsOptFlowViewer:public yarp::os::BufferedPort<VelocityBuffer>
{
    //yarp::sig::ImageOf<yarp::sig::PixelMono16> mBaseImg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> mBaseImg;
    //BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * outPort;
    BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb> > * outPort;

    yarp::sig::Matrix xVels;
    yarp::sig::Matrix yVels;
    int zeroFlag;

    double kth_smallest(double * a, int n, int k);

    void medianFilter2D(VelocityBuffer &);
    void medianFilterSeprabale(VelocityBuffer & );

    void resetVelMtrx();
    double sign(double&);
    void createColorMap();

    std::string displayMode;
    double toNorm;
    uchar *colorMap;
    bool *exist;
public:
    tsOptFlowViewer(std::string&, double&);
    //void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * );
    void setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb> > * );
    ~tsOptFlowViewer();
    void onRead(VelocityBuffer &);
};

#endif //OPTICALFLOWVIEWER

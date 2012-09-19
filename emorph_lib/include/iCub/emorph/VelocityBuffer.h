/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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


#ifndef VELOCITYBUFFER_H_
#define VELOCITYBUFFER_H_

#define BUFFER_LENGTH 5000

#include <float.h>
#include <cmath>
#include <iostream>

#include <vector>

#include <yarp/os/Portable.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>

using namespace yarp::os;

class VelocityBuffer : public yarp::os::Portable{
    int size;
    double vxMin;
    double vxMax;
    double vyMin;
    double vyMax;
    short Xs[BUFFER_LENGTH];
    short Ys[BUFFER_LENGTH];
    double Vxs[BUFFER_LENGTH];
    double Vys[BUFFER_LENGTH];
    unsigned long TSs [BUFFER_LENGTH];
    unsigned long bufferingTime;

    void initialize();

public:

    VelocityBuffer();
    VelocityBuffer(unsigned long timeInt );

    virtual ~VelocityBuffer();
    virtual bool read(ConnectionReader &);
    virtual bool write(ConnectionWriter &);

    void setData(const VelocityBuffer &src);
    bool addData(const VelocityBuffer &data);
    bool addData(short, short, double, double,  unsigned long , double);
    bool addDataCheckFull(short, short, double, double, unsigned long, double );
    bool isFull();
    bool isEmpty();
    void emptyBuffer();

    inline short getSize(){return size;};
    inline short getX(int idx){return Xs[idx];};
    inline short getY(int idx){return Ys[idx];};
    inline double getVx(int idx){return Vxs[idx];};
    inline double getVy(int idx){return Vys[idx];};
    inline unsigned long getTs(int idx){return TSs[idx];};
    inline double getRel(int idx){return 1;};

    double getVxMin();
    double getVxMax();
    double getVyMin();
    double getVyMax();

    void setVx(int idx, double vx );
    void setVy(int idx, double vy );
    void setVxMax(double v){vxMax = v;};
    void setVyMax(double v){vyMax = v;};
};


#endif /* VELOCITYBUFFER_H_ */


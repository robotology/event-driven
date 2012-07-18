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
#ifndef TSOPTFLOW_HPP
#define TSOPTFLOW_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdio>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Matrix.h>

#include "eventBuffer.h"

#include "tsOptFlowThread.hpp"
#include "sendVelBuf.hpp"

class tsOptFlow:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer>
{
public:
    tsOptFlow(uint&, uint&, std::string&, uint&, uint&, uint&, double&, uint&, uint&, uint&, double&, double&, int&, uint&, bool&, yarp::os::BufferedPort<VelocityBuffer>*);
    ~tsOptFlow();
    void onRead(emorph::ebuffer::eventBuffer&);
private:
    
    yarp::os::Semaphore *mutex;
    tsOptFlowThread *tsofThreadPos;
    tsOptFlowThread *tsofThreadNeg;
    sendVelBuf *sendvelbuf;
    yarp::sig::Matrix *vxMat;   
    yarp::sig::Matrix *vyMat;

    VelocityBuffer *velBuf;
};

#endif //TSOPTFLOW_HPP

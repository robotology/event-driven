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
#ifndef SENDVELBUF_HPP
#define SENDVELBUF_HPP

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

#include "VelocityBuffer.h"

#define RATE 5
typedef unsigned int uint;

class sendVelBuf: public yarp::os::RateThread
{
public:
    sendVelBuf(VelocityBuffer*, yarp::os::Semaphore*, yarp::os::BufferedPort<VelocityBuffer>*, uint&);
    ~sendVelBuf();

    void run();
private:
    VelocityBuffer *velBuf;
    yarp::os::BufferedPort<VelocityBuffer>* port;
    yarp::os::Semaphore *mutex;

    bool init;
    uint refts;
    uint acc;
};

#endif //SENDVELBUF_HPP

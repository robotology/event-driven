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
#include "sendVelBuf.hpp"

using namespace yarp::os;
sendVelBuf::sendVelBuf(VelocityBuffer* _velb, Semaphore *_sem, BufferedPort<VelocityBuffer> *_port, uint &_acc)
:RateThread(RATE), acc(_acc)
{
    velBuf=_velb;
    mutex=_sem;
    port=_port;

    init=true;
}

sendVelBuf::~sendVelBuf()
{
    port->close();
}

void sendVelBuf::run()
{
    //std::cout << "Mutex free?" << std::endl;
    mutex->wait();
    //std::cout << "\tMutex took" << std::endl;
    if(!velBuf->isEmpty())
    {
        if(init)
        {
            refts=velBuf->getTs(0);
            init=false;
        }
        else if( ((refts+acc)<velBuf->getTs(velBuf->getSize()-1)) || velBuf->isFull() )
        {
            refts=velBuf->getTs(velBuf->getSize()-1);
            VelocityBuffer& tmp = port->prepare();
            tmp=*velBuf;
            port->write();
            velBuf->emptyBuffer();
        }
    }
    mutex->post();
   // std::cout << "Mutex freed" << std::endl;
}

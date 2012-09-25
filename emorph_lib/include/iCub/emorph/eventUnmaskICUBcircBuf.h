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
#ifndef EVENTUNMASKICUBCIRCBUF_H
#define EVENTUNMASKICUBCIRCBUF_H

#define _DEBUG_
#define NBBUF 3
#include "eventUnmask.h"
#include <yarp/os/Semaphore.h>

#ifdef _DEBUG_
#include <cstdio>
#endif

namespace emorph
{
namespace eunmask
{

class eventUnmaskICUBcircBuf:public emorph::eunmask::eventUnmask
{
public:
    eventUnmaskICUBcircBuf(bool _save=false);
    eventUnmaskICUBcircBuf(const eventUnmaskICUBcircBuf&);

    ~eventUnmaskICUBcircBuf();

    eventUnmaskICUBcircBuf& operator=(const eventUnmaskICUBcircBuf&);

	void setBuffer(char*, uint){};
	void setBuffer(emorph::ebuffer::eventBuffer&);
    void reshapeBuffer();
    int getUmaskedData(uint&, uint&, int&, uint&, uint&);
    int reset();
private:
    void saveBuffer(char*, uint);
    uint snapBuffer();

    void objcpy(const eventUnmaskICUBcircBuf&);
    int whichBuf;
    yarp::os::Semaphore *cBufMutex;
    uint **buffer;
    uint *szBufs;
    uint *bufSnapShot;
    uint tsPacket;
    bool bufInUse;
    bool save;
#ifdef _DEBUG_
    FILE *dump;
#endif
};
}
}
#endif //EVENTUNMASKICUB_H

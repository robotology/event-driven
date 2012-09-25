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
#ifndef EVENTUNMASK_H
#define EVENTUNMASK_H

//#ifdef _DEBUG
#include <iostream>
//#endif
#include <sstream>

#include <ctime>
#include <cmath>
#include <cstring>

#include <stdint.h>
//#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>

#include "eventBuffer.h"

#define BUFFERBLOCK 65536//32768 

typedef unsigned int uint;
typedef uint32_t u32;

using namespace std;
namespace emorph
{
namespace eunmask
{

class eventUnmask
{
public:
	//unmaskingthread(){};
	eventUnmask();
	eventUnmask(const eventUnmask&);
	~eventUnmask();

    eventUnmask& operator=(const eventUnmask&);

	virtual void setBuffer(char*, uint)=0;
	virtual void setBuffer(emorph::ebuffer::eventBuffer&)=0;
    virtual void reshapeBuffer()=0;
    virtual int reset()=0;
    virtual int getUmaskedData(uint&, uint&, int&, uint&, uint&)=0;

    uint getIndex(){return eventIndex;}; 
protected:
    void objcpy(const eventUnmask&);
	/**
	* @brief This method unmasked the raw which come from the TCP socket
	* This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
	* @param *evPU A pointer on the raw casted from char* to int*
	* @param x Set with the x coordinate of the pixel
	* @param y Set with the y coordinate of the pixel
	* @param pol Set with the ON/OFF polarity of the pixel.
	* @param ts ...
	*/
	void unmaskEvent(uint, uint&, uint&, int&, uint&);
    virtual void saveBuffer(char*, uint)=0;

	/*Variables needed by the unmask method*/
	uint szBuffer;
	uint szBufSnapShot;
    uint szInMem;
    
    uint nEvent;
    uint eventCounter;
    uint eventIndex;
    
    uint blob;

	uint yshift;
	uint xshift;
	uint polshift;
	uint eyeshift;

	uint xmask;
	uint ymask;
	uint polmask;
	uint eyemask;

	uint retinalSize;

    uint timestampMonotonyWrap;
    uint ptimestamp;
    yarp::os::Semaphore mutex;
	//**************************************
};
}
}
#endif //EVENTUNMASK_H

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
#ifndef UNMASK_HPP
#define UNMASK_HPP

//#ifdef _DEBUG
#include <iostream>
//#endif
#include <sstream>

#include <ctime>
#include <cmath>
#include <cstring>

//#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>

#define BUFFERBLOCK 65536//32768 

typedef unsigned int uint;

using namespace std;
class unmask
{
public:
	//unmaskingthread(){};
	unmask();
	unmask(const unmask&);
	~unmask();

    unmask& operator=(const unmask&);

	virtual void setBuffer(char*, uint)=0;
    virtual void reshapeBuffer()=0;
    virtual int reset()=0;
    virtual int getUmaskedData(uint&, uint&, int&, uint&, uint&)=0;

    uint getIndex(){return eventIndex;}; 
protected:
    void objcpy(const unmask&);
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

#endif //UNMASK_HPP

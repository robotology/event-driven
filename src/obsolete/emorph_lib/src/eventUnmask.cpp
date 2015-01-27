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
#include "iCub/emorph/eventUnmask.h"

namespace emorph
{
namespace eunmask
{
eventUnmask::eventUnmask()
{
    eventCounter=0;
    eventIndex=0;
    szInMem=BUFFERBLOCK;
    szBuffer=0;
    szBufSnapShot=0;
    nEvent=0;

    timestampMonotonyWrap=0;
    ptimestamp=0;
    blob=0;

    yshift=0;
    xshift=0;
    polshift=0;
    eyeshift=0;
    xmask=0;
    ymask=0;
    polmask=0;
    eyemask=0;
    retinalSize=0;
}

eventUnmask::eventUnmask(const eventUnmask &_obj)
{
    objcpy(_obj);
}

eventUnmask::~eventUnmask()
{
}

void eventUnmask::objcpy(const eventUnmask &_obj)
{
    if(this!=&_obj)
    {
        eventCounter=_obj.eventCounter;
        eventIndex=_obj.eventIndex;
        szInMem=_obj.szInMem;
        szBuffer=_obj.szBuffer;
        szBufSnapShot=_obj.szBufSnapShot;
        nEvent=_obj.nEvent;

        timestampMonotonyWrap=_obj.timestampMonotonyWrap;
        ptimestamp=_obj.ptimestamp;
        blob=_obj.blob;
        yshift=_obj.yshift;
        xshift=_obj.xshift;
        polshift=_obj.polshift;
        eyeshift=_obj.eyeshift;
        xmask=_obj.xmask;
        ymask=_obj.ymask;
        polmask=_obj.polmask;
        eyemask=_obj.eyemask;
        retinalSize=_obj.retinalSize;
    }
}

eventUnmask& eventUnmask::operator=(const eventUnmask &_obj)
{
    objcpy(_obj);
    return *this;
}

void eventUnmask::unmaskEvent(uint evPU, uint& x, uint& y, int& pol, uint& eye)
{
	//x   = (retinalSize-1) - ((evPU & xmask)>>xshift);
	x   = ((evPU & xmask)>>xshift);
	y   = ((evPU & ymask)>>yshift);
	pol = (((evPU & polmask)>>polshift)==0)?-1:1;	//+1 ON, -1 OFF
	eye = ((evPU & eyemask)>>eyeshift);
}

}
}

//*********************************************************


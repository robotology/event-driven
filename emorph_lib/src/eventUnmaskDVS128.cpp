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
#include "iCub/emorph/eventUnmaskDVS128.h"

namespace emorph
{
namespace eunmask
{
eventUnmaskDVS128::eventUnmaskDVS128(uint _type, bool _save)
:save(_save)
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

    xmask   = 0x000000fE;
    ymask   = 0x00007f00;
    polmask = 0x00000001;
    eyemask = 0x00008000;

    yshift = 8;
    xshift = 1;
    polshift = 0;
    eyeshift=15;

    retinalSize = 128;

    typeOfRecord=_type;
    buffer=new char[BUFFERBLOCK];
    bufSnapShot=NULL;
}

eventUnmaskDVS128::eventUnmaskDVS128(const eventUnmaskDVS128 &_obj)
{
    objcpy(_obj);
}

eventUnmaskDVS128::~eventUnmaskDVS128()
{
    delete[] buffer;
}

eventUnmaskDVS128& eventUnmaskDVS128::operator=(const eventUnmaskDVS128 &_obj)
{
    objcpy(_obj);
    return *this;
}

void eventUnmaskDVS128::objcpy(const eventUnmaskDVS128 &_obj)
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

        delete[] buffer;
        buffer = new char[szInMem];
        memcpy(buffer, _obj.buffer, szBuffer);

        typeOfRecord=_obj.typeOfRecord;
    }
}

void eventUnmaskDVS128::setBuffer(char* i_buffer, uint i_sz)
{
    mutex.wait();
    
    //std::cout << "\t\tConcate a new buffer" << std::endl;
    if(szBuffer==0)
    {
#ifdef _DEBUG
        std::cout << "[eventUnmaskDVS128] expand the size of the gBuffer" << std::endl;
#endif
        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)i_sz/(double)BUFFERBLOCK)*BUFFERBLOCK;
        //delete[] buffer;
        buffer = new char[szInMem];
    }
    else if(szBuffer+i_sz>=szInMem)
    {
#ifdef _DEBUG
        std::cout << "[eventUnmaskDVS128] expand the size and recopy of the gBuffer" << std::endl;
#endif
        char *buftmp=new char[szBuffer];
        memcpy(buftmp, buffer, szBuffer);

        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)(i_sz+szBuffer)/(double)BUFFERBLOCK)*BUFFERBLOCK;
        delete[] buffer;
        buffer = new char[szInMem];

        memcpy(buffer, buftmp, szBuffer);
    }
#ifdef _DEBUG
    std::cout << "[eventUnmaskDVS128] concat the buffer (size in byte: " << szBuffer+i_sz << ", real size in mem: " << szInMem << ")" << std::endl;
#endif
    memcpy(buffer+szBuffer, i_buffer, i_sz);
    szBuffer+=i_sz;
    nEvent=szBuffer/typeOfRecord;

    mutex.post();
}

void eventUnmaskDVS128::reshapeBuffer()
{
    mutex.wait();
    std::cout << "\t\t*** Reshape the buffer ***" << std::endl;
    szBuffer=szBuffer-(eventIndex*typeOfRecord);
    std::cout << "\t\t\t- Remaining size in byte: " << szBuffer << std::endl;
    char *buftmp=new char[szBuffer];
    memcpy(buftmp, buffer+eventIndex, szBuffer);
    delete[] buffer;
    szInMem=(uint)ceil((double)szBuffer/(double)BUFFERBLOCK)*BUFFERBLOCK;
    std::cout << "\t\t\t- New real size in mem: " << szInMem << std::endl;
    buffer=new char[szInMem];
    memcpy(buffer, buftmp, szBuffer);
    eventIndex=0;
    eventCounter=0;
    nEvent=szBuffer/typeOfRecord;
    std::cout << "\t\t*** Buffer reshaped ***" << std::endl;
    //std::cout << "\t\t\t- New size in byte: " << sz << ", real size in mem: " << szInMem << std::endl;
    mutex.post();
}

int eventUnmaskDVS128::getUmaskedData(uint& addrx, uint& addry, int& polarity, uint& eye, uint& timestamp)
{
    int res=1;
    if((eventIndex+(typeOfRecord))>szBufSnapShot || szBufSnapShot==0)
    {
#ifdef _DEBUG
        std::cout << "[eventUnmaskDVS128] make a snapshot of the buffer" << std::endl;
#endif
        if(!snapBuffer())
            return 0;
    }
#ifdef _DEBUG
    std::cout << "[eventUnmaskDVS128] unmask data [" << eventIndex << " " << eventIndex+(typeOfRecord-1) << "] on " << szBufSnapShot << std::endl;
#endif
    if( (typeOfRecord==4) && ((bufSnapShot[eventIndex+3]&0x80)==0x80) )
    {
        // timestamp bit 15 is one -> wrap
        // now we need to increment the timestampMonotonyWrap
        timestampMonotonyWrap+=0x4000; //uses only 14 bit timestamps
    }
    else if( (typeOfRecord==4) && ((bufSnapShot[eventIndex+3]&0x40)==0x40  ) )
    {
        // timestamp bit 14 is one -> timestampMonotonyWrap reset
        // this firmware version uses reset events to reset timestamps
        timestampMonotonyWrap=0;
    }
    else
    {
//----------------------------Unmask the data----------------------------------------------------------//
        //uint blob;
        switch(typeOfRecord)
        {
            case 4: blob = ((0x000000FF&bufSnapShot[eventIndex]))|((0x000000FF&bufSnapShot[eventIndex+1])<<8);
                    unmaskEvent(blob, addrx, addry, polarity, eye);
                    timestamp = (((0x000000FF&bufSnapShot[eventIndex+2]))|((0x000000FF&bufSnapShot[eventIndex+3])<<8))+timestampMonotonyWrap;
                    break;
            case 6: blob = ((0x000000FF&bufSnapShot[eventIndex+1]))|((0x000000FF&bufSnapShot[eventIndex])<<8);
                    unmaskEvent(blob, addrx, addry, polarity, eye);
                    timestamp = ((0x000000FF&bufSnapShot[eventIndex+5]))|((0x000000FF&bufSnapShot[eventIndex+4])<<8)|((0x000000FF&bufSnapShot[eventIndex+3])<<16)|((0x000000FF&bufSnapShot[eventIndex+2])<<24);
                    break;
            case 8: blob = (0x000000FF&bufSnapShot[eventIndex+3])|((0x000000FF&bufSnapShot[eventIndex+2])<<8)|((0x000000FF&bufSnapShot[eventIndex+1])<<16)|((0x000000FF&bufSnapShot[eventIndex])<<24);
                    unmaskEvent(blob, addrx, addry, polarity, eye);
                    timestamp = (0x000000FF&bufSnapShot[eventIndex+7])|((0x000000FF&bufSnapShot[eventIndex+6])<<8)|((0x000000FF&bufSnapShot[eventIndex+5])<<16)|((0x000000FF&bufSnapShot[eventIndex+4])<<24);
                    break;
        }
        if(blob&0x8000)
            res=2;
        if(ptimestamp>timestamp)
            timestamp=ptimestamp;
        ptimestamp=timestamp;
        
        addry=127-addry;
    }
    eventIndex+=typeOfRecord;
    return res;
}

uint eventUnmaskDVS128::snapBuffer()
{
    mutex.wait();
    if(szBuffer==0)
    {
        mutex.post();
        return 0;
    }
#ifdef _DEBUG
    std::cout << "[eventUnmaskDVS128] buffer not empty (=> size: " << szBuffer << ", make the snapShot" << std::endl;
#endif
    delete[] bufSnapShot;
    bufSnapShot = new char[szBuffer];
    memcpy(bufSnapShot, buffer, szBuffer);
    szBufSnapShot=szBuffer;
    nEvent=szBufSnapShot/typeOfRecord;
    eventCounter=0;
    eventIndex=0;

    delete[] buffer;
    buffer =NULL;//new char[BUFFERBLOCK];
    szBuffer=0;
    szInMem=0;//BUFFERBLOCK;

    mutex.post();
    return 1;
}

int eventUnmaskDVS128::reset()
{
    mutex.wait();

    delete[] bufSnapShot;
    bufSnapShot = NULL;
    szBufSnapShot=0;
    nEvent=0;
    eventCounter=0;
    eventIndex=0;

    delete[] buffer;
    buffer =NULL;//new char[BUFFERBLOCK];
    szBuffer=0;
    szInMem=0;//BUFFERBLOCK;

    ptimestamp=0;
    timestampMonotonyWrap=0;
    mutex.post();
    return 1;
}

void eventUnmaskDVS128::saveBuffer(char *_buf, uint _sz)
{    
}

}
}

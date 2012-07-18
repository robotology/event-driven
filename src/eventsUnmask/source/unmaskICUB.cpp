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
#include "unmaskICUB.hpp"

unmaskICUB::unmaskICUB(bool _save)
:save(_save)
{
    eventCounter=0;
    eventIndex=0;
    szInMem=BUFFERBLOCK;
    szBuffer=0;
    szBufSnapShot=0;
    nEvent=0;

    timestampMonotonyWrap=0;
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

    buffer=new uint[BUFFERBLOCK];
}

unmaskICUB::unmaskICUB(const unmaskICUB &_obj)
{
    objcpy(_obj);
}

unmaskICUB::~unmaskICUB()
{
    delete[] buffer;
}

unmaskICUB& unmaskICUB::operator=(const unmaskICUB &_obj)
{
    objcpy(_obj);
    return *this;
}

void unmaskICUB::objcpy(const unmaskICUB &_obj)
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
        buffer = new uint[szInMem];
        memcpy(buffer, _obj.buffer, szBuffer);
    }
}

void unmaskICUB::setBuffer(char* i_buffer, uint i_sz)
{
    mutex.wait();
    //std::cout << "\t\tConcate a new buffer" << std::endl;
    
    if(szBuffer=0)
    {
        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)i_sz/(double)BUFFERBLOCK)*BUFFERBLOCK;
        //delete[] buffer;
        buffer = new uint[szInMem/4];
    }
    else if(szBuffer+i_sz>=szInMem)
    {
        //std::cout << "\t\t\t- Expand the size of the gBuffer" << std::endl;
        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)(i_sz+szBuffer)/(double)BUFFERBLOCK)*BUFFERBLOCK;
        uint *buftmp=new uint[szBuffer/4];
        memcpy(buftmp, buffer, szBuffer);
        delete[] buffer;
        buffer = new uint[szInMem/4];
        memcpy(buffer, buftmp, szBuffer);
    }
    //std::cout << "\t\t\t- Concat the buffer (size in byte: " << sz+i_sz << ", real size in mem: " << szInMem << ")" << std::endl;
    memcpy(buffer+(szBuffer/4), (uint*)i_buffer, i_sz);
    szBuffer+=i_sz;
    nEvent=szBuffer/4;
    mutex.post();
}

void unmaskICUB::reshapeBuffer()
{
    mutex.wait();
    std::cout << "\t\t*** Reshape the buffer ***" << std::endl;
    szBuffer=szBuffer-(eventIndex*4);
    std::cout << "\t\t\t- Remaining size in byte: " << szBuffer << std::endl;
    uint *buftmp=new uint[szBuffer/4];
    memcpy(buftmp, buffer+eventIndex, szBuffer);
    delete[] buffer;
    szInMem=(uint)ceil((double)szBuffer/(double)BUFFERBLOCK)*BUFFERBLOCK;
    std::cout << "\t\t\t- New real size in mem: " << szInMem << std::endl;
    buffer=new uint[szInMem/4];
    memcpy(buffer, buftmp, szBuffer);
    eventIndex=0;
    eventCounter=0;
    nEvent=szBuffer/4;
    std::cout << "\t\t*** Buffer reshaped ***" << std::endl;
    //std::cout << "\t\t\t- New size in byte: " << sz << ", real size in mem: " << szInMem << std::endl;
    mutex.post();
}

int unmaskICUB::getUmaskedData(uint& cartX, uint& cartY, int& polarity, uint& eye, uint& timestamp)
{
    if((eventIndex+8)>szBufSnapShot || szBufSnapShot==0)
    {
        if(!snapBuffer())
            return 0;
    }
    if(eventCounter++>=nEvent)
        return 0;
        // unmask the data ( first 4 bytes timestamp, second 4 bytes address)
    tsPacket = buffer[eventIndex++];
    //Check if s tamistamp wrap around occured
    if ((tsPacket & 0xFC000000) == 0x88000000){ // if it's TSWA then skip it
        timestampMonotonyWrap += 0x04000000;
        tsPacket = buffer[eventIndex++];
    }
    timestamp = (tsPacket &  0x03FFFFFF) + timestampMonotonyWrap;

    blob = buffer[eventIndex++];
    blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!
    unmaskEvent(blob, cartX, cartY, polarity, eye);
    return 1;
}

uint unmaskICUB::snapBuffer()
{
    mutex.wait();
    if(szBuffer==0)
    {
        mutex.post();
        return 0;
    }
    delete[] bufSnapShot;
    bufSnapShot = new uint[szBuffer/4];
    memcpy(bufSnapShot, buffer, szBuffer);
    szBufSnapShot=szBuffer;
    nEvent=szBufSnapShot/8;
    eventCounter=0;
    eventIndex=0;

    delete[] buffer;
    buffer =NULL;//new char[BUFFERBLOCK];
    szBuffer=0;
    szInMem=0;//BUFFERBLOCK;

    mutex.post();
    return 1;
}

int unmaskICUB::reset()
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


void unmaskICUB::saveBuffer(char *_buf, uint _sz){}

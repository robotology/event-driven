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
#include "iCub/emorph/eventUnmaskICUBcircBuf.h"

namespace emorph
{
namespace eunmask
{
eventUnmaskICUBcircBuf::eventUnmaskICUBcircBuf(bool _save)
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

    //buffer=new uint[BUFFERBLOCK/4];
    bufSnapShot=NULL;
#ifdef _DEBUG_
    dump = fopen("/home/clercq/Documents/Temp/iCubDump.txt", "w");
    if(dump==NULL)
        std::cout << "[eventUnmaskICUBcircBuf] Error, file can't be opened or created" << std::endl;
#endif
    bufInUse=false;
    whichBuf=0;
    cBufMutex=new yarp::os::Semaphore[NBBUF];
    buffer=new uint*[NBBUF];
    szBufs=new uint[NBBUF];
    for(uint i=0; i<NBBUF; i++)
    {
        buffer[i]=new uint[BUFFERBLOCK/sizeof(uint)];
        szBufs[i]=0;
    }
        
}

eventUnmaskICUBcircBuf::eventUnmaskICUBcircBuf(const eventUnmaskICUBcircBuf &_obj)
{
    objcpy(_obj);
}

eventUnmaskICUBcircBuf::~eventUnmaskICUBcircBuf()
{
    for(uint i=0; i<NBBUF; ++i)
        delete[] buffer[i];
    delete[] buffer;
    delete[] cBufMutex;
    delete[] szBufs;
    delete[] bufSnapShot;
#ifdef _DEBUG_
    fclose(dump);
#endif
}

eventUnmaskICUBcircBuf& eventUnmaskICUBcircBuf::operator=(const eventUnmaskICUBcircBuf &_obj)
{
    objcpy(_obj);
    return *this;
}

void eventUnmaskICUBcircBuf::objcpy(const eventUnmaskICUBcircBuf &_obj)
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

        whichBuf=_obj.whichBuf;
        for(uint i=0; i<NBBUF; i++)
        {
            cBufMutex[i]=_obj.cBufMutex[i];
            szBufs[i]=_obj.szBufs[i];
            delete[] buffer[i];
            buffer[i]=new uint[szBufs[i]/sizeof(uint)];
            memcpy(buffer[i], _obj.buffer[i], szBufs[i]);
        }
    }
}

void eventUnmaskICUBcircBuf::setBuffer(char* i_buffer, uint i_sz)
{    
    cBufMutex[whichBuf].wait();
    //std::cout << "[eventUnmaskICUBcircBuf] setBuffer, fills buffer " << whichBuf << std::endl;
    
    //szInMem+=BUFFERBLOCK;
    szInMem=(uint)ceil((double)i_sz/(double)BUFFERBLOCK)*BUFFERBLOCK;
    if(buffer[whichBuf]!=NULL) delete[] buffer[whichBuf];
    buffer[whichBuf] = new uint[szInMem/sizeof(uint)];
    memset(buffer[whichBuf], 0, szInMem);
    memcpy(buffer[whichBuf], i_buffer, i_sz);
    szBufs[whichBuf]=i_sz;
    nEvent=szBufs[whichBuf]/sizeof(uint);

    cBufMutex[whichBuf].post();
    mutex.wait();
    whichBuf=(++whichBuf)%NBBUF;
    mutex.post();
    bufInUse=true;

    //std::cout << "[eventUnmaskICUBcircBuf] leaves setBuffer" << std::endl;
}

void eventUnmaskICUBcircBuf::reshapeBuffer()
{
}

int eventUnmaskICUBcircBuf::getUmaskedData(uint& cartX, uint& cartY, int& polarity, uint& eye, uint& timestamp)
{
    int res=0;
    //if(4*(eventIndex+3)>szBufSnapShot || szBufSnapShot==0)
    //if(4*(eventIndex-1)>=szBufSnapShot || szBufSnapShot==0)
    if((double)(4*(eventIndex-1))>=0.95*(double)szBufSnapShot || szBufSnapShot==0)
    //if((szBufSnapShot-(4*(eventIndex)))<3 || szBufSnapShot==0)
    {
        if(!snapBuffer())
            return 0;
    }
    tsPacket = bufSnapShot[eventIndex++];
    //Check if s tamistamp wrap around occured
    if(tsPacket & 0x80000000)
    {
        if ((tsPacket & 0xFC000000) > 0x80000000)
        {
            if ((tsPacket & 0xFC000000) == 0x88000000)
            {
#ifdef _DEBUG_
                fprintf(dump,"%08X ",tsPacket);
                fprintf(dump,"%s","CAFECAFE");
                fprintf(dump,"%s\n","<=WRAP");
#endif
                timestampMonotonyWrap += 0x04000000;
            }
            else
            {
#ifdef _DEBUG_
                fprintf(dump,"%08X ",tsPacket);
                fprintf(dump,"%s","CAFECAFE");
                fprintf(dump,"%s\n","<=CTRL");
#endif
            }
            tsPacket = bufSnapShot[eventIndex++];
        }
#ifdef _DEBUG_
        fprintf(dump,"%08X ",tsPacket);
#endif
        timestamp = (tsPacket &  0x03FFFFFF) + timestampMonotonyWrap;
        ptimestamp=timestamp;
    }
    else
    {
#ifdef _DEBUG_
        fprintf(dump,"%s","DEADDEAD ");
#endif
        timestamp=ptimestamp;
        eventIndex--;
    }
    blob = bufSnapShot[eventIndex++];
    if(blob&0x80000000)
    {
#ifdef _DEBUG_
        fprintf(dump,"%s","DEADDEAD");
        fprintf(dump,"%s\n","<=MISS");
#endif
        eventIndex--;
        res=0;
    }
    else if((blob&0xFFFF0000)>0x00010000)
    {
#ifdef _DEBUG_
        fprintf(dump,"%08X",blob);
        fprintf(dump,"%s\n","<=CORRUPTED");
#endif
        res=0;
    }
    else if((blob&0xFFFF0000)==0x00010000)
    {
#ifdef _DEBUG_
        fprintf(dump,"%08X",blob);
        fprintf(dump,"%s\n","<=SYNCH");
#endif
        blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        unmaskEvent(blob, cartX, cartY, polarity, eye);
        res=2;
    }
    else
    {
#ifdef _DEBUG_
        fprintf(dump,"%08X\n",blob);
#endif
        blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        unmaskEvent(blob, cartX, cartY, polarity, eye);
        res=1;
    }
    return res;
}

uint eventUnmaskICUBcircBuf::snapBuffer()
{
    //std::cout << "[eventUnmaskICUBcircBuf] snaptBuffer called" << std::endl;

    if(!bufInUse)
        return 0;
    mutex.wait();
    int whichBufToUse=whichBuf-1;
    mutex.post();
    if(whichBufToUse<0)
        whichBufToUse=NBBUF-1;
    //std::cout << "[eventUnmaskICUBcircBuf] snapBuffer use buffer " << whichBufToUse << std::endl;

    cBufMutex[whichBufToUse].wait();
    if(szBufs[whichBufToUse]==0)
    {
        cBufMutex[whichBufToUse].post();
        return 0;
    }
#ifdef _DEBUG_
    //fprintf(dump,"%s\n","--SNAP--");
#endif
    int remainingSz=szBufSnapShot-(4*eventIndex);
    if(szBufSnapShot>0 && remainingSz>0)
    {
#ifdef _DEBUG_
//        fprintf(dump,"--SNAP with rescue RSZ = %d--", remainingSz);
#endif
        uint *buftmp=new uint[(uint)ceil((double)remainingSz/4)];
        memcpy(buftmp, bufSnapShot+eventIndex, remainingSz);
        delete[] bufSnapShot;
        bufSnapShot = new uint[(uint)ceil((double)(szBufs[whichBufToUse]+remainingSz)/4)];
        memcpy(bufSnapShot, buftmp, remainingSz);
        delete[] buftmp;
        memcpy(bufSnapShot+(remainingSz/sizeof(uint)), buffer[whichBufToUse], szBufs[whichBufToUse]);
        szBufSnapShot=szBufs[whichBufToUse]+remainingSz;
    }
    else
    {
#ifdef _DEBUG_
//        fprintf(dump,"--SNAP without rescue RSZ = %d--", remainingSz);
#endif
        delete[] bufSnapShot;
        bufSnapShot = new uint[szBufs[whichBufToUse]/sizeof(uint)];
        memcpy(bufSnapShot, buffer[whichBufToUse], szBufs[whichBufToUse]);
        szBufSnapShot=szBufs[whichBufToUse];
    }
/*
    delete[] bufSnapShot;
    bufSnapShot = new uint[szBuffer/4];
    memcpy(bufSnapShot, buffer, szBuffer);
    szBufSnapShot=szBuffer;
*/
    nEvent=szBufSnapShot/8;
    eventCounter=0;
    eventIndex=0;

    delete[] buffer[whichBufToUse];
    buffer[whichBufToUse] =NULL;//new char[BUFFERBLOCK];
    szBufs[whichBufToUse]=0;
    szInMem=0;//BUFFERBLOCK;

    cBufMutex[whichBufToUse].post();

    //std::cout << "[eventUnmaskICUBcircBuf] leavea snapBuffer" << std::endl;

    return 1;
}

int eventUnmaskICUBcircBuf::reset()
{
/*    mutex.wait();

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
    mutex.post();*/
    return 1;
}


void eventUnmaskICUBcircBuf::saveBuffer(char *_buf, uint _sz){}

}
}

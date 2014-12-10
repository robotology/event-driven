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
#include "iCub/emorph/eventUnmaskICUB.h"

namespace emorph
{
namespace eunmask
{
eventUnmaskICUB::eventUnmaskICUB(bool _save)
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

    buffer=new u32[BUFFERBLOCK/4];
    bufSnapShot=NULL;
#ifdef _DEBUG_
    dump = fopen("/home/icub/Clercq/icubDump.txt", "w");
    //dump = fopen("/home/clercq/Documents/Temp/icubDump.txt", "w");
    if(dump==NULL)
        std::cout << "[eventUnmaskICUB] Error, file can't be opened or created" << std::endl;
#endif
}

eventUnmaskICUB::eventUnmaskICUB(const eventUnmaskICUB &_obj)
{
    objcpy(_obj);
}

eventUnmaskICUB::~eventUnmaskICUB()
{
    delete[] buffer;
#ifdef _DEBUG_
    fclose(dump);
#endif
}

eventUnmaskICUB& eventUnmaskICUB::operator=(const eventUnmaskICUB &_obj)
{
    objcpy(_obj);
    return *this;
}

void eventUnmaskICUB::objcpy(const eventUnmaskICUB &_obj)
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
        buffer = new u32[szInMem];
        memcpy(buffer, _obj.buffer, szBuffer);
    }
}

//void eventUnmaskICUB::setBuffer(char* i_buffer, uint i_sz)
void eventUnmaskICUB::setBuffer(emorph::ebuffer::eventBuffer &_buf)
{
    mutex.wait();
    //std::cout << "[eventUnmaskICUB] setBuffer called" << std::endl;
    u32  *i_buffer = (u32*)_buf.get_packet();
    uint i_sz =_buf.get_sizeOfPacket();

    if(szBuffer==0)
    {
#ifdef _DEBUG_
        //fprintf(dump,"%s\n","--SET Z--");
#endif
        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)i_sz/(double)BUFFERBLOCK)*BUFFERBLOCK;
        //delete[] buffer;
        buffer = new uint[szInMem/4];
    }
    else if(szBuffer+i_sz>=szInMem)
    {
#ifdef _DEBUG_
         //fprintf(dump,"%s\n","--SET NZ--");
#endif
        //std::cout << "\t\t\t- Expand the size of the gBuffer" << std::endl;
        //szInMem+=BUFFERBLOCK;
        szInMem=(uint)ceil((double)(i_sz+szBuffer)/(double)BUFFERBLOCK)*BUFFERBLOCK;
        u32 *buftmp=new u32[szBuffer/4];
        memcpy(buftmp, buffer, szBuffer);
        delete[] buffer;
        buffer = new u32[szInMem/4];
        memcpy(buffer, buftmp, szBuffer);
    }
    //std::cout << "\t\t\t- Concat the buffer (size in byte: " << sz+i_sz << ", real size in mem: " << szInMem << ")" << std::endl;
    //memcpy(buffer+(szBuffer/4), (uint*)i_buffer, i_sz);
    memcpy(buffer+(szBuffer/4), i_buffer, i_sz);
    szBuffer+=i_sz;
    nEvent=szBuffer/4;
    mutex.post();
}

void eventUnmaskICUB::reshapeBuffer()
{
/*
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
    mutex.post();*/
}

int eventUnmaskICUB::getUmaskedData(uint& cartX, uint& cartY, int& polarity, uint& eye, uint& timestamp)
{
    int res=0;
    //if(4*(eventIndex+3)>szBufSnapShot || szBufSnapShot==0)
    if(4*(eventIndex)>=szBufSnapShot || szBufSnapShot==0)
    //if((double)(4*(eventIndex-1))>=0.95*(double)szBufSnapShot || szBufSnapShot==0)
    //if(((int)szBufSnapShot-(4*(int)eventIndex))<2 || szBufSnapShot==0)
    {
        if(!snapBuffer())
            return 0;
    }
    tsPacket = bufSnapShot[eventIndex++];
    /*if( ((tsPacket & 0xFC000000) > 0x80000000) && ((szBufSnapShot-(4*(eventIndex)))<3) )
    {
        szBufSnapShot=0;
        return 0;
    }*/
    
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
                //timestampMonotonyWrap += 0x04000000;
                timestampMonotonyWrap += 0x1000000;//1<<24;
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
        fprintf(dump,"%08X",tsPacket);
        fprintf(dump,"%s\n","<=MISSTS");
#endif
        timestamp=ptimestamp;
        //eventIndex--;
        return 0;
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
        //unmaskEvent(blob, cartX, cartY, polarity, eye);
        cartX   = ((blob & xmask)>>xshift);
        cartY   = ((blob & ymask)>>yshift);
        polarity= (((blob & polmask)>>polshift)==0)?-1:1;   //+1 ON, -1 OFF
        eye     = ((blob & eyemask)>>eyeshift);

        res=2;
    }
    else
    {
#ifdef _DEBUG_
        fprintf(dump,"%08X\n",blob);
#endif
        blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        //unmaskEvent(blob, cartX, cartY, polarity, eye);
        cartX   = ((blob & xmask)>>xshift);
        cartY   = ((blob & ymask)>>yshift);
        polarity= (((blob & polmask)>>polshift)==0)?-1:1;   //+1 ON, -1 OFF
        eye     = ((blob & eyemask)>>eyeshift);

        res=1;
    }
    return res;
}

uint eventUnmaskICUB::snapBuffer()
{
    mutex.wait();
    if(szBuffer==0)
    {
        mutex.post();
        return 0;
    }
#ifdef _DEBUG_
    //fprintf(dump,"%s\n","--SNAP--");
#endif
/*    int remainingSz=szBufSnapShot-(4*eventIndex);
    if(szBufSnapShot>0 && remainingSz>0)
    {
#ifdef _DEBUG_
//        fprintf(dump,"--SNAP with rescue RSZ = %d--", remainingSz);
#endif
        uint *buftmp=new u32[(uint)ceil((double)remainingSz/4)];
        memcpy(buftmp, bufSnapShot+eventIndex, remainingSz);
        delete[] bufSnapShot;
        bufSnapShot = new u32[(uint)ceil((double)(szBuffer+remainingSz)/4)];
        memcpy(bufSnapShot, buftmp, remainingSz);
        delete[] buftmp;
        memcpy(bufSnapShot+(remainingSz/4), buffer, szBuffer);
        szBufSnapShot=szBuffer+remainingSz;
    }
    else
    {
#ifdef _DEBUG_
//        fprintf(dump,"--SNAP without rescue RSZ = %d--", remainingSz);
#endif
        delete[] bufSnapShot;
        bufSnapShot = new u32[szBuffer/4];
        memcpy(bufSnapShot, buffer, szBuffer);
        szBufSnapShot=szBuffer;
    }*/

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

int eventUnmaskICUB::reset()
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


void eventUnmaskICUB::saveBuffer(char *_buf, uint _sz){}

}
}

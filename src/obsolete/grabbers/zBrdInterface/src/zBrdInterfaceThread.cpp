// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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

/**
 * @file zBrdInterfaceThread.cpp
 * @brief Implementation of the thread (see header zBrdInterfaceThread.h)
 */

#include <iCub/zBrdInterfaceThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace emorph::ebuffer;

#define INTERVFACTOR 1              // resolution 160ns = 6.25Mhz, 128ns = 7.8125Mhz ... gaep 1us
#define COUNTERRATIO 1 //1.25       //1.25 is the ratio 0.160/0.128
#define MAXVALUE 0xFFFFFF //4294967295
#define THRATE 5
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
//#define retinalSize 128
#define CHUNKSIZE 32768 //65536 //8192
#define dim_window 5
#define synch_time 1

//#define VERBOSE

zBrdInterfaceThread::zBrdInterfaceThread() : RateThread(THRATE) {
    responseGradient = 127;
    retinalSize      = 128;  //default value before setting 
  
    synchronised = false;
    greaterHalf  = false;
    firstRun     = true;
    count        = 0;
    minCount     = 0; //initialisation of the timestamp limits of the first frame
    slen = sizeof(si_other);

    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
    countStop = 0;
    verb = false;
    string i_fileName("events.log");
    raw = fopen(i_fileName.c_str(), "wb");
    lc = rc = 0;	
    minCount      = 0;
    minCountRight = 0;
}

zBrdInterfaceThread::~zBrdInterfaceThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

void zBrdInterfaceThread::diep(char *s)
{
  perror(s);
}

bool zBrdInterfaceThread::threadInit() {
    sz = 0;

    sizeofstructaer  = sizeof(struct aer);
    sizeofstructatom = sizeof(struct atom);

    printf(" \nstarting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    fout = fopen("./dump.txt","w+");

    int deviceNum = 0;
    printf("opening port for sending the read events \n");
    //std::stringstream ); 
    //str_buf << "/icub/retina" << deviceNum << ":o";
    port.open("/icub/retina0:o");
    portEventBottle.open("/zBrdInterface/eventBottle:o");

    resize(retinalSize, retinalSize);
    printf("starting the converter!!!.... \n");
    
    //cfConverter=new cFrameConverter();
    //cfConverter->useCallback();
    //cfConverter->setRetinalSize(retinalSize);
    //cfConverter->open(getName("/retina:i").c_str());
    
    printf("\n opening retina\n");
    printf("starting the plotter \n");

    //minCount = cfConverter->getEldestTimeStamp();
    startTimer = Time::now();
    //clock(); //startTime ;
    //T1 = times(&start_time);
    //microseconds = 0;
    //microsecondsPrev = 0;
    //gettimeofday(&tvend, NULL);

    count = 0;
    microsecondsPrev = 0;
    minCount = 0;
    minCountRight= 0;

    printf("starting the udp connection \n");

    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
       diep("socket");
       printf("socket error! \n");
       return false;
    }
    else {
        printf("socket correctly defined \n");
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s, (const struct sockaddr *)&si_me, sizeof(si_me))==-1){
      diep("bind");
      printf("bind error \n");
      return false;
    }
    else {
        printf("bind correctly executed \n");
    }

    printf("Initialisation in collector thread correctly ended \n");
    return true;
}

void zBrdInterfaceThread::interrupt() {
 //   outPort.interrupt();
 //   outPortRight.interrupt();
}

void zBrdInterfaceThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string zBrdInterfaceThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void zBrdInterfaceThread::resize(int widthp, int heightp) {
  //  imageLeft = new ImageOf<PixelMono>;
  //  imageLeft->resize(widthp,heightp);
  //  imageRight = new ImageOf<PixelMono>;
  //  imageRight->resize(widthp,heightp);
}


//void zBrdInterfaceThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera){
//
//}

int zBrdInterfaceThread::prepareUnmasking(char* bufferCopy, Bottle* res) {
    
}

void zBrdInterfaceThread::run() {
    //printf("Reading from the UDP socket \n");

    int k2 = 0;
    char*       pBuffer = &buf[0];
    uint32_t *     buf2 = (uint32_t*)    pBuffer;
     
    
    PacketLenght=recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *)&si_other, (socklen_t*) &slen);
    //printf("read the content \n");

    if (PacketLenght==-1){
        diep("recvfrom()");
        printf("Closing the module \n");
    }
     
    //printf("Received a new packet %d bytes wide\r\n",PacketLenght);
    if (PacketLenght<1024) {    // Message Packet
        ptchar=&buf[0];
        
        printf("data %s\n",ptchar);
        fflush(stdout);
    }
    else {                      // Data Packet
        //printf("dimension of the data packet %d bytes \n", PacketLenght);
        pt=(int *)&buf[0];
        uint32_t* pt32 = (uint32_t*) &buf[0];
//        fprintf(fout,"\n###########\n");
        for (j=0;j<(PacketLenght/4);j++) {
            if(j%2==0) {
                time_received = *pt32;
                //printf("%03d) 0x%08x time ",j,*pt);
                //buf2[k2++] =  (uint32_t) *pt32; //removed this line to invert the time-stamp address bytes
                //fprintf(fout, "%08x ", *pt32);
                //printf("0x%08x \n",(uint32_t) *pt );
            }                
            else {
                //printf("%08x %08x\n", *pt, time_received);
                //printf(" 0x%08x data\n",*pt32);
                fprintf(fout, "%08x %08x\n", *pt32, time_received);
                buf2[k2++] =  (uint32_t) *pt32;
                buf2[k2++] = (uint32_t) time_received;
            }
            //pt+=1;
            pt32++;
        }
    }

    sz = k2 * sizeofstructatom;
    char* buf = (char*) buf2;

    if (port.getOutputCount()) {
        eventBuffer data2send(pBuffer, sz);  //adding 8 bytes for extra word 0xCAFECAFE and TS_WA    
        eventBuffer& tmp = port.prepare();
        tmp = data2send;
        port.write();
    }

    if (portEventBottle.getOutputCount()) {       
        printf("Sending the bottle %d bytes \n", sz);
        eventBottle data2send(pBuffer, sz);
        eventBottle& tmp = portEventBottle.prepare(); 
        tmp = data2send;
        portEventBottle.write();
    } 
          
}

void zBrdInterfaceThread::threadRelease() {
    idle = false;
    fclose(fout);
    printf("zBrdInterfaceThread release:freeing bufferCopy \n");
    //free(bufferCopy);
    printf("zBrdInterfaceThread release:closing ports \n");
  //  outPort.close();
  //  outPortRight.close();
    
    printf("correctly freed memory from the bottleHandler \n");

    port.interrupt();
    port.close();
    portEventBottle.interrupt();
    portEventBottle.close();
    
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------


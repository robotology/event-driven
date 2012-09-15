// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
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

/*
 * This class use the USB retina driver wrote by
 * Martin Ebner, IGI / TU Graz (ebner at igi.tugraz.at)
 *
 *  The term of the contract of the used source is :
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * (Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com))
 *
 */

#include <iCub/device2yarp.h>
//#include <sys/types.h>

using namespace std;
using namespace yarp::os;

#define timestep 10000 //10 ms OneSecond/100
#define OneSecond 1000000 //1sec
#define latchexpand 100
#define countBias 12
#define LATCH_KEEP 1
#define LATCH_TRANSPARENT 0

#define CLOCK_LO 0
#define CLOCK_HI 1
#define THRATE   5

#define KERNELDEVICEREAD SIZE_OF_DATA //32768


//#define FAST


//device2yarp::device2yarp(string portDeviceName, bool i_bool, string fileName):RateThread(THRATE), save(i_bool) , i_fileName(fileName){
device2yarp::device2yarp(string portDeviceName, bool i_bool, string fileName):save(i_bool) , i_fileName(fileName){
    printf("initialising the module \n");

    wrapAround=0;

    verbosity = false;
    len=0;
    sz=0;
    ec = 0;
    numberOfTimeWraps = 0;
    memset(buffer, 0, SIZE_OF_DATA);
    //const u32 seqAllocChunk_b = 8192 * sizeof(struct aer); //allocating the right dimension for biases
    
    //allocation monitor
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);
    pmon = (aer *)  malloc(monBufSize_b);
    if ( pmon == NULL ) {
        printf("pmon malloc failed \n");
    }
    
    /*
    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;
    seqEvents = 0;
    seqSize_b = 0;
    */
    
    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;

    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }
    else {
        int err;
#ifdef FAST
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x07,0xc8,	    // cas
                                0x10,0xe9,0x8C,		// injGnd
                                0xFF,0xFF,0xFF,		// reqPd
                                0x7c,0x7f,0xf5,		// puX
                                0x00,0x00,0x84,		// diffOff          ++
                                0x02,0x6d,0xab,		// req
                                0x00,0x03,0xc9,		// refr             ++ refractory of the pixels
                                0xFF,0xFF,0xFF,		// puY
                                0x03,0x34,0x4c,		// diffOn           ++
                                0x00,0x33,0x45,		// diff
                                0x00,0x01,0x0f,		// foll
                                0x00,0x01,0x0f}; 	// Pr               ++ Velocity of the "log circuit"        
#else
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x00,0x36,     // cas
                                0x10,0xe9,0x8C,     // injGnd
                                0xFF,0xFF,0xFF,     // reqPd
                                0x7c,0x7f,0xf5,     // puX
                                0x00,0x00,0x84,     // diffOff
                                0x02,0x6d,0xab,     // req
                                0x00,0x00,0x06,     // refr
                                0xFF,0xFF,0xFF,     // puY
                                0x07,0x5c,0x8b,     // diffOn
                                0x00,0x75,0xc9,     // diff
                                0x00,0x00,0x33,     // foll
                                0x00,0x00,0x03      // Pr
                                };
        

#endif //FAST
        err = write(file_desc,bias,41); //5+36
        //int err = write(file_desc,bias,41); //5+36
        cout << "Return of the bias writing : " << err << endl;
        unsigned char start[5];
        start[0] = 0xb3;
        start[1] = 0;
        start[2] = 0;
        start[3] = 0;
        start[4] = 0;
        err = write(file_desc,start,5);
        cout << "Return of the start writing : " << err << endl;
    }
    
    

    
}

device2yarp::~device2yarp() {
    //if(save)
    //    fclose(raw);

    //port.close();


}

void device2yarp::threadRelease() {
    if(save) {
        fclose(raw);
    }
//    fclose(tmpFile);
    port.interrupt();
    port.close();

    unsigned char stop[5];
    stop[0] = 0xb4;
    stop[1] = 0;
    stop[2] = 0;
    stop[3] = 0;
    stop[4] = 0;
    //err = write(file_desc,stop,5);
    printf("%d address events read\n",len/4);
    close(file_desc);

}

bool device2yarp::threadInit() {
    if(save) {
        printf("Opening the file to dump down all the events \n");
        //raw = fopen(i_fileName.c_str(), "wb");
        raw = fopen(i_fileName.c_str(), "w");
	if(raw==NULL)
	    printf("%s\n", "Error: file not found or can't be created");
    }
    else {
        printf("Saving option not activated \n");
        
    }
//    tmpFile = fopen("log.txt","wb");
    //int deviceNum = 1;
    str_buf << "/icub/retina" << deviceNum << ":o";
    port.open(str_buf.str().c_str());
    packetsSent.clear();
    
    return true;
}


void device2yarp::setDeviceName(string deviceName) {
    printf("saving portDevice \n");
    portDeviceName=deviceName;
}



void  device2yarp::run() {
    // read address events from device file which is not /dev/aerfx2_0
    memset(buffer, 0, SIZE_OF_DATA);
    wrapOccured=0;
    ptimestamp=0;
    uint32_t * buf32 = new uint32_t[SIZE_OF_DATA];
    memset(buf32, 0, SIZE_OF_DATA*sizeof(uint32_t));

    while(1)
    {
        sz = read(file_desc,buffer,KERNELDEVICEREAD);
        //sz = pread(file_desc,buffer,KERNELDEVICEREAD, 0);
        int numberofWords = 0;
	//memset(outbuffer, 0, SIZE_OF_DATA);
        //uint32_t * buf32 = (uint32_t*) outbuffer;

        //uint32_t * buf32 = new uint32_t[SIZE_OF_DATA];
	//memset(buf32, 0, SIZE_OF_DATA*sizeof(uint32_t));

#ifdef _DEBUG_
        cout << sz <<"  bytes"<<endl;    
#endif
    //    Bottle& newBuffer = port.prepare();
        
        for (int i = 0 ; i < sz ; i+=4) {


	    //printf("[dvsGrabber] Read buffer at %d over %d\n", i, sz);
            u32 adrs, ts, tag;
            unsigned int part_1 = 0xFF & *(buffer+i);    //extracting the 1 byte        
            unsigned int part_2 = 0xFF & *(buffer+i+1);  //extracting the 2 byte        
            unsigned int part_3 = 0xFF & *(buffer+i+2);  //extracting the 3 byte
            unsigned int part_4 = 0xFF & *(buffer+i+3);  //extracting the 4 byte
            if(part_3 == 0 && part_4 == 0 && part_1 == 0 && part_2 == 0){
                continue;
            }
        
            adrs = 0x0000FFFF & (part_1|part_2<<8);

/*            ts = 0x80000000;
            if (part_4 >= 0xB0  ) {
               numberOfTimeWraps++;                     

               if (numberOfTimeWraps >= 1<<12){
                  ts = 0x88000000;
                  numberOfTimeWraps = 0;
                  buf32[numberofWords++] = ts; 
                  if (save){ 
                     fprintf(raw,"%08X\n",ts);  
                  }
                  continue;
               }
            }
            else 
            {
               ts = ts | ((part_3)|(part_4<<8));                
            }             
            ts = numberOfTimeWraps<<14 | ts; 
*/
	    timestamp=0x0000FFFF&( part_3|(part_4<<8));
		    //if( (part_4&0xff)==0x80 )
	    if( timestamp==0x8000 )
	    {
#ifdef _DEBUG_
		printf("%s\n","[dvsGrabber] Increment wrap around");
#endif
		wrapOccured=1;
		wrapAround+=0x4000;
		if(save)
		    //fprintf(raw,"%s\n","--SUBWRAP--");  
		    fprintf(raw,"%08X \t CAFECAFE \t CAFECAFE <= SUBWRAP\n", timestamp);  
	    }
	    //else if( (part_4&0xf0)==0x40 )
	    else if( timestamp==0x4000 )
	    {
#ifdef _DEBUG_
		printf("%s\n","[dvsGrabber] Reset wrap around");
#endif
		wrapAround=0;
		if(save)
		    fprintf(raw,"%08X \t EFACEFAC \t EFACEFAC <= RESET\n", timestamp);  
	    }
            else
            {
                //if(wrapAround>=(1<<24))
		if(timestamp<ptimestamp && !wrapOccured)
			wrapAround+=0x4000;
		wrapOccured=0;
                if(wrapAround>=0x1000000)
                {
                    wrapAround=0;
                    buf32[numberofWords++] = 0x88000000;
                    if (save)
		    {
                        //fprintf(raw,"%s\n","--WRAP--");  
                        fprintf(raw,"00000000 \t 88000000 \t 00000000");  
		    }
                }
                ts=0x80000000|((0x3fff&timestamp)+wrapAround);
		ptimestamp=timestamp;


        //        if(numberOfTimeWraps >= 1<<12){
        //            ts = 0x88000000;
        //            numberOfTimeWraps = 0;
        //            buf32[numberofWords++] = ts; 
        //            if (save){ 
        //               fprintf(raw,"%08X\n",ts);  
        //            }
        //            continue;
        //        }
        //        else {
        //            ts = 0x80000000;
        //            if(part_4 >= 0xB0  ) {
        //                    numberOfTimeWraps++;      
        //                    
        //                }
        //            else 
        //            {
        //                ts = ts | ((part_3)|(part_4<<8));                
        //            }             
        //            ts = numberOfTimeWraps<<14 | ts; 
        //        }     
                 


                //newBuffer[countRealEvents] = ts;
                //newBuffer[countRealEvents+1] = adrs;
                //newBuffer.addInt(ts);
                //newBuffer.addInt(adrs);
                
                buf32[numberofWords++] = ts; 
                buf32[numberofWords++] = adrs; 

        //        fprintf(tmpFile,"%08X \t %08X \t\t %u\n",ts,adrs,numberOfTimeWraps);  

                if (save){
                   //fprintf(raw,"%08X \t %08X \r\n",ts,adrs);  
                   fprintf(raw,"%08X \t %08X \t %08X\n", 0x3fff&timestamp, ts,adrs);  
                }
            }
        }
        
        /*
        if(port.getOutputCount()) {
            port.write();        
        }
        */

        if(port.getOutputCount()) {
#ifdef _DEBUG_
            printf("%s\n", "[dvsGrabber] Send buf32");
#endif
            //emorph::ebuffer::eventBuffer data2send(outbuffer, sz);  //adding 8 bytes for extra word 0xCAFECAFE and TS_WA    
            //emorph::ebuffer::eventBuffer data2send(buffer, sz);  //adding 8 bytes for extra word 0xCAFECAFE and TS_WA    

            //emorph::ebuffer::eventBuffer data2send((char*)buf32, sz*sizeof(uint32_t));    
            //emorph::ebuffer::eventBuffer data2send((char*)buf32, sz*2);    
            unsigned int sz2s=numberofWords<=0?0:(numberofWords-1)*sizeof(uint32_t);
            printf("[device2yarp] Send %d bytes\n", sz2s);
            emorph::ebuffer::eventBuffer data2send((char*)buf32, sz2s);
            emorph::ebuffer::eventBuffer& tmp = port.prepare();
            tmp = data2send;
            port.write();
/*            int npart=(int)ceil((double)sz2s/32768.0);
            emorph::ebuffer::eventBuffer data2send;
            for(int ii=0;ii<npart;++ii)
            {
                if(sz2s>32768)
                    data2send.set_data((char*)(buf32+ii*8192), 32768);
                else
                    data2send.set_data((char*)(buf32+ii*8192), sz2s);
                sz2s-=32768;
                emorph::ebuffer::eventBuffer& tmp = port.prepare();
                tmp = data2send;
                port.write();
            }*/
        }
        //resetting buffers    
        memset(buffer, 0, SIZE_OF_DATA);
    }
    delete[] buf32;
}


/*
void device2yarp::sendingBias() {
    printf("-------------------------------------------- \n");
    printf("trying to write to kernel driver %d %d \n", seqDone_b,seqSize_b);
    while (seqDone_b < seqSize_b) {      
        // try writing to kernel driver 
        printf( "calling write fd: sS: %d  sD: %d \n", seqSize_b, seqDone_b);
        int w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);
        if (w > 0) {
           seqDone_b += w;
           printf("writing accumulator %d \n",seqDone_b);
        } else {
           printf("writing error \n");   
        }
    }
    printf("writing successfully ended \n");

    ////////////////////////////////////////////////////////////


    double TmaxSeqTimeEstimate = 
        seqTime * 0.128 * 1.10 +   // summed up seq intervals in us plus 10%
        seqEvents * 1.0           // plus one us per Event
    ;

    printf("seqEvents: %d \n", seqEvents);
    printf("seqTime * 0.128: %d \n", (u64)(seqTime * 0.128));
    //printf("TmaxSeqTimeEstima PRIu64  %f %f\n", 
    //  (TmaxSeqTimeEstimate / 1000000), (TmaxSeqTimeEstimate % 1000000));
}
*/

/*
void device2yarp::progBias(string name,int bits,int value) {
    int bitvalue;
    for (int i=bits-1;i>=0;i--) {
        int mask=1;
        for (int j=0; j<i; j++) {
            mask*=2;
        }
        printf("mask %d ",mask);
        if (mask & value) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        printf("---------------------------- %d \n",bitvalue);
        progBit(bitvalue);
    }
}
*/

/*
void device2yarp::latchCommit() {
    printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_LO, 0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0);
    printf("exiting latch_commit \n");
}
*/

/*
void device2yarp::progBit(int bitvalue) {
    //set data
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue);
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue);
}
*/

/*
void device2yarp::monitor (int secs) {
    printf("entering monitor \n");
    biasprogtx(secs * OneSecond, LATCH_KEEP, CLOCK_LO, 0);
    printf("exiting monitor \n");
} 
*/

/*
void device2yarp::biasprogtx(int time,int latch,int clock,int data) {
    unsigned char addr[4];
    unsigned char t[4];
    int err;
    //setting the time
    //printf("biasProgramming following time %d \n",time);
    t[0]= time & 0xFF000000;
    t[1]= time & 0x00FF0000;
    t[2]= time & 0x0000FF00;
    t[3]= time & 0x000000FF;
   
    
    //setting the addr
    addr[0] = 0xFF;
    addr[1] = 0x00;
    addr[2] = 0x00;
    if(data) {
        addr[3] += 0x01;
    }
    if(clock) {
        addr[3] += 0x02;
    }
    if (latch) {
        addr[3] += 0x04;
    }
    //printf("data:0x%x, 0x%x, 0x%x, 0x%x \n",addr[0],addr[1],addr[2],addr[3]);
    
    
    //u32 seqSize_b = sizeof(struct aer);
    u32 timeToSend, addressToSend;
    timeToSend=time;
    addressToSend=0;
    if(data) {
        addressToSend += 0x01;
    }
    if(clock) {
        addressToSend += 0x02;
    }
    if (latch) {
        addressToSend += 0x04;
    }
    addressToSend+=0xFF000000;

    u32 hwival = (u32)(timeToSend * 7.8125);
    //printf("saving the aer.address %x \n", addressToSend);
    pseq[seqEvents].address = addressToSend;
    //printf("saving the aer.time  \n");
    pseq[seqEvents].timestamp = hwival;

    seqEvents++;
    printf("number of saved events %d as tot %d ",seqEvents,seqSize_b);
    seqTime += hwival;
    seqSize_b += sizeof(struct aer);
    

    //printf("writing the event");
    //int w = write(file_desc, ((u8*)pseq), seqSize_b);
    //addr = int(sys.argv[1], 16)
    //err = write(file_desc,t,4); //4 byte time: 1 integer
    //err = write(file_desc,addr,4); //4 byte time: 1 integer  
}
*/

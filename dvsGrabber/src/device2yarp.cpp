// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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



device2yarp::device2yarp(string portDeviceName, bool i_bool, string i_fileName):RateThread(10), save(i_bool) {
    len=0;
    sz=0;
    memset(buffer, 0, SIZE_OF_DATA);
    const u32 seqAllocChunk_b = 8192 * sizeof(struct aer); //allocating the right dimension for biases
    

    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;

    seqEvents = 0;
    seqSize_b = 0;

    if(save)
        raw = fopen(i_fileName.c_str(), "wb");
    int deviceNum=0;
    str_buf << "/icub/retina" << deviceNum << ":o";
    port.open(str_buf.str().c_str());

    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }
    else {
        int err;
        if (!strcmp(portDeviceName.c_str(),"/dev/retina0")) {
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
        err = write(file_desc,bias,41); //5+36
#else
#ifdef SLOW
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
        err = write(file_desc,bias,41); //5+36
#else
//        unsigned char bias[] = {0xb8,               //request
//                                0x00, 0x00,         //value
//                                0x00, 0x00,          //index
//                                0x00,0x07,0xc8,	    // cas
//								0x10,0xe9,0x8C,		// injGnd
//								0xFF,0xFF,0xFF,		// reqPd
//								0x7c,0x7f,0xf5,		// puX
//								0x00,0x01,0xbe,		// diffOff
//								0x04,0xb9,0x56,		// req
//								0x00,0x09,0x31,		// refr
//								0xFF,0xFF,0xFF,		// puY
//								0x01,0xa5,0xae,		// diffOn
//								0x00,0x1d,0x72,		// diff
//								0x00,0x01,0x10,		// foll
//								0x00,0x07,0x5d}; 	// Pr
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x07,0xc8,     // cas
                                0x10,0xe9,0x8C,     // injGnd
                                0xFF,0xFF,0xFF,     // reqPd
                                0x7c,0x7f,0xf5,     // puX
                                0x00,0x04,0xfe,     // diffOff
                                0x04,0xb9,0x56,     // req
                                0x00,0x09,0x31,     // refr
                                0xFF,0xFF,0xFF,     // puY
                                0x01,0xe5,0x20,     // diffOn
                                0x00,0x26,0xd2,     // diff
                                0x00,0x01,0x10,     // foll
                                0x00,0x09,0x31      // Pr
                                };
        err = write(file_desc,bias,41); //5+36
#endif //SLOW
#endif //FAST
        }
        
        if(!strcmp(portDeviceName.c_str(),"/dev/aerfx2_0")) {
            printf("sending biases as events to the device ... \n");
            /*
            *    biasvalues = {
            *    "11" "cas": 1966, 7AE
            *    "10" "injGnd": 22703, 58AF
            *    "9" "reqPd": 16777215, FFFFFF
            *    "puX": 4368853, 42A9D5
            *    "diffOff": 3207, C87
            *    "req": 111347, 1B2F3
            *    "refr": 0, 0
            *    "puY": 16777215, FFFFFF
            *    "diffOn": 483231, 75F9F
            *    "diff": 28995, 7143
            *    "foll": 19, 13
            *    "Pr": 8, 8
            *}
            */

            int biasValues[]={1966,        // cas
                              22703,       // injGnd
                              16777215,    // reqPd
                              4368853,     // puX
                              3207,        // diffOff
                              111347,      // req
                              0,           // refr
                              16777215,    // puY
                              483231,      // diffOn
                              28995,       // diff 
                              19,          // foll
                              8            //Pr 
            };

            string biasNames[] = {
                                "cas",
                                "injGnd",
                                "reqPd",
                                "puX",
                                "diffOff",
                                "req",
                                "refr",
                                "puY",
                                "diffOn",
                                "diff",
                                "foll",
                                "Pr"
                                };
            /*unsigned char biasValues[] = {
                                0x00,0x07,0xAE,     // cas
                                0x00,0x58,0xAF,     // injGnd
                                0xFF,0xFF,0xFF,     // reqPd
                                0x42,0xA9,0xD5,     // puX
                                0x00,0x0C,0x87,     // diffOff
                                0x01,0xB2,0xF3,     // req
                                0x00,0x00,0x00,     // refr
                                0xFF,0xFF,0xFF,     // puY
                                0x07,0x5F,0x9F,     // diffOn
                                0x00,0x71,0x43,     // diff
                                0x00,0x00,0x13,     // foll
                                0x00,0x00,0x08      // Pr
                                };
            */

            
            //int err = write(file_desc,bias,41); //5+36 
            seqEvents = 0;
            seqSize_b = 0;
            for(int j=0;j<countBias;j++) {
                progBias(biasNames[j],24,biasValues[j]);
            }
            latchCommit();
            monitor(10);
            sendingBias();
        }

        //int err = write(file_desc,bias,41); //5+36
        cout << "Return of the bias writing : " << err << endl;
        unsigned char start[5];
        start[0] = 0xb3;
        start[1] = 0;
        start[2] = 0;
        start[3] = 0;
        start[4] = 0;
        //err = write(file_desc,start,5);
        cout << "Return of the start writing : " << err << endl;
    }
}

device2yarp::~device2yarp() {
    if(save)
        fclose(raw);

    port.close();

    unsigned char stop[5];
    stop[0] = 0xb4;
    stop[1] = 0;
    stop[2] = 0;
    stop[3] = 0;
    stop[4] = 0;
    err = write(file_desc,stop,5);
    printf("%d address events read\n",len/4);
    close(file_desc);
}


void device2yarp::setDeviceName(string deviceName) {
    portDeviceName=deviceName;
}



void  device2yarp::run() {
    // read address events from device file 
    //sz = read(file_desc,buffer,SIZE_OF_DATA);
    sz = pread(file_desc,buffer,SIZE_OF_DATA,0);
    cout << "Size of the buffer : " << sz << endl;
    for(int i=0; i<sz; i+=4)
    {
            unsigned int part_1 = 0x00FF&buffer[i];
            unsigned int part_2 = 0x00FF&buffer[i+1];
            unsigned int part_3 = 0x00FF&buffer[i+2];
            unsigned int part_4 = 0x00FF&buffer[i+3];
            unsigned int blob = (part_1)|(part_2<<8);
            unsigned int timestamp = ((part_3)|(part_4<<8));
            //printf("%x : %x\n", blob, timestamp);
    }
    sendingBuffer data2send(buffer, sz);
    sendingBuffer& tmp = port.prepare();
    tmp = data2send;
    port.write();
    if(save)
    {
        fwrite(&sz, sizeof(int), 1, raw);
        fwrite(buffer, 1, sz, raw);
    }
    memset(buffer, 0, SIZE_OF_DATA);
}


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


    ////////////////////////////////////////////////////////////

}

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

void device2yarp::latchCommit() {
    printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_LO, 0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0);
    printf("exiting latch_commit \n");
}

void device2yarp::progBit(int bitvalue) {
    //set data
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue);
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue);
}

void device2yarp::monitor (int secs) {
    printf("entering monitor \n");
    biasprogtx(secs * OneSecond, LATCH_KEEP, CLOCK_LO, 0);
    printf("exiting monitor \n");
} 

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

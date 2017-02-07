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

#include <device2yarp.h>
//#include <sys/types.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>

#include <sys/types.h>
#include <inttypes.h>
//#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <sys/time.h>
#include <sched.h>

using namespace std;
using namespace yarp::os;

extern int errno;
//EAGAIN		11	/* Resource temporarily unavailable */



/*
#################################################################
# time constants

OneSecond = 1000000 # ???

# genova visit timings:
timestep  = OneSecond // 10000
latchexpand = 100

# new faster timings, timestep 1us, latch 10 timesteps
timestep  = OneSecond // 1000000
latchexpand = 10

# ultra slow
timestep  = OneSecond // 100
latchexpand = 100

# very slow 1ms timestep
timestep  = OneSecond // 1000
latchexpand = 100

# 10us timestep, 100us latch
timestep  = OneSecond // 100000
latchexpand = 8
reset_pins_expand = 4
#################################################################
*/

#define INPUT_BIN_U32U32LE

#define reset_pins_expand  4
#define timestep 1000 //10 us OneSecond/1000
#define OneSecond 1000000 //1sec
#define latchexpand 100
#define powerdownexpand 100
#define countBias 12
#define LATCH_KEEP 1
#define LATCH_TRANSPARENT 0

#define CLOCK_LO 0
#define CLOCK_HI 1
#define THRATE 1


static const char* byte_to_binary( int x ) {
    static char b[27];
    b[0] = '\0';
    int z;

    for (z =67108864 ; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

device2yarp::device2yarp(string portDeviceName, bool i_bool, string i_fileName = " "):RateThread(THRATE) {
    printf("device2yarp initialization .... \n");

    bottle2send = new Bottle();
    // initialization of the paramenters
    //countCycle       = 0;
    t_prev           = 0;
    fout             = 0;
    fbottle          = 0;
    countErrors      = 0;
    countInWraps     = 0;
    countInWrapsTS   = 0;
    countErrorsGAEP1 = 0;
    countErrorsGAEP2 = 0;
    countErrorsGAEP3 = 0;
    minCountInWraps  = 16777215;
    maxCountInWraps  = 0;
    countLostAE      = 0;
    wrapOccured      = false;
    onlyLeft         = false;

    associatedTimestamp = 0;
    timestampAvailable = false;


    /*   ORIGINAL VALUES
    *   from DVS128_PAER.xml, set Tmpdiff128
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



    /**************************************************************************
#define FAST
#ifdef FAST
    //int biasValues[]={1966,        // cas
      //1137667,       // injGnd
      //16777215,      // reqPd
      //8053457,       // puX
      //133,           // diffOff
      //160712,        // req
      //944,           // refr
      //16777215,      // puY
      //205255,        // diffOn
      //3207,          // diff
      //278,           // foll
      //217            // Pr
      //};



    printf("valus from DVS128Fast.xml \n");
    // from DVS128Fast.xml, set Tmpdiff128
    cas = 52458;        // cas
    injg = 101508;      // injGnd
    reqPd = 16777215;   // reqPd
    pux = 8053457;      // puX
    diffoff = 133;      // diffOff
    req = 160712;       // req
    refr = 944;         // refr
    puy = 16777215;     // puY
    diffon = 639172;    // diffOn
    diff = 30108;       // diff
    foll = 20;          // foll
    pr= 8;              // Pr

    casRight = 52458;        // cas
    injgRight = 101508;      // injGnd
    reqPdRight = 16777215;   // reqPd
    puxRight = 8053457;      // puX
    diffoffRight = 133;      // diffOff
    reqRight = 160712;       // req
    refrRight = 944;         // refr
    puyRight = 16777215;     // puY
    diffonRight = 639172;    // diffOn
    diffRight = 30108;       // diff
    follRight = 20;          // foll
    prRight = 8;             // Pr


#else

    printf("valus from DVS128_PAER.xml \n");
    cas = 1966;         // cas
    injg = 22703;       // injGnd
    reqPd = 16777215;   // reqPd
    pux = 4368853;      // puX
    diffoff = 3207;     // diffOff
    req = 111347;       // req
    refr = 0;           // refr
    puy = 16777215;     // puY
    diffon = 483231;    // diffOn
    diff = 28995;       // diff
    foll = 19;          // foll
    pr = 8;             // Pr

    casRight = 1966;        // cas
    injgRight = 22703;      // injGnd
    reqPdRight = 16777215;  // reqPd
    puxRight = 4368853;     // puX
    diffoffRight = 3207;    // diffOff
    reqRight = 111347;      // req
    refrRight = 0;          // refr
    puyRight = 16777215;    // puY
    diffonRight = 483231;   // diffOn
    diffRight = 28995;      // diff
    follRight = 19;         // foll
    prRight = 8;            // Pr


    // int biasValues[]={1966,        // cas
    // 22703,       // injGnd
    // 16777215,    // reqPd
    // 4368853,     // puX
    // 3207,        // diffOff
    // 111347,      // req
    // 0,           // refr
    // 16777215,    // puY
    // 483231,      // diffOn
    // 28995,       // diff
    // 19,          // foll
    // 8            // Pr
    // };

#endif
***************************************************************/




#define WIEN
#ifdef WIEN
    printf("values from WIEN \n");
    cas = 52458;         // cas
    injg = 101508;       // injGnd
    reqPd = 16777215;    // reqPd
    pux = 8053457;       // puX
    diffoff = 133;       // diffOff
    req = 160712;        // req
    refr = 944;          // refr
    puy = 16777215;      // puY
    diffon = 639172;     // diffOn
    diff = 30108;        // diff
    foll = 20;           // foll
    pr = 5;              // Pr


    if(onlyLeft) {
        casRight = 0;         // cas
        diffoffRight = 0;     // diffOff
        refrRight = 0;        // refr
        diffonRight = 0;      // diffOn
        diffRight = 0;        // diff
        prRight = 0;          // Pr
    }
    else {
        casRight = 52458;
        diffoffRight = 133;
        refrRight = 944;
        diffonRight = 639172;
        diffRight = 30108;
        prRight = 5;

    }
    injgRight = 101508;       // injGnd
    reqPdRight = 16777215;    // reqPd
    puxRight = 8053457;       // puX
    reqRight = 160712;        // req
    puyRight = 16777215;      // puY
    follRight = 20;           // foll



#endif

    save = false;
    sync = false;
    // passing the parameter to the class variable
    this->portDeviceName = portDeviceName;
    this->biasFileName   = i_fileName;

    //initialisation of the module
    countAEs = 0;
    countData = 0;
    len=0;
    sz=0;
    ec = 0;
    //u64 ec = 0;

    memset(buffer, 0, SIZE_OF_DATA);
    // 8192 number of max event that can be read
    //SIZE_OF_DATA is SIZE_OF_EVENT * 8bytes
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);
    monBufSize_a = SIZE_OF_EVENT * 2 * sizeof(struct atom);

    // pmon reads events from the device
    pmon = (aer *)  malloc(monBufSize_b);
    if ( pmon == NULL ) {
        printf("pmon malloc failed \n");
    }
    pmonatom = (atom *) malloc(monBufSize_a);
    if ( pmonatom == NULL ) {
        printf("pmon atom  malloc failed \n");
    }

    // opening the port for sending out events
    int deviceNum=0;
    printf("opening port for sending the read events \n");
    str_buf << "/icub/retina" << deviceNum << ":o";
    //port.open(str_buf.str().c_str());
    //portEventBottle.open("/aexGrabber/eventBottle:o");
    portDimension.open("/aexGrabber/dim:o");
    portvBottle.open("/aexGrabber/vBottle:o");

    // opening the file when the biases are programmed by file
    biasFromBinary = i_bool;
    // preparing and sending biases
    prepareBiases();
    prepareBiasesRight();
    startInt=Time::now();

    //std::cout << "Closing file descriptor after biases set" << std::endl;
    //close(file_desc);
    //std::cout << "Done" << std::endl;

}


device2yarp::~device2yarp() {

//    port.close();
//    portEventBottle.close();
//    portDimension.close();
//    portvBottle.close();

}

void device2yarp::prepareBiases() {
    //opening the device
    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK | O_SYNC );
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }

    //initialisation
    const u32 seqAllocChunk_b = SIZE_OF_EVENT * sizeof(struct aer); //allocating the right dimension for biases
    int r, w;
    struct timeval tv;
    u64  Tseqstart, Tnow;
    //u64 TmaxSeqTimeEstimate;
    u32 ival, addr, hwival;
    //int aexfd;
    int busy;

#ifdef INPUT_BIN_U32U32LE
    u32 rbuf[2];
#endif

    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;

    seqEvents = 0;
    seqSize_b = 0;


    //preparing the biases
    if(biasFromBinary) {
        printf("sending biases read from the binary file \n");
        //opening the file
        binInput = fopen(biasFileName.c_str(),"r");
        if (binInput == NULL) {
            fputs ("File error",stderr);
            return;
        }
        else {
            printf("File correctly opened \n");
        }
        while (1) {
            if (seqAlloced_b < seqSize_b) {
                //fprintf(stderr, "code error 1234: %" PRIu32 " < %" PRIu32 "\n", seqAlloced_b, seqSize_b);
                yarp::os::exit(1);
            }
            // realloc needed?
            if (seqAlloced_b == seqSize_b) {
                seqAlloced_b += seqAllocChunk_b;
                pseq = (aer*)realloc(pseq, seqAlloced_b);
                if (pseq == NULL) printf("realloc failed:");
            }

#ifndef INPUT_BIN_U32U32LE
            r = fscanf(binInput, "%" PRIu32 " %" PRIu32, &ival, &addr);

            if (r == EOF) {
                fprintf(stderr, "parsing input completed.\n");
                fclose(ifd);
                break;
            }
            if (r != 2) {
                fprintf(stderr, "input parsing error!!!\n");
                yarp::os::exit(1); // FIXME
            }
#else
            r = fread(rbuf, 8, 1, binInput);
            //printf("reading from file %d \n",r);

            if (r == 0) {
                if (feof(binInput)) {
                    fprintf(stderr, "parsing input completed.\n");
                    fclose(binInput);
                    break;
                } else {
                    fprintf(stderr, "input parsing error!!!\n");
                    perror("errno");
                    yarp::os::exit(1);
                }
            }
            ival = rbuf[0];
            addr = rbuf[1];
#endif

            /* timestamp expressed in <ts> * 1e-6 / 128e-9, with <ts> in microseconds */
            hwival = (u32)(ival * 7.8125);
            pseq[seqEvents].address = addr;
            pseq[seqEvents].timestamp = hwival;

            seqEvents++;
            seqTime += hwival;
            seqSize_b += sizeof(struct aer);


            assert(seqEvents * sizeof(struct aer) == seqSize_b);
        } //end of the while

        /* save start of sequencing time */
        gettimeofday(&tv, NULL);
        Tnow = ((u64)tv.tv_sec) * 1000000 + ((u64)tv.tv_usec);
        Tseqstart = Tnow;
        seqDone_b = 0;

        /* try writing to kernel driver */
        if (seqDone_b < seqSize_b) {
            //fprintf(stderr, "calling write fd: %d  sS: %d  sD: %d  ps: %x\n", aexfd, seqSize_b, seqDone_b, (u32)pseq);


            w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);

            //fprintf(stderr, "wrote: %d\n", w);
            if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                // we were not busy, maybe someone else is...
            } else {
                perror("invalid write");
                yarp::os::exit(1);
            }
        }
        //closing the file where the biases are set
        //if(biasFromBinary){
        //    printf("closing the file where the biases are saved \n");
        //    fclose(binInput);
        //}
    }
    else {
        printf("sending biases as following variables.... \n");
        printf("cas:%d \n",cas);
        printf("injg:%d \n",injg);
        printf("reqPd:%d \n",reqPd);
        printf("pux:%d \n",pux);
        printf("diffoff:%d \n",diffoff);
        printf("req:%d \n",req);
        printf("refr:%d \n",refr);
        printf("puy:%d \n",puy);
        printf("diffon:%d \n",diffon);
        printf("diff:%d \n",diff);
        printf("foll:%d \n",foll);
        printf("pr:%d \n",pr);
        //int err;

        printf("sending biases as events to the device ... \n");


        int biasValues[]={cas,        // cas
                          injg,       // injGnd
                          reqPd,      // reqPd
                          pux,        // puX
                          diffoff,    // diffOff
                          req,        // req
                          refr,       // refr
                          puy,        // puY
                          diffon,     // diffOn
                          diff,       // diff
                          foll,       // foll
                          pr          // Pr
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


        //int err = write(file_desc,bias,41); //5+36
        seqEvents = 0;
        seqSize_b = 0;
        for(int j=0;j<countBias;j++) {
            progBias(biasNames[j],24,biasValues[j],1);
        }
        latchCommitAEs(1);
        //monitor(10);
        releasePowerdown(1);
        sendingBias();
    }
}

void device2yarp::prepareBiasesRight() {
    //opening the device
    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }

    //initialisation
    const u32 seqAllocChunk_b = SIZE_OF_EVENT * sizeof(struct aer); //allocating the right dimension for biases
    int r, w;
    struct timeval tv;
    u64 Tseqstart, Tnow;
    //u64 TmaxSeqTimeEstimate;
    u32 ival, addr, hwival;
    //int aexfd;
    int busy;

#ifdef INPUT_BIN_U32U32LE
    u32 rbuf[2];
#endif

    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;

    seqEvents = 0;
    seqSize_b = 0;


    //preparing the biases
    if(biasFromBinary) {
        printf("sending biases read from the binary file \n");
        //opening the file
        binInput = fopen(biasFileName.c_str(),"r");
        if (binInput == NULL) {
            fputs ("File error",stderr);
            return;
        }
        else {
            printf("File correctly opened \n");
        }
        while (1) {
            if (seqAlloced_b < seqSize_b) {
                //fprintf(stderr, "code error 1234: %" PRIu32 " < %" PRIu32 "\n", seqAlloced_b, seqSize_b);
                yarp::os::exit(1);
            }
            // realloc needed?
            if (seqAlloced_b == seqSize_b) {
                seqAlloced_b += seqAllocChunk_b;
                pseq = (aer*)realloc(pseq, seqAlloced_b);
                if (pseq == NULL) printf("realloc failed:");
            }

#ifndef INPUT_BIN_U32U32LE
            r = fscanf(binInput, "%" PRIu32 " %" PRIu32, &ival, &addr);

            if (r == EOF) {
                fprintf(stderr, "parsing input completed.\n");
                fclose(ifd);
                break;
            }
            if (r != 2) {
                fprintf(stderr, "input parsing error!!!\n");
                yarp::os::exit(1); // FIXME
            }
#else
            r = fread(rbuf, 8, 1, binInput);
            //printf("reading from file %d \n",r);

            if (r == 0) {
                if (feof(binInput)) {
                    fprintf(stderr, "parsing input completed.\n");
                    fclose(binInput);
                    break;
                } else {
                    fprintf(stderr, "input parsing error!!!\n");
                    perror("errno");
                    yarp::os::exit(1);
                }
            }
            ival = rbuf[0];
            addr = rbuf[1];
#endif

            /* timestamp expressed in <ts> * 1e-6 / 128e-9, with <ts> in microseconds */
            hwival = (u32)(ival * 7.8125);
            pseq[seqEvents].address = addr;
            pseq[seqEvents].timestamp = hwival;

            seqEvents++;
            seqTime += hwival;
            seqSize_b += sizeof(struct aer);


            assert(seqEvents * sizeof(struct aer) == seqSize_b);
        } //end of the while

        /* save start of sequencing time */
        gettimeofday(&tv, NULL);
        Tnow = ((u64)tv.tv_sec) * 1000000 + ((u64)tv.tv_usec);
        Tseqstart = Tnow;
        seqDone_b = 0;

        /* try writing to kernel driver */
        if (seqDone_b < seqSize_b) {
            //fprintf(stderr, "calling write fd: %d  sS: %d  sD: %d  ps: %x\n", aexfd, seqSize_b, seqDone_b, (u32)pseq);


            w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);

            //fprintf(stderr, "wrote: %d\n", w);
            if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                // we were not busy, maybe someone else is...
            } else {
                perror("invalid write");
                yarp::os::exit(1);
            }
        }
        //closing the file where the biases are set
        //if(biasFromBinary){
        //    printf("closing the file where the biases are saved \n");
        //    fclose(binInput);
        //}
    }
    else {
        printf("sending biases as following variables.... to the RIGHT \n");
        printf("cas:%d \n",casRight);
        printf("injg:%d \n",injgRight);
        printf("reqPd:%d \n",reqPdRight);
        printf("pux:%d \n",puxRight);
        printf("diffoff:%d \n",diffoffRight);
        printf("req:%d \n",reqRight);
        printf("refr:%d \n",refrRight);
        printf("puy:%d \n",puyRight);
        printf("diffon:%d \n",diffonRight);
        printf("diff:%d \n",diffRight);
        printf("foll:%d \n",follRight);
        printf("pr:%d \n",prRight);
        //int err;

        printf("sending biases as events to the device ... \n");


        int biasValues[]={casRight,        // cas
                          injgRight,       // injGnd
                          reqPdRight,      // reqPd
                          puxRight,        // puX
                          diffoffRight,    // diffOff
                          reqRight,        // req
                          refrRight,       // refr
                          puyRight,        // puY
                          diffonRight,     // diffOn
                          diffRight,       // diff
                          follRight,       // foll
                          prRight           //Pr
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


        //int err = write(file_desc,bias,41); //5+36
        seqEvents = 0;
        seqSize_b = 0;
        for(int j=0;j<countBias;j++) {
            progBias(biasNames[j],24,biasValues[j],0);
        }
        latchCommitAEs(0);
        //monitor(10);
        releasePowerdown(0);
        sendingBias();
    }
}


void device2yarp::setDeviceName(string deviceName) {
    printf("saving portDevice \n");
    portDeviceName=deviceName;
}

void device2yarp::closeDevice(){
    close(file_desc);
}

void  device2yarp::run() {
    bool printExa = true;

    r = read(file_desc, pmonatom, monBufSize_b);
    //printf("received %d bytes \n", r);
    //r = read(file_desc, &buffer, monBufSize_b);
    //printf("called read() with monBufSize_b == %d -> retval: %d\n", (int)monBufSize_b, (int)r);

    if(r < 0) {
        if (errno == EAGAIN) {
            // everything ok, just no data available at the moment...
            // we will be called again in a few miliseconds..
            return;
        } else {
            printf("error reading from aerfx2: %d\n", (int)errno);
            perror("perror:");
            return;
        }
    }

    //int sizeofstructaer  = sizeof(struct aer);
    int sizeofstructatom = sizeof(struct atom);


    //*******************************************************************************************************
    // STATISTICS ON GAEP EVENT FLOW

    // check error of extra byte
    if (r % sizeofstructatom != 0) {
        printf("ERROR: read %d bytes from the AEX!!!\n", r);
    }
    monBufEvents = r / (sizeofstructatom);
    countAEs += monBufEvents;

    //int k = 0;
    int k2 = 0;
    char*       pBuffer = &buffer[0];
    uint32_t *     buf2 = (uint32_t*)          pBuffer;
    //unsigned char* buf1 = (unsigned char*)     pmonatom;
    u32 a, t, c;
    u32 tempA, tempA_unmasked, tempB_unmasked;
    u32 tempC_unmasked;
    //int alow, ahigh;
    //int tlow, thigh;
    int lastTSindex = -1;
    int lastAEindex = -1;
    int countEventSent = 0;
    int countUndetectedWA = 0;

    yarp::os::Bottle tmpBottle;
    for (unsigned int i = 0; i < monBufEvents ; i++ ) {


        tempA = pmonatom[i].data;

        // dumping the atom as it is
        /*
        if(save) {
            fprintf(fout, "     %08X  \n",tempA);
        }
        */

        // running statistic of the received train of events
        tempA_unmasked = (tempA & 0xFC000000) >> 26;

        // ------ CONTROL MESSAGE  ------------
        if(tempA_unmasked == 0x31) {
            printf("CONTROL MESSAGE ");
            tempC_unmasked = tempA & 0x00FFFFFF;
            printf("  %d address events lost \n", tempC_unmasked);
            c = tempA;
            countLostAE += tempC_unmasked;

            lastTSindex = -1;
            lastAEindex = -1;

        }
        // ---------- ADDRESS EVENT -----------------
        else if(tempA_unmasked == 0x00 ){

            a = tempA;

            if((countData - lastAEindex != 2) && (lastAEindex != -1)) {
                printf("ERROR AE 1 > %d %d %08X \n",  countData, lastAEindex, a);
                countErrorsAE++;
                //a = 0xDEADDEAD;
            }
            lastAEindex = countData;
            countInWraps++;

            a &= 0x0000FFFF; //removing extra bits which do not match the protocol
            if(sync)
            {
                printf("Add sync bit\n");
                a+=0x00010000;
                //fprintf(fout,"**sync**\n");
                sync=false;
            }

            if (save) {
                if(printExa) {
                    fprintf(fout,"%08X \n",a);
                }
                else {
                    fprintf(fout,"%lu,",(long unsigned int)a);
                }
            }

            if(!timestampAvailable)
            {
                std::cerr << "An addressevent occured without a preceeding "
                             "timestamp. This really shouldn't happen but it "
                             "did, so perhaps check it out!" << std::endl;
                continue;
            }

            //first put in our associate timestamp (it could be from the last
            //buffer of events!)
            buf2[k2++] = associatedTimestamp;
            bottle2send->addInt(associatedTimestamp);
            timestampAvailable = false;

            //then put in our address

            buf2[k2++] = a;   // passing the address event to the data flow to send
            countEventSent++;

            //tmpBottle.addInt(tempA); //<----- used in the alternative B
            bottle2send->addInt(tempA);

        }
        // -------------- TIMESTAMP EVENT ----------------
        else if (tempA_unmasked == 0x20) {
            t = tempA;
            countInWrapsTS++;

            tempC_unmasked = (tempA & 0x03000000) >> 24; // buffer overflow
            if(tempC_unmasked == 0x01) {
                //printf("BUFFER OVERFLOW  \n");
                countErrorsGAEP1++;
            }
            //time stamp sequence error
            tempC_unmasked = (tempA & 0x03000000) >> 24; // timestamp error
            if(tempC_unmasked == 0x02) {
                //printf("TIMESTAMP ERROR %08X \n",tempA);
                countErrorsGAEP2++;
            }
            //address event sequence error
            tempC_unmasked = (tempA & 0x03000000) >> 24; // address error
            if(tempC_unmasked == 0x03) {
                //printf("ADDRESS ERROR %08X \n", tempA);
                countErrorsGAEP3++;
            }

            // if TS, check the 26th bit for GAEP error
            tempB_unmasked = tempA_unmasked & 0x01;
            if(tempB_unmasked == 1) {
                //printf("GAEP ERROR!!!!!!! \n");
                //general GAEP warning. No longer printed out
            }

            // checking for undetected wrap around
            if((t & 0x00FFFFFF) < (t_prev & 0x00FFFFFF )) {
                //printf("undetected wrap_around %08X %08X > %s %s > %08X \n", t_prev & 0x00FFFFFF, t & 0x00FFFFFF, byte_to_binary(t_prev & 0x00FFFFFF),byte_to_binary(t & 0x00FFFFFF),  (t_prev & 0x00FFFFFF) - (t & 0x00FFFFFF));
                countUndetectedWA++;
            }
            t_prev = t;

            //checking for missed addresses
            if((countData - lastTSindex != 2) && (lastTSindex != -1) && ( t != 0x80000000)) {
                //a = 0xDEADDEAD;
                a = 0x0000DEAD;
                printf("ERROR TS 1 > %d %d %08X \n", countData, lastTSindex, t);
                countErrorsTS++;
            }
            lastTSindex = countData;

            if (save) {
                if(printExa) {
                    fprintf(fout,"%08X ",t);
                }
                else {
                    fprintf(fout,"%lu,",(long unsigned int)t);
                }
            }
            //cout<<"raw ts: "<<t<<std::endl;



            //we delay the sending of the timestamp till when we know the
            //address event it is associated with

            if(timestampAvailable)
            {
                //if a timestamp is already available we have an error!
                std::cerr << "Multiple timestamps without addressevent. "
                             "This really shouldn't happen but it did, so "
                             "perhaps check it out!" << std::endl;
            }

            //Arren isn't sure whether this should be incremented here. I'm not
            //sure if it's events sent by the hardware (and received here)
            //of if it's events sent by this module (in which case the timestamp
            //is delayed and this int might not be sent yet!

            countEventSent++;

            associatedTimestamp = tempA;
            timestampAvailable = true;

            //copying the atomic block to send

            //directly add timestamp
            //buf2[k2++] = t;
            //bottle2send->addInt(tempA);

        }
        // --------------- TIMESTAMP WRAP AROUND ------------
        else if(tempA_unmasked == 0x22) {
            wrapOccured = true;
            // if TS, check the 26th bit for GAEP error
            tempB_unmasked = tempA_unmasked & 0x01;
            if(tempB_unmasked == 1) {
                //printf("GAEP ERROR!!!!!!! \n");
                //general error warning no longer printed out
            }

            tempC_unmasked = (tempA & 0x03000000) >> 24; // buffer overflow
            if(tempC_unmasked == 0x01) {
                //printf("BUFFER OVERFLOW \n");
                countErrorsGAEP1++;
            }
            //time stamp sequence error
            tempC_unmasked = (tempA & 0x03000000) >> 24; // timestamp error
            if(tempC_unmasked == 0x02) {
                //printf("TIMESTAMP ERROR %08X \n", tempA);
                countErrorsGAEP2++;
            }
            //address event sequence error
            tempC_unmasked = (tempA & 0x03000000) >> 24; // address error
            if(tempC_unmasked == 0x03) {
                //printf("ADDRESS ERROR2 %08X \n", tempA);
                countErrorsGAEP3++;
            }

            /********************************************************/
            // wrap-around occured printing out statistics

            printf("\n \n ===================== WRAP AROUND ===================================== \n");
            //a = 0xCAFECAFE;
            a = 0x0000CAFE;
            t = tempA;
            lastTSindex = -1;
            lastAEindex = -1;
            if((countInWraps > maxCountInWraps) && (!firstWrap)) {
                maxCountInWraps = countInWraps;
            }
            if((countInWraps < minCountInWraps) && (!firstWrap)) {
                minCountInWraps = countInWraps;
            }

            if(firstWrap) {
                firstWrap = false;
            }
            else {
                double maxRate = (double)  maxCountInWraps / (0xFFFFFF * 0.001);
                double minRate = (double)  minCountInWraps / (0xFFFFFF * 0.001);
                double lostRate = (double) countLostAE     / (0xFFFFFF * 0.001);
                double dataRate = (double) countInWraps    / (0xFFFFFF * 0.001);
                printf("Overall Statistic \n");
                printf("================= \n");
                printf("max data rate received  %f kAE/s   ; min data rate received %f kAE/s \n", maxRate, minRate);
                printf("data rate received %f kAE/s    ;  data rate lost %f kAE/s    ;    LOR %f%% \n", dataRate, lostRate, (double)((lostRate / (lostRate + dataRate)) * 100.0));
                printf(" \n Error Resume \n");
                printf("============= \n");
                printf("type1: %04d     type2: %04d       type3: %04d/%04d \n", countErrorsGAEP1, countErrorsGAEP2, countErrorsGAEP3, countInWraps + countInWrapsTS);
                printf("type1: %04f type2: %04f type3: %04f \n", (double)countErrorsGAEP1 / (countInWraps + countInWrapsTS), (double)countErrorsGAEP2 / (countInWraps + countInWrapsTS), (double)countErrorsGAEP3 / (countInWraps + countInWrapsTS));
                printf("undetected WrapAdd error %d \n", countUndetectedWA);
            }
            //**********************************************************/
            // reinitialising the statistic
            printf("resetting the statistic \n");
            countInWraps     = 0;
            countInWrapsTS   = 0;
            countLostAE      = 0;
            countErrorsGAEP1 = 0;
            countErrorsGAEP2 = 0;
            countErrorsGAEP3 = 0;
            t_prev = 0;

            buf2[k2++] = t; // passing the timestamp to the data flow to send
            //buf2[k2++] = a;
            countEventSent++;

            countData=0; // ADDED RESET TO COUNTDATA

            if (save) {
                if(printExa) {
                    fprintf(fout,"%08X\n",t);
                }
                else {
                    fprintf(fout,"%lu ",(long unsigned int)t);
                }
                //fprintf(fout," %08X ",0xCAFECAFE);
            }
        }
        // --------------- other events  ------------
        else {

            if (save) {
                if(printExa) {
                    fprintf(fout,"%08X\n",tempA);
                }
                else {
                    fprintf(fout,"%lu,",(long unsigned int)tempA);
                }
                //fprintf(fout," %08X ",0xCAFECAFE);
            }

            //TODO: it is possible for the board to send hardware-based
            //events other than address events. for example cluster events.
            //these events should be accounted for here but currently the
            //events are just discared.
            //Currently we *shouldn't* get any of these event types

            std::cout << "Unknown Event (discarded): " << tempA << std::endl;

            //copying the atomic block to send
            //tempA &= 0x0000FFFF;
            //buf2[k2++] = tempA; // passing the timestamp to the data flow to send
            //countEventSent++;

            //tmpBottle.addInt(tempA); //<----- used in the alternative B
            //bottle2send->addInt(tempA);

        }

        countData++;
        //fprintf(stdout, "count... %d\n", countData) ;
    }

    //if (save) {
    //    fprintf(fout,"------------------------\n");
    //}

     //*******************************************************************************************************

    sz = countEventSent * sizeofstructatom;  //sz is size in bytes
    char* buf = (char*) buf2;

//    if (port.getOutputCount()) {
//        //printf("preparing the data to send on the port \n");
//        eventBuffer data2send(buffer, sz);  //adding 8 bytes for extra word 0xCAFECAFE and TS_WA
//        eventBuffer& tmp = port.prepare();
//        tmp = data2send;
//        port.write();
//    }

//    if (portEventBottle.getOutputCount()) {
//        //countCycle++;
//        //printf("bytes %d on the portEventBottle \n", bottle2send->size());
//        Stamp st;
//        st.update();

//        eventBottle data2send(bottle2send);
//        eventBottle& tmp = portEventBottle.prepare();
//        tmp = data2send;
//        portEventBottle.setEnvelope(st);

//        //fprintf(stdout, "TIME ... %lf\n", st.getTime()) ;

//        portEventBottle.write();
//        //fprintf(stdout, "count... %d\n", countCycle) ;

//        /*
//        if (save) {
//            fprintf(fbottle,"dim %d \n", bottle2send->size());
//            //printf("dim %d \n", bottle2send->size());
//            for(int i = 0; i < bottle2send->size(); i++) {
//                fprintf(fbottle,"%08X \n", bottle2send->get(i).asInt());
//            }
//            fprintf(fbottle,"------------------------\n");
//        }
//        */
//    }

    if (portvBottle.getOutputCount()) {
        ev::vBottle &vb = portvBottle.prepare();
        vb.clear();

        //do some sketchy casting to make things fast at this part of the
        //project

        Bottle * bb = (Bottle *)&vb;
        bb->addString("AE");

        //this doesn't ensure that the data goes TS -> AE -> TS ->
        //and that might be a problem

        //bb->append(*bottle2send);
        yarp::os::Bottle &eventlist = bb->addList();
        eventlist = *bottle2send;

        vStamp.update();
        portvBottle.setEnvelope(vStamp);
        portvBottle.write();

    }

    wrapOccured = false;

    if (portDimension.getOutputCount()) {
        Bottle& b = portDimension.prepare();
        b.clear();
        b.addDouble((double)r);
        portDimension.write();
    }

    //resetting buffers
    bottle2send->clear();
    //printf("resetting the buffer \n");
    memset(buffer, 0, SIZE_OF_DATA);

}


void device2yarp::sendingBias() {
    int busy;
    seqDone_b = 0;
    printf("-------------------------------------------- \n");
    printf("trying to write to kernel driver %d %d \n", seqDone_b,seqSize_b);
    while (seqDone_b < seqSize_b) {
        // try writing to kernel driver
        printf( "calling write fd: sS: %d  sD: %d \n", seqSize_b, seqDone_b);
        int w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);

        if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                /* we were not busy, maybe someone else is... */
            } else {
                perror("invalid write");
                yarp::os::exit(1);
            }
    }
    printf("writing successfully ended \n");

    ////////////////////////////////////////////////////////////


    double TmaxSeqTimeEstimate =
        seqTime * 0.128 * 1.10 +   // summed up seq intervals in us plus 10%
        seqEvents * 1.0;           // plus one us per Event


    printf("seqEvents: %d \n", seqEvents);
    printf("seqTime * 0.128: %d \n", (int)(u64)(seqTime * 0.128));
    //printf("TmaxSeqTimeEstima PRIu64  %f %f\n",
    //  (TmaxSeqTimeEstimate / 1000000), (TmaxSeqTimeEstimate % 1000000));


    ////////////////////////////////////////////////////////////

}

void device2yarp::progBias(string name,int bits,int value, int camera ) {
    int bitvalue;

    for (int i=bits-1;i>=0;i--) {
        int mask=1;
        for (int j=0; j<i; j++) {
            mask*=2;
        }
        //printf("mask %d ",mask);
        if (mask & value) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        progBitAEs(bitvalue, camera);
    }
    //after each bias value, set pins back to default value
    //resetPins();
}

void device2yarp::latchCommit(int camera ) {
    //printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_LO, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    //printf("exiting latch_commit \n");
}

void device2yarp::latchCommitAEs(int camera ) {
    //printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_HI, 0,0, camera );
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0, camera);
    //printf("exiting latch_commit \n");
}

void device2yarp::resetPins(int camera ) {
    //now
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, camera );
    biasprogtx(reset_pins_expand * timestep, LATCH_KEEP, CLOCK_HI, 0, camera);
    //printf("exiting latch_commit \n");
}

void device2yarp::releasePowerdown(int camera ) {
    //now
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 0, camera);
    biasprogtx(latchexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 0, camera);
    //printf("exiting latch_commit \n");
}


void device2yarp::setPowerdown(int camera ) {
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 1, camera);
    biasprogtx(powerdownexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 1, camera);
}


void device2yarp::progBit(int bitvalue, int camera ) {
    //set data
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera );
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera);
}

void device2yarp::progBitAEs(int bitvalue, int camera ) {

    //set data (now)
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    //and wait a little
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
}

void device2yarp::monitor (int secs, int camera ) {
    //printf("entering monitor \n");
    biasprogtx(secs * OneSecond, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    //printf("exiting monitor \n");
}

void device2yarp::biasprogtx(int time,int latch,int clock,int data, int powerdown, int camera ) {
    unsigned char addr[4];
    unsigned char t[4];
    //int err;
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
    if (powerdown) {
        addr[3] += 0x08;
    }
    //printf("data:0x%x, 0x%x, 0x%x, 0x%x \n",addr[0],addr[1],addr[2],addr[3]);


    //u32 seqSize_b = sizeof(struct aer);
    u32 timeToSend, addressToSend;
    timeToSend=time;


    // performing trasmittion differently if the camera is left (1) or right (0)
    // keep the clock high for the other eye
    if(camera) {
        addressToSend=0x000000060;
        if(data) {
            addressToSend += 0x01;
        }
        if(clock) {
            addressToSend += 0x02;
        }
        if (latch) {
            addressToSend += 0x04;
        }
        if (powerdown) {
            addressToSend += 0x08;
        }
        addressToSend+=0xFF000000;
    }
    else {
        addressToSend=0x000000006;
        if(data) {
            addressToSend += 0x10;
        }
        if(clock) {
            addressToSend += 0x20;
        }
        if (latch) {
            addressToSend += 0x40;
        }
        if (powerdown) {
            addressToSend += 0x80;
        }
        addressToSend+=0xFF000000;
    }

    u32 hwival = (u32)(timeToSend * 7.8125);
    //printf("saving the aer.address %x \n", addressToSend);
    pseq[seqEvents].address = addressToSend;
    //printf("saving the aer.time  \n");
    pseq[seqEvents].timestamp = hwival;

    seqEvents++;
    //printf("number of saved events %d as tot %d ",seqEvents,seqSize_b);
    seqTime += hwival;
    seqSize_b += sizeof(struct aer);

    //printf("writing the event");
    //int w = write(file_desc, ((u8*)pseq), seqSize_b);
    //addr = int(sys.argv[1], 16)
    //err = write(file_desc,t,4); //4 byte time: 1 integer
    //err = write(file_desc,addr,4); //4 byte time: 1 integer
}

void device2yarp::setDumpEvent(bool value) {
    save = value;

    if(!value) {
        if(fout != NULL) {
            fclose(fout);
            printf("closing the file \n");
        }

        if(fbottle != NULL) {
            fclose(fbottle);
            printf("closing the file \n");
        }

    }
}

bool device2yarp::setDumpFile(std::string value) {
    dumpfile = value;
    //fout.open(dumpfile.c_str());
    //bool ret = fout.is_open();
    //if (!ret)
    //    cout << "unable to open file" << endl;

    fout    = fopen(dumpfile.c_str(),"w");
    fbottle = fopen("dumpBottle.txt","w");
    if(fout!=NULL)
        return true;
    else
        return false;
}

void device2yarp::setSyncBit() {
    sync = true;
}

void device2yarp::threadRelease() {

    /* it is better not to set the powerdown at the end!
    const u32 seqAllocChunk_b = SIZE_OF_EVENTS * sizeof(struct aer); //allocating the right dimension for biases
    memset(pseq,0,seqAllocChunk_b);
    setPowerdown();
    sendingBias();
    */

    stopInt=Time::now();
    double diff = stopInt - startInt;
    double maxRate = (double) maxCountInWraps / (0xFFFFFF * 0.001);
    double minRate = (double) minCountInWraps / (0xFFFFFF * 0.001);
    printf("max data rate received  %f  kAE/s ; min data rate received %f kAE/s \n", maxRate, minRate);
    printf("the grabber has collected %d AEs (%d errors) in %f seconds \n",countAEs,countErrors,diff);
    //port.close();
    portDimension.close();
    //portEventBottle.close();

    close(file_desc);

}

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
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>

#include <sys/types.h>
#include <inttypes.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <sys/time.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;
using namespace yarp::os;
//using namespace emorph::ebuffer;

extern int errno;

#define __DEBUG__
#define INPUT_BIN_U32U32LE

#define THRATE 1


device2yarp::device2yarp(int file_desc):RateThread(THRATE) {
    printf("device2yarp initialization .... \n");
    
    wrapOccured      = false;
    //onlyLeft         = false;
    
    countAEs = 0;
    countErrors = 0;
    
    //associatedTimestamp = 0;
    //save = false;
    //sync = false;
    // passing the parameter to the class variable
    this->file_desc = file_desc;
    //this->biasFileName   = i_fileName;
    
    //initialisation of the module
    len=0;
    sz=0;
    
    
    deviceData.resize(1024); //max size of data from device /dev/spinn2neu
    
    startInt=Time::now();
}


device2yarp::~device2yarp() {
    
        portvBottle.close();
    
}


bool device2yarp::threadInit(std::string moduleName){
  //check device?
    printf("opening port for sending the events from spinn2neu \n");
    std::string outPortName = moduleName + "/vBottle:o";
    bool success = portvBottle.open(outPortName);

    if(!success)
    {
        fprintf(stdout,"error opening port %s", outPortName.c_str());
        return false;
    }

    return true;
}



void  device2yarp::run() {
    
    //fprintf(stdout,"device2yarp::run()\n");
    
    int           i;
    int           j;
    //int           real_data;
    int           offset;
    
    int           devData;
    
    // decode the event sent by spinnaker
    
    // address
    int           word0;
    int           polarity;
    int           channel;
    int           x,y;
    long int      ts;
    
    // timestamp
    
    
    // create address event that we will store in the vBottle
    emorph::AddressEvent aep;
    
    //read data from device
    
    //RXDATA = (unsigned int *) calloc(1024, sizeof(unsigned int));
    
    j = 0;
    offset = 0;
    
    
    //real_data = read(file_desc, (char *)(RXDATA)+offset, 1024*sizeof(unsigned int));
    devData = read(file_desc, (char *)(deviceData.data()), 1024*sizeof(unsigned int));
    
    if (devData<0){
        if (errno == EAGAIN) {
            // everything ok, just no data available at the moment...
            // we will be called again in the nex run()...
            return;
            
        } else {
            printf("error reading from spinn2neu: %d\n", (int)errno);
            perror("perror:");
            return;
        }
    } else if (devData == 0)
	{
	    // everything ok, no data available, just call the run again later
		return; 
	}
    
    // Show data
#ifdef __DEBUG__
//    for (i=0; i<(devData/sizeof(unsigned int)); i++)
//        fprintf(stderr, "%4d 0x%08x %10u %s\n", j++, deviceData[i],  deviceData[i], i%2 ? "D" : "T");
    
//    sleep(1);
    
#endif
    
    //*******************************************************************************************************
    // STATISTICS ON EVENT FLOW -- TO BE DONE!!!!
    
    
    
    //*******************************************************************************************************
    // convert data to YARP vBottle
    
    emorph::vBottle &evtDevice = portvBottle.prepare();
    evtDevice.clear();
    
    for (i=0; i<(devData/sizeof(unsigned int)); i++)
    {
        word0 = deviceData[i];
        
        if (i%2) // Address
        {
            polarity=word0&0x01;
            
            word0>>=1;
            x=word0&0x7f;
            
            word0>>=7;
            y=word0&0x7f;
            
            word0>>=7;
            channel=word0&0x01;
            
            // fill address event
            
            aep.setChannel(channel);
            aep.setPolarity(polarity);
            aep.setX(x);
            aep.setY(y);
            
        }
        else // Timestamp
        {
            ts = word0&0x00ffffff;
            aep.setStamp(ts);
        }
        evtDevice.addEvent(aep);
        
            
#ifdef __DEBUG__
	fprintf(stderr,"read %d events from device\n",devData/sizeof(unsigned int));
       // fprintf(stderr, "vector %4d 0x%08x %10u %s\n", j++, deviceData[i],  deviceData[i], i%2 ? "D" : "T");
    
#endif
    
        
    }
    // add event to cluster
    
    // write vBottle to output port
    portvBottle.write();
    countAEs = countAEs + devData/sizeof(unsigned int);

    
    /*if ((offset = (real_data+offset) % sizeof(unsigned int)) != 0)
        RXDATA[0] = RXDATA[i];
    return;
    */
}

void device2yarp::threadRelease() {
    
    stopInt=Time::now();
    double diff = stopInt - startInt;
    //double maxRate = (double) maxCountInWraps / (0xFFFFFF * 0.001);
    //double minRate = (double) minCountInWraps / (0xFFFFFF * 0.001);
    //printf("max data rate received  %f  kAE/s ; min data rate received %f kAE/s \n", maxRate, minRate);
    printf("the grabber has collected %d AEs (%d errors) in %f seconds \n",countAEs,countErrors,diff);
    portvBottle.close();
    //free(RXDATA);
    
}

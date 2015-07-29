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
#include <unistd.h>
#include <errno.h>

extern int errno;

#define __DEBUG__
#define INPUT_BIN_U32U32LE

#define THRATE 1


device2yarp::device2yarp(int file_desc):RateThread(THRATE) {
       
    countAEs = 0;
    this->file_desc = file_desc;
    deviceData.resize(1024);
}

bool device2yarp::threadInit(std::string moduleName){

    std::string outPortName = moduleName + "/vBottle:o";
    return portvBottle.open(outPortName);

}

void  device2yarp::run() {


    //read the device
    int devData = read(file_desc, (char *)(deviceData.data()), 1024*sizeof(unsigned int));
    if (devData < 0){
        if (errno != EAGAIN) {
            printf("error reading from spinn2neu: %d\n", (int)errno);
            perror("perror:");
        }
        //if errno == EAGAIN ther is just no data to read just now
        // we are using a non-blocking call so we need to return and wait for
        // the thread to run again.
        return;
    } else if(devData == 0) {
        // everything ok, no data available, just call the run again later
        return;
    }


    int nEvtsRead = devData / sizeof(unsigned int);

    //data will always be less than 2^16 as it fits in the first 16 bits
    if(deviceData[0] < 65536 && deviceData[1] < 65536) {
        //we have no way to distinquish between these values...
        std::cout << "Blind Spot" << std::endl;
        return;
    }

    int i = 0;
    if(deviceData[0] < 65536) {
        //this is not a timestamp so we want to ignore it.
        i = 1;
    }

//    if(nEvtsRead % 2) {
//        std::cerr << "An odd number of events where read. We need to "
//                     "implement a robust checking of timestamp - data ordering"
//                     << std::endl;
//        std::cerr << "Exiting this thread because we cannot gaurantee the first"
//                     " event is a timestamp! (and we cannot gaurantee it for any"
//                  "of the following data reads either)" << std::endl;
//        return;
//    }



    // convert data to YARP vBottle
    
    emorph::vBottle &evtDevice = portvBottle.prepare();
    evtDevice.clear();
    
    emorph::AddressEvent ae;
    for (i; i < nEvtsRead; i += 2)
    {
        int ts = deviceData[i] & 0x00ffffff;
        int word0 = deviceData[i+1];

        
        int polarity=word0&0x01;
        word0>>=1;

        int x=word0&0x7f;
        word0>>=7;

        int y=word0&0x7f;
        word0>>=7;

        int channel=word0&0x01;

        ae.setStamp(ts);
        ae.setChannel(channel);
        ae.setPolarity(polarity);
        ae.setX(x);
        ae.setY(y);
        evtDevice.addEvent(ae);

    }

    portvBottle.write();
    countAEs = countAEs + nEvtsRead;

}

void device2yarp::threadRelease() {

    std::cout << "The grabber has collected " << countAEs << " events"
              << std::endl;

    portvBottle.close();
    
}

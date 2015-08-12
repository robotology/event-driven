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
using namespace std;
using namespace yarp::os;
//using namespace emorph::ebuffer;

extern int errno;

#define __DEBUG__
#define INPUT_BIN_U32U32LE

#define THRATE 1


device2yarp::device2yarp():RateThread(THRATE) {
       
    countAEs = 0;
    deviceData.resize(1024);
}

bool device2yarp::threadInit(std::string moduleName){

    std::string outPortName = moduleName + "/vBottle:o";
    return portvBottle.open(outPortName);

}

void  device2yarp::run() {
    
    if(devManager->readFifoFull()){
        std::cout<<"D2Y read: error fifo full"<<std::endl;
        
    }
    
    
    //read the device, returns the number of bytes read
    int devData = devManager->readDevice(deviceData);
    
    // check this!
    if (devData <= 0){
        return;
    }
    
    // pointer to the vector of events
    char* buff = (char *)deviceData.data();

    /*
    int nEvtsRead = devData / sizeof(unsigned int);

    if(nEvtsRead % 2) {
        std::cerr << "An odd number of events where read. We need to "
                     "implement a robust checking of timestamp - data ordering"
                     << std::endl;
        std::cerr << "Exiting this thread because we cannot gaurantee the first"
                     " event is a timestamp! (and we cannot gaurantee it for any"
                  "of the following data reads either)" << std::endl;
        return;
    }
    */


    // convert data to YARP vBottle
    
    emorph::vBottle &evtDevice = portvBottle.prepare();
    evtDevice.clear();
    
    //do some sketchy casting to make things fast at this part of the
    //project
    Bottle * bb = (Bottle *)&evtDevice;
    
    //now we can add our searchable tag
    bb->addString("AE");
    yarp::os::Bottle &eventlist = bb->addList();
    
    int bytesdropped = 0;
    int i = 0;
    
    // scan the vector read from the device
    while(i <= devData - 8) {
        
        int TS =  *(int *)(buff + i);//= deviceData[i];
        int AE =  *(int *)(buff + i + 4);//deviceData[i+1];
        
        if(!(TS & 0x80000000) || (AE & 0xFFFF0000)) {
            //misalignment, move on by 1 byte
            bytesdropped++;
            i += 1;
        } else {
            //successful data match move on by 8 bytes
            if(!eventlist.size())
                std::cout << "1 " << (int)(TS & 0x00FFFFFF) << std::endl;
            else
                std::cout << "0 " << (int)(TS & 0x00FFFFFF) << std::endl;
            
            eventlist.add((int)(TS & 0x80FFFFFF));
            eventlist.add(AE);
            i += 8;
        }
    }
    bytesdropped += devData - i;
    
    //if(bytesdropped)
    //    std::cerr << "Lost " << bytesdropped << " bytes" << std::endl;
    
    countAEs += eventlist.size();
    
    vStamp.update();
    portvBottle.setEnvelope(vStamp);
    portvBottle.write();

}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;
    
    
    portvBottle.close();
    
}

void  device2yarp::attachDeviceManager(deviceManager* devManager) {
    this->devManager = devManager;
    
}

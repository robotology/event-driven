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
#define THRATE 1

device2yarp::device2yarp() : RateThread(THRATE) {
       
    countAEs = 0;
    prevTS = 0;

}

bool device2yarp::threadInit(std::string moduleName){

    //portvBottle.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return portvBottle.open(outPortName);

}

void  device2yarp::run() {
    
    //get the data from the device read thread
    int nBytesRead = 0;
    const std::vector<char> &data = devManager->readDevice(nBytesRead);
    if (!nBytesRead) return;

    if(nBytesRead > devManager->getBufferSize()*0.75) {
        std::cerr << "Software buffer was over 3/4 full - check the "
                     "device2yarp thread is not delayed" << std::endl;
    }

    int bstart = 0;
    int bend = 0;

    int *TS =  (int *)(data.data() + bend);
    int *AE =  (int *)(data.data() + bend + 4);
    bool BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFFF0000);

    //while we have a non multiple of 8 bytes, the first two ints are not
    //correctly a TS then AE and we aren't greater than the total # bytes
    while(((nBytesRead-bend) % 8 || BITMISMATCH) && (bend <= nBytesRead - 8)) {

        //note: BITMISMATCH stopped occuring by the time this code was
        //implemented and as such this never had a chance to be tested.
        //good luck.

        if(BITMISMATCH) {
            //send on what we have checked is not mismatched so far
            if(bend - bstart > 0) {
                std::cerr << "BITMISMATCH in yarp2device" << std::endl;
                sender.setdata(data.data()+bstart, bend-bstart);
                countAEs += (bend - bstart) / 8;
                vStamp.update();
                portvBottle.setEnvelope(vStamp);
                portvBottle.write(sender);
            }
            //then increment by 1 to find the next alignment
            bend++;
            bstart = bend;
        } else {
            bend += 8;
        }

        TS =  (int *)(data.data() + bend);
        AE =  (int *)(data.data() + bend + 4);
        BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFFF0000);

    }

    if(nBytesRead - bstart > 0) {
        sender.setdata(data.data()+bstart, 8*((nBytesRead-bstart)/8));
        countAEs += (nBytesRead - bstart) / 8;
        vStamp.update();
        portvBottle.setEnvelope(vStamp);
        portvBottle.write(sender);
    }

    if(yarp::os::Time::now() - prevTS > 15) {
        std::cout << "ZynqGrabber running happily: " << countAEs
                  << " events" << std::endl;
        prevTS = yarp::os::Time::now();
    }

//    if(countAEs > 50000) {
//        //std::cout << "Specialisation Test: " << bb->getSpecialization() << " "
//        //          << eventlist.getSpecialization() << std::endl;
//        std::cout << countAEs / (yarp::os::Time::now() - prevTS)
//                  << " v/sec" << std::endl;
//        countAEs = 0;
//        prevTS = yarp::os::Time::now();
//    }

}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;
    
    
    portvBottle.close();
    
}

void  device2yarp::attachDeviceManager(deviceManager* devManager) {
    this->devManager = devManager;
    
}

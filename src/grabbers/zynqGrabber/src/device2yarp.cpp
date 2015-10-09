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

    portvBottle.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return portvBottle.open(outPortName);

}

void  device2yarp::run() {
    
    //get the data from the device read thread
    int nBytesRead = 0;
    const std::vector<char> &data = devManager->readDevice(nBytesRead);
    if (!nBytesRead) return;
    if(nBytesRead > devManager->getBufferSize()/2) {
        std::cerr << "Software buffer was over half full - check the "
                     "device2yarp thread is not delayed" << std::endl;
    }

    // convert data to YARP vBottle
    emorph::vBottle &evtDevice = portvBottle.prepare();
    evtDevice.clear();
    
    //do some sketchy casting to make things fast at this part of the project
    yarp::os::Bottle * bb = (yarp::os::Bottle *)&evtDevice;
    
    //now we can add our searchable tag
    bb->addString("AE");
    //and add our bottle to fill with events
    yarp::os::Bottle &eventlist = bb->addList();

    int vcount = 0, cts = 0;

    if(devManager->getDevType() == "/dev/iit_hpucore") {
        //we are on the zturn

        //if we read a multiple of 8 bytes (uint32 TS uint32 XYPC) we assume the
        //data is aligned correctly and add it to the bottle otherwise we check
        //until we find a misalignment, re-align, then add the rest
        int i = 0;
        if((nBytesRead) % 8 == 0) {

            while(i <= nBytesRead - 8) {
                int *TS =  (int *)(data.data() + i);//= deviceData[i];
                int *AE =  (int *)(data.data() + i + 4);//deviceData[i+1];
                //eventlist.add((int)(*TS & 0x80FFFFFF));
                //eventlist.add(*AE);
                vcount++; cts = *TS & 0x00FFFFFF;
                i += 8;
            }

        } else {
            std::cout << "Alignment Required" << std::endl;
            while(i <= nBytesRead - 8) {

                int *TS =  (int *)(data.data() + i);//= deviceData[i];
                int *AE =  (int *)(data.data() + i + 4);//deviceData[i+1];

                if((nBytesRead - i) % 8 && (!(*TS & 0x80000000) || (*AE & 0xFFFF0000)))
                    i++;
                else {
                    //eventlist.add((int)(*TS & 0x80FFFFFF));
                    //eventlist.add(*AE);
                    vcount++; cts = *TS & 0x00FFFFFF;
                    i += 8;
                }
            }
        }

        countAEs += vcount;

        if(countAEs > 50000) {
            if(prevTS > cts) prevTS -= 16777216;
            std::cout << 1000000 * countAEs / (double)(cts - prevTS)
                      << " v/sec" << std::endl;
            countAEs = 0;
            prevTS = cts;
        }

    } else if(devManager->getDevType() == "/dev/aerfx2_0") {

        int i = 0;
        while(i <= nBytesRead - 8) {
            int* TS =  (int *)(data.data() + i);//= deviceData[i];
            int* AE =  (int *)(data.data() + i + 4);//deviceData[i+1];

            if(!(*TS & 0x80000000) || (*AE & 0xFFFF0000)) {
                //misalignment, move on by 1 byte
                i += 1;
            } else {
                //successful data match move on by 8 bytes
                eventlist.add((int)(*TS & 0x80FFFFFF));
                eventlist.add(*AE);
                vcount++; cts = *TS & 0x00FFFFFF;
                i += 8;
            }
        }

        countAEs += vcount;

        if(countAEs > 50000) {
            if(prevTS > cts) prevTS -= 16777216;
            std::cout << 7.8125 * 1000000 * countAEs / (double)(cts - prevTS)
                      << " v/sec" << std::endl;
            countAEs = 0;
            prevTS = cts;
        }

    }

    //countAEs += eventlist.size() / 2;

    vStamp.update();
    portvBottle.setEnvelope(vStamp);
    portvBottle.writeStrict();
    
//    int bytesdropped = 0;
//    int i = 0;
//    int deltabetween = 0;
//    int deltawithin = 0;
//    int pts = 0;

//    if(nBytesRead % 8)
//        std::cerr << "We aren't reading at 8x" << std::endl;

//    // scan the vector read from the device
//    while(i <= nBytesRead - 8) {


//        int TS =  *(int *)(data.data() + i);//= deviceData[i];
//        int AE =  *(int *)(data.data() + i + 4);//deviceData[i+1];
        
//        if(!(TS & 0x80000000) || (AE & 0xFFFF0000)) {
//            //misalignment, move on by 1 byte
//            bytesdropped++;
//            std::cout << i << " ";
//            i += 1;
//        } else {
//            //successful data match move on by 8 bytes

//            //delta between last bottle and this bottle
//            if(!deltabetween) deltabetween = (TS & 0x00FFFFFF) - prevTS;
//            //delta between stamps in the bottle
//            if(pts)
//                deltawithin = std::max(deltawithin, (TS & 0x00FFFFFF) - (pts & 0x00FFFFFF));
//            pts = TS;

//            eventlist.add((int)(TS & 0x80FFFFFF));
//            eventlist.add(AE);
//            i += 8;
//        }
//    }
//    int tail = nBytesRead - i;
    
//    if(bytesdropped)
//        std::cerr << "Lost " << bytesdropped << " bytes within the data"
//                  << " and " << tail << " at the tail (" << nBytesRead
//                  << ")" << std::endl;
//    if(deltabetween > 5000 || deltawithin > 5000) {
//        std::cout << "Delta between bottles: " << deltabetween << ", Delta within"
//                     " bottles: " << deltawithin << std::endl;
//    }
    
//    countAEs += eventlist.size() / 2;
//    prevTS = (pts & 0x00FFFFFF);

//    vStamp.update();
//    portvBottle.setEnvelope(vStamp);
//    portvBottle.write();

//    std::cout << "Read " << nBytesRead << ". TS: " << pts << std::endl;

}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;
    
    
    portvBottle.close();
    
}

void  device2yarp::attachDeviceManager(deviceManager* devManager) {
    this->devManager = devManager;
    
}

/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include <iCub/device2yarp.h>
#define THRATE 1

device2yarp::device2yarp() : RateThread(THRATE) {
    countAEs = 0;
    prevTS = 0;
    strict = false;
    doChannelShift = true;
}

bool device2yarp::threadInit(std::string moduleName, bool strict){

    this->strict = strict;
    if(strict) {
        std::cout << "D2Y: setting output port to strict" << std::endl;
        portvBottle.setStrict();
    } else {
        std::cout << "D2Y: setting output port to not-strict" << std::endl;
    }
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return portvBottle.open(outPortName);

}

void  device2yarp::run() {

    //display an output to let everyone know we are still working.
    if(yarp::os::Time::now() - prevTS > 10) {
        std::cout << "ZynqGrabber running happily: " << countAEs
                  << " events" << std::endl;
        prevTS = yarp::os::Time::now();
    }

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

    while(bend < nBytesRead - 7) {

        //check validity
        int *TS =  (int *)(data.data() + bend);
        int *AE =  (int *)(data.data() + bend + 4);
        bool BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFEF0000);

        if(BITMISMATCH) {
            //send on what we have checked is not mismatched so far
            if(bend - bstart > 0) {
                std::cerr << "BITMISMATCH in yarp2device" << std::endl;
                std::cerr << *TS << " " << *AE << std::endl;

                emorph::vBottleMimic &vbm = portvBottle.prepare();
                vbm.setdata(data.data()+bstart, bend-bstart);
                countAEs += (bend - bstart) / 8;
                vStamp.update();
                portvBottle.setEnvelope(vStamp);
                if(strict) portvBottle.writeStrict();
                else portvBottle.write();

//                sender.setdata(data.data()+bstart, bend-bstart);
//                countAEs += (bend - bstart) / 8;
//                vStamp.update();
//                portvBottle.setEnvelope(vStamp);
//                portvBottle.write(sender); //port is always strict
            }
            //then increment by 1 to find the next alignment
            bend++;
            bstart = bend;
        } else {
            //else scale the timestamp
            //*TS = 0x80000000 | ((*TS & 0x7FFFFFFF) * 1);
#ifdef TENBITCODEC
            if(doChannelShift) //using FPGA with C@bit20
                *AE = (((*AE & 0x00100000) << 1) | *AE) & 0xFFEFFFFF;
            else {
                *AE = (((*AE & 0x00008000) << 6) | *AE) & 0xFFFF7FFF; //ch
                *AE = (((*AE & 0x00007F00) << 3) | *AE) & 0xFFFFF8FF; //y
            }
#else
            if(doChannelShift)
                *AE = (((*AE & 0x00100000) >> 5) | *AE) & 0xFFEFFFFF;
#endif
            //and then check the next two ints
            bend += 8;
        }
    }



    //    int bstart = 0;
    //    int bend = 0;

    //    int *TS =  (int *)(data.data() + bend);
    //    int *AE =  (int *)(data.data() + bend + 4);
    //    bool BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFFF0000);

    //    //while we have a non multiple of 8 bytes, the first two ints are not
    //    //correctly a TS then AE and we aren't greater than the total # bytes
    //    while(((nBytesRead-bend) % 8 || BITMISMATCH) && (bend <= nBytesRead - 8)) {

    //        if(BITMISMATCH) {
    //            //send on what we have checked is not mismatched so far
    //            if(bend - bstart > 0) {
    //                std::cerr << "BITMISMATCH in yarp2device" << std::endl;
    //                sender.setdata(data.data()+bstart, bend-bstart);
    //                countAEs += (bend - bstart) / 8;
    //                vStamp.update();
    //                portvBottle.setEnvelope(vStamp);
    //                portvBottle.write(sender);
    //            }
    //            //then increment by 1 to find the next alignment
    //            bend++;
    //            bstart = bend;
    //        } else {
    //            bend += 8;
    //        }

    //        TS =  (int *)(data.data() + bend);
    //        AE =  (int *)(data.data() + bend + 4);
    //        BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFFF0000);

    //    }

    if(nBytesRead - bstart > 7) {
        emorph::vBottleMimic &vbm = portvBottle.prepare();
        vbm.setdata(data.data()+bstart, 8*((nBytesRead-bstart)/8));
        countAEs += (nBytesRead - bstart) / 8;
        vStamp.update();
        portvBottle.setEnvelope(vStamp);
        if(strict) portvBottle.writeStrict();
        else portvBottle.write();
    }



}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;

    portvBottle.close();

}

bool  device2yarp::attachDeviceManager(deviceManager* devManager) {
    this->devManager = dynamic_cast<aerDevManager*>(devManager);

    if (!devManager){
        return false;

    }

    if(devManager->getDevType() == "/dev/aerfx2_0")
        doChannelShift = false;

    clockScale = this->devManager->getTickToUs();
    return true;


}

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
#include <math.h>

#define THRATE 1

device2yarp::device2yarp() : RateThread(THRATE) {
    countAEs = 0;
    prevTS = 0;
    strict = false;
    doChannelShift = true;
    clockScale = 1;
    countTotEvts = 0;
    countLostEvts = 0;
    prevPayload = -1;
    firstTs = 0;
    
}

bool device2yarp::threadInit(std::string moduleName, bool strict){

    this->strict = strict;
    if(strict) {
        std::cout << "D2Y: setting output port to strict" << std::endl;
        portvBottle.setStrict();
        portScope.setStrict();
    } else {
        std::cout << "D2Y: setting output port to not-strict" << std::endl;
    }
    std::string outPortName = "/" + moduleName + "/vBottle:o";
 
    return portvBottle.open(outPortName)& portScope.open("/" + moduleName + "/scope");

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
    std::cout << "nBytesRead " << nBytesRead << std::endl;
    if (!nBytesRead) return;

    if(nBytesRead > devManager->getBufferSize()*0.75) {
        std::cerr << "Software buffer was over 3/4 full - check the "
                     "device2yarp thread is not delayed" << std::endl;
    }

    int bstart = 0;
    int bend = 0;

    std::vector<int> diffTs;
    diffTs.resize((nBytesRead/8));
    long unsigned int prevTs;
    int countBotEvts = 0;
    
    while(bend < nBytesRead - 7) {

        // debug prints
        
        printf("T: 0x%08X --> ", data[bend]);
        if (data[bend+1] & 0x40000)
        {
            printf("APS: 0x%08X \n", data[bend+1]);
        } else {
            printf("TD: 0x%08X \n", data[bend+1]);
            
        }

        //check validity
        int *TS =  (int *)(data.data() + bend);
        int *AE =  (int *)(data.data() + bend + 4);
        std::cout << "TS: " << *TS << " AE: " << *AE << std::endl;
        //std::cout << "TS: " << *TS  & 0x80000000 << " AE: " << *AE & 0xFFEF0000 << std::endl;
        
//        bool BITMISMATCH = !(*TS & 0x80000000) || (*AE & 0xFFEF0000);

/*        if(BITMISMATCH) {
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
            }
            //then increment by 1 to find the next alignment
            bend++;
            bstart = bend;
        } else {*/
            countTotEvts ++;
            
//            if (prevPayload == -1){
//                firstTs = *TS & 0x7FFFFFFF;
//    
//            }
//            prevTs = unwrap.currentTime();
//            unwrap(*TS & 0x7FFFFFFF);
//            int currPayload = (*AE & 0x0003FFFF);
//            if (prevPayload != -1){
//                
//                if (prevPayload > currPayload){ // wraps of the address
//                    countLostEvts = currPayload - prevPayload - 1 + 0x0003FFFF;
//                    
//                } else if (prevPayload == currPayload){
//                    std::cout << "Error! Same address! " << std::endl;
//                }
//                else {
//                
//                countLostEvts = currPayload - prevPayload - 1;
//                }
//                diffTs[countBotEvts] = unwrap.currentTime() - prevTs;
//                countBotEvts ++;
//                
//            }
//            prevPayload = currPayload;
//        
            int tempAE;
            tempAE = 0;
            // find ch - bit 20 put it to bit 21
            tempAE |= (*AE & 0x00100000) << 1;
            // find pol - bit 0
            tempAE |= (*AE & 0x00000001);
            // find x - bits 9-1 to 10-1
            tempAE |= (*AE & 0x000003FE);
            // find y - bits 17-10 to 20-11
            tempAE |= (*AE & 0x0003FC00) << 1;
            
            *AE = tempAE;
            
            //and then check the next two ints
            bend += 2;
  //      }
    }
    
    std::cout << "Evts: " << countTotEvts << ", Lost: " << countLostEvts <<", Time: " << unwrap.currentTime() - firstTs << std::endl;
    
    
    if(nBytesRead - bstart > 7) {
        emorph::vBottleMimic &vbm = portvBottle.prepare();
        vbm.setdata(data.data()+bstart, 8*((nBytesRead-bstart)/8));
        countAEs += (nBytesRead - bstart) / 8;
        vStamp.update();
        portvBottle.setEnvelope(vStamp);
        if(strict) portvBottle.writeStrict();
        else portvBottle.write();
    }
    if(portScope.getOutputCount() && countBotEvts)
    {
        yarp::os::Bottle &sbot = portScope.prepare();
        sbot.clear();
        int sum = 0;
        for (int i = 0; i < countBotEvts; i++){
            
            sum += diffTs[i];
            
        }
        
        double meanTs;
        meanTs = ((double)sum)/countBotEvts;
        double stdTs = 0;
        
        for (int i = 0; i < countBotEvts; i++){
            
            stdTs += pow(diffTs[i] - meanTs, 2.0);
        }
        
        stdTs /= countBotEvts;
        stdTs = sqrt(stdTs);
        
        sbot.addDouble(meanTs);
        sbot.addDouble(stdTs);
        
        scStamp.update();
        portScope.setEnvelope(scStamp);
        if(strict) portScope.writeStrict();
        else portScope.write();
    }
    
}

void device2yarp::threadRelease() {

    std::cout << "D2Y: has collected " << countAEs << " events from device"
              << std::endl;

    portvBottle.close();

}

bool  device2yarp::attachDeviceManager(deviceManager* devManager) {

    this->devManager = devManager;

    aerfx2_0DevManager* tempfx2manager = dynamic_cast<aerfx2_0DevManager*>(devManager);
    if(tempfx2manager)
        doChannelShift = false;

    aerDevManager * tempdevmanager = dynamic_cast<aerDevManager*>(devManager);
    if(tempdevmanager)
        clockScale = tempdevmanager->getTickToUs();

    return true;


}

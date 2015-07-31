//
//  yarp2device.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 24/07/15.
//
//

#include "iCub/yarp2device.h"
#include <unistd.h>

#define MAX_DATA_SIZE 512

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    flagStart = false;
    countAEs = 0;
    writtenAEs = 0;
}

bool yarp2device::open(std::string moduleName)
{
    //setFileDesc(devDesc);
    
    this->useCallback();
    
    fprintf(stdout,"opening port for receiving the events from yarp \n");
    
    std::string inPortName = moduleName + "/vBottle:i";
    return yarp::os::BufferedPort< emorph::vBottle >::open(inPortName);
    
}

/******************************************************************************/
void yarp2device::close()
{
    std::cout << "Y2D: received " << countAEs << " events from yarp" << std::endl;
    
    std::cout << "Y2D: written " << writtenAEs << " events to device"
    << std::endl;
    yarp::os::BufferedPort<emorph::vBottle>::close();
}

/******************************************************************************/
void yarp2device::interrupt()
{
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/******************************************************************************/
void yarp2device::onRead(emorph::vBottle &bot)
{
    
    emorph::vQueue q = bot.getAll();
    //deviceData.resize(q.size()*2);
    deviceData.resize(MAX_DATA_SIZE);
    countAEs += q.size();
    //std::cout<<"Y2D onRead - deviceData size: "<<deviceData.size()<<std::endl;
    
    // checks for empty or non valid queue????
    int i = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        
        int channel = aep->getChannel();
        int polarity = aep->getPolarity();
        int x = aep->getX();
        int y = aep->getY();
        int ts = aep->getStamp();
        
        // address
        int word0 = ((channel&0x01)<<15)|((y&0x7f)<<8)|((x&0x7f)<<1)|(polarity&0x01);
        
        // set intial timestamp to compute diff
        if (flagStart == false)
        {
            tsPrev = ts;
            flagStart = true;
        }
        // timestamp difference
        int word1 = (ts - tsPrev);
        
        if (tsPrev > ts)
        {
            word1 += emorph::vtsHelper::maxStamp();
        }
        
        word1 = 1;//(4 * word1);
        
        deviceData[i] = word1;   //timstamp
        deviceData[i+1] = word0; //data
        
        i += 2;
        tsPrev = ts;
        
        // the maximum size that can be written to the device is 512, if we hit this value we need to write and then start filling deviceData from i=0
        if (i == MAX_DATA_SIZE){
            i = 0;
            int devData = devManager->writeDevice(deviceData);
            //std::cout<<"Y2D write: writing to device"<<q.size()<< "events"<<std::endl;
            if (devData <= 0)
            {
                fprintf(stdout,"Y2D write: devData: %d",devData);
            }
            else
            {
                int wroteData = devData/(2*sizeof(unsigned int));
                writtenAEs += wroteData;
                if (wroteData != MAX_DATA_SIZE){
                    std::cout<<"Y2D mismatch - yarp events: "<<MAX_DATA_SIZE/2<<" wrote events:"<<wroteData<<std::endl;
                }
            }
            //            if (deviceData.size()>MAX_DATA_SIZE){
            //                std::cout<<"Y2D write - deviceData size: "<<deviceData.size()<<std::endl;
            //                i = 0;
            //                deviceData.resize(deviceData.size() - MAX_DATA_SIZE); //resize the deviceData to the correct amount of events left after the write, it shouldn't be necessary because the for loop iterates on the events queue
            //                std::cout<<"Y2D write - deviceData size: "<<deviceData.size()<<std::endl;
            //            }
            
        }
    }
    if (i>0){
        deviceData.resize(i);
        int devData = devManager->writeDevice(deviceData);
        //std::cout<<"Y2D write: writing to device"<<q.size()<< "events"<<std::endl;
        if (devData <= 0)
        {
            fprintf(stdout,"Y2D write: devData: %d",devData);
        }
        else
        {
            int wroteData = devData/(2*sizeof(unsigned int));
            writtenAEs += wroteData;
            if (wroteData != q.size()){
                std::cout<<"Y2D mismatch - yarp data: "<< i <<" wrote data:"<<wroteData<<std::endl;
            }
        }
    }
}
void  yarp2device::attachDeviceManager(deviceManager* devManager) {
    this->devManager = devManager;
    
}


//empty line to make gcc happy

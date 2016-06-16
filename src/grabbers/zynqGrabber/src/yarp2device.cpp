/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: chiara.bartolozzi@iit.it
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

#include "iCub/yarp2device.h"
//#include <unistd.h>

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    flagStart = false;
    countAEs = 0;
    writtenAEs = 0;
    clockScale = 1;
}

bool yarp2device::open(std::string moduleName)
{
    //setFileDesc(devDesc);

    this->useCallback();

    fprintf(stdout,"opening port for receiving the events from yarp \n");

    std::string inPortName = "/" + moduleName + "/vBottle:i";
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

/*
 void yarp2device::onRead(emorph::vBottle &bot)
 {

 emorph::vQueue q = bot.getAll();
 //deviceData.resize(q.size()*2);
 deviceData.resize(MAX_DATA_SIZE); // deviceData has TS and ADDRESS, its size is double the number of events
 // MAX_DATA_SIZE is the maximum number of words that we can write to the driver, that is 1024, corresponding to 512 events
 countAEs += q.size(); // counter for total number of events received from yarp port for the whole duration of the thread

 std::cout<<"Y2D onRead - events queue size: "<<q.size()<<std::endl;
 std::cout<<"Y2D onRead - deviceData size: "<<deviceData.size()<<std::endl;

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

 deviceData[i] = word1;   //timestamp
 deviceData[i+1] = word0; //data

 i += 2;
 tsPrev = ts;

 // the maximum size that can be written to the device is 512, if we hit this value we need to write and then start filling deviceData from i=0
 if (i == MAX_DATA_SIZE){ // i counts the elements of deviceData, that is double the amount of events
 i = 0;

 // writing to device
 int devData = devManager->writeDevice(deviceData); // devData is the number of bytes written by the device driver in the FIFO, if devData<deviceData.size() then we should send the remaining events that were dropped by the device driver

 //std::cout<<"Y2D write: writing to device"<<q.size()<< "events"<<std::endl;
 if (devData <= 0)
 {
 fprintf(stdout,"Y2D write max_data_size: devData: %d",devData);
 return;
 }
 else
 {
 int wroteData = devData/(2*sizeof(unsigned int)); // number of events written to the FIFO
 writtenAEs += wroteData;
 if (wroteData != MAX_DATA_SIZE){
 std::cout<<"Y2D mismatch - yarp events: "<<MAX_DATA_SIZE/2<<" wrote events:"<<wroteData<<std::endl;

 } else {
 return;
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
 std::cout<<"Y2D end cicle - yarp data: "<< i <<" wrote data:"<<wroteData<<std::endl;
 }
 }
 }
 }*/

void yarp2device::onRead(emorph::vBottle &bot)
{

    emorph::vQueue q = bot.getAll();
    deviceData.resize(q.size()*2);  // deviceData has TS and ADDRESS, its size is double the number of events
    countAEs += q.size(); // counter for total number of events received from yarp port for the whole duration of the thread

    //std::cout<<"Y2D onRead - events queue size: "<<q.size()<<std::endl;
    //std::cout<<"Y2D onRead - deviceData size/2: "<<deviceData.size()/2<<std::endl;

    // write events on the vector of unsigned int
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
            //std::cout<<"TS prev  :"<<tsPrev<<"us"<<std::endl;
            //std::cout<<"TS       :"<<ts<<"us"<<std::endl;
            std::cout<<"Delta TS :"<<ts - tsPrev<<"us"<<std::endl;

            //std::cout<<"Initial TS"<<ts<<"us"<<std::endl;
            tsPrev = ts;
            flagStart = true;
        }
        // timestamp difference
        int word1 = (ts - tsPrev);

        if (tsPrev > ts)
        {
            //std::cout<<"Wrap TS: ts      "<<ts<<"us"<<std::endl;
            //std::cout<<"Wrap TS: ts prev "<<tsPrev<<"us"<<std::endl;
            word1 += emorph::vtsHelper::maxStamp();

            //std::cout<<"Wrap TS: max     "<<emorph::vtsHelper::maxStamp()<<"us"<<std::endl;
            std::cout<<"--------------- Wrap TS: Delta TS new "<<word1<<"us--------------------"<<std::endl;
            word1 = 0;

        }

        word1 = word1 * clockScale;

        deviceData[i] = word1;   //timestamp
        deviceData[i+1] = word0; //data

        i += 2;
        tsPrev = ts;

    }
    flagStart = false; // workaround for long delays due to large delta ts across bottles
    // write to the device

    unsigned int wroteData = devManager->writeDevice(deviceData); // wroteData is the number of data written to the FIFO (double the amount of events)

    //    std::cout<<"Y2D write: writing to device"<<deviceData.size()<< "elements"<<std::endl;
    //    std::cout<<"Y2D write: wrote to device"<<wroteData<< "elements"<<std::endl;
    if (!wroteData)
    {
        std::cout<<"Y2D write: error writing to device"<<std::endl;
        return;
    }
    else
    {
        writtenAEs += wroteData/2; // written events
        if (wroteData != deviceData.size()){
            std::cout<<"Y2D mismatch - sent events: "<<deviceData.size()<<" wrote events:"<<wroteData<<std::endl;

        } else {
            return;
        }
    }
}

bool  yarp2device::attachDeviceManager(deviceManager* devManager) {

    aerDevManager* tempdm = dynamic_cast<aerDevManager*>(devManager);
    if(tempdm)
        clockScale = tempdm->getUsToTick();

    this->devManager = devManager;


    return true;



}


//empty line to make gcc happy

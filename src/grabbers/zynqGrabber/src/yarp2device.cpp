//
//  yarp2device.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 24/07/15.
//
//

#include "iCub/yarp2device.h"
#include <unistd.h>

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    flagStart = false;
    countAEs = 0;
}

bool yarp2device::open(int devDesc, std::string moduleName)
{
    setFileDesc(devDesc);

    this->useCallback();
    
    fprintf(stdout,"opening port for receiving the events from yarp \n");
    
    std::string inPortName = moduleName + "/vBottle:i";
    return yarp::os::BufferedPort< emorph::vBottle >::open(inPortName);

}

/******************************************************************************/
void yarp2device::close()
{
    std::cout << "Y2D: received " << countAEs << " events." << std::endl;
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
    deviceData.resize(q.size()*2);
    countAEs += q.size();

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
        
        word1 = (4 * word1);
        
        deviceData[i] = word1;   //timstamp
        deviceData[i+1] = word0; //data

        i += 2;
        tsPrev = ts;
        
    }
    

    int devData = ::write(devDesc, (char *)deviceData.data(), 2*q.size()*sizeof(unsigned int));

    
}

void yarp2device::setFileDesc(int devDesc)
{
    this->devDesc = devDesc;
}


//empty line to make gcc happy

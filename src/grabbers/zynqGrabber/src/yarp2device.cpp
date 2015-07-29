//
//  yarp2device.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 24/07/15.
//
//

#include "iCub/yarp2device.h"
#include <unistd.h>

#define __DEBUG__
/******************************************************************************/
//yarp2device
/******************************************************************************/

bool yarp2device::open(int file_desc, std::string moduleName)
{
    this->useCallback();
    
    fprintf(stdout,"opening port for receiving the events from yarp \n");
    
    std::string inPortName = moduleName + "/vBottle:i";
    bool success = yarp::os::BufferedPort< emorph::vBottle >::open(inPortName);

    if (!success)
    {
        fprintf(stdout,"unable to open port %s\n",inPortName.c_str());
    }
    // flag to initialise timestamp
    flagStart = false;
    setFileDesc(file_desc); 
    countAEs = 0;    
    return success;
}

/******************************************************************************/
void yarp2device::close()
{
	fprintf(stdout,"yarp to device total received events: %d",countAEs);
    BufferedPort<emorph::vBottle >::close();
}

/******************************************************************************/
void yarp2device::interrupt()
{
    BufferedPort< emorph::vBottle >::interrupt();
}

/******************************************************************************/
void yarp2device::onRead(emorph::vBottle &bot)
{
#ifdef __DEBUG__
//	fprintf(stdout,"y2d: got data from yarp\n");
#endif
    //create event queue and iterator
    emorph::vQueue q = bot.getAll();
    emorph::vQueue::iterator qi;
    
    // encode the event sent by spinnaker
    int           word0, word1;
    int           polarity;
    int           channel;
    int           x,y;
    int           i;
    long int      ts;
    
    int devData;
    
    i = 0;
    
    //write data to device
    
    //unsigned int *TXDATA;
    //TXDATA = (unsigned int *) calloc(q.size(), sizeof(unsigned int));
    deviceData.resize(q.size()*2);
    countAEs += q.size();
    // checks for empty or non valid queue????
    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        
        channel = aep->getChannel();
        polarity = aep->getPolarity();
        x = aep->getX();
        y = aep->getY();
        ts = aep->getStamp();
#ifdef __DEBUG__        
//        fprintf(stdout, "x:%d y:%d ts:%ld\n", x,  y, ts);
#endif
  	// address
        word0 = ((channel&0x01)<<15)|((y&0x7f)<<8)|((x&0x7f)<<1)|(polarity&0x01);
        deviceData[i] = word0;
        
        // set intial timestamp to compute diff
        if (flagStart == false)
        {
            tsPrev = ts;
            flagStart = true;
        }
        // timestamp difference
        word1 = (ts - tsPrev);
        
        if (tsPrev > ts)
        {
           word1 += emorph::vtsHelper::maxStamp();
        }
        
        word1 = word1 & 0x00ffffff;
        
        deviceData[i+1] = word1;
        i = i+2;
        tsPrev = ts;
        
    }
    
    // write to device
    //    ::write(file_desc, (char *)TXDATA, q.size()*sizeof(unsigned int));
    devData = ::write(file_desc, (char *)deviceData.data(), q.size()*sizeof(unsigned int));
    
#ifdef __DEBUG__
    fprintf(stdout,"wrote %d events to device\n",devData);
    //for (i=0; i<q.size(); i++)
    // fprintf(stderr, "Sent data %d : 0x%08x  -  %10d\n", i, TXDATA[i], TXDATA[i]);
    //  fprintf(stderr, "Sent data %d : 0x%08x  -  %10d\n", i, deviceData[i], deviceData[i]);
#endif
    
    //free(TXDATA);
    
}

void yarp2device::setFileDesc(int file_desc)
{
    this->file_desc = file_desc;
}


//empty line to make gcc happy

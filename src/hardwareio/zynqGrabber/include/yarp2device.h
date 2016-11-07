//
//  yarp2device.h
//  eventdriven
//
//  Created by Chiara Bartolozzi on 24/07/15.
//
//

#ifndef __eventdriven__yarp2device__
#define __eventdriven__yarp2device__

#include <stdio.h>


#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <ctime>
#include <string>
#include "deviceManager.h"


/******************************************************************************/
//yarp2device
/******************************************************************************/
class yarp2device : public yarp::os::BufferedPort<eventdriven::vBottle>
{
    int           devDesc;                    // file descriptor for opening device /dev/spinn2neu
    bool          flagStart;                    // flag to check if this is the first time we run the callback,
    // used to initialise the timestamp for computing the difference
    int      tsPrev;                       // FIRST TIMESTAMP TO COMPUTE DIFF
    int           countAEs;
    int           writtenAEs;
    double clockScale;

    deviceManager* devManager;

    void setFileDesc(int devDesc);

    //vector to store the masked address events to the device
    std::vector<unsigned int> deviceData;


public:

    yarp2device();
    virtual    bool    open(std::string moduleName);
    bool    init();
    void    close();
    void    onRead(eventdriven::vBottle &bot);
    void    interrupt();
    bool    attachDeviceManager(deviceManager* devManager);


};


#endif /* defined(__eventdriven__yarp2device__) */

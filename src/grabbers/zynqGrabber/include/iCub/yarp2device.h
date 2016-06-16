//
//  yarp2device.h
//  eMorph
//
//  Created by Chiara Bartolozzi on 24/07/15.
//
//

#ifndef __eMorph__yarp2device__
#define __eMorph__yarp2device__

#include <stdio.h>


#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <ctime>
#include <string>
#include "iCub/deviceManager.h"


/******************************************************************************/
//yarp2device
/******************************************************************************/
class yarp2device : public yarp::os::BufferedPort<emorph::vBottle>
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
    void    onRead(emorph::vBottle &bot);
    void    interrupt();
    bool    attachDeviceManager(deviceManager* devManager);


};


#endif /* defined(__eMorph__yarp2device__) */

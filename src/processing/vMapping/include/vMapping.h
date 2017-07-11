//
// Created by miacono on 11/07/17.
//

#ifndef ICUB_EVENT_DRIVEN_VMAPPING_H
#define ICUB_EVENT_DRIVEN_VMAPPING_H

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>

class vMappingModule : public yarp::os::RFModule {
private :

public :
    
    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the modulereturn
    
    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
    
};

#endif //ICUB_EVENT_DRIVEN_VMAPPING_H

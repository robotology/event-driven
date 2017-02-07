/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

/// \defgroup HardwareIO HardwareIO
/// \defgroup spinterface spinterface
/// \ingroup HardwareIO
/// \brief reads and writes events to SpiNNaker using EIEIO protocol

#ifndef __ICUB_SPINTERFACE__
#define __ICUB_SPINTERFACE__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>

#include <EIEIOReceiver.h>
#include <EIEIOSender.h>

class YARPspinI : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    spinnio::EIEIOSender   *spinSender;

    //for helping with timestamp wrap around
    ev::vtsHelper unwrapper;

    int downsamplefactor;
    int height;
    int width;

public:

    YARPspinI();


    bool    open(const std::string &name);
    void    close();
    void    interrupt();
    void    attachEIEIOSender(spinnio::EIEIOSender*);

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);

};

class YARPspinO : public yarp::os::RateThread
{
private:

    spinnio::EIEIOReceiver   *spinReceiver;
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;

    int width;
    int height;
    int downsamplefactor;

public:

    YARPspinO();

    bool initThread(std::string moduleName, spinnio::EIEIOReceiver *spinReceiverPtr);
    void run();
    void threadRelease();
};

class YARPspinIO : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    spinnio::EIEIOSender   *spinSender;
    spinnio::EIEIOReceiver   *spinReceiver;
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;

    int downsamplefactor;
    int height;
    int width;

public:

    YARPspinIO();

    bool    open(const std::string &name);
    void    close();
    void    interrupt();
    void    attachEIEIOmodules(spinnio::EIEIOSender* spinSenderPtr, spinnio::EIEIOReceiver *spinReceiverPtr);

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &inbottle);

};

class vSpinInterface : public yarp::os::RFModule
{

    //the event bottle input and output handler

    YARPspinI      inputManager;
    YARPspinO      outputManager;

    void initSpin(int spinPort, int sendPort, std::string ip,
                  std::string databasefile, spinnio::EIEIOReceiver **eir,
                                  spinnio::EIEIOSender **eis);

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy

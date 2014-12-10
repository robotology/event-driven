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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/os/RpcClient.h>

#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventCodec.h>

#include <time.h>
#include <string>

class EventBottleManager : public yarp::os::BufferedPort<eventBottle>
{
private:

    std::string 				                moduleName;         //string containing module name
    std::string 				                inPortName;        	//string containing events input port name
    std::string 				                leftPortName;	    //string containing image output port name
    std::string 				                rightPortName;	    //string containing image output port name
    std::string 				                outPortName;	    //string containing events output port name

    yarp::os::BufferedPort<eventBottle>         inPort;             //input port for the eventBottles from aexGrabber or dataSetPlayer
    yarp::os::Port                              leftPort;           //output port for the image left
    yarp::os::Port                              rightPort;          //output port for the image right
    yarp::os::BufferedPort<eventBottle>         outPort;            //output port for the eventBottle with the new events computed by the module

    std::string                                 fileName;
    int                                         alphaShape;
    int                                         alphaPos;

public:
    /**
     * constructor
     * @param moduleName is passed to the thread in order to initialise all the ports correctly (default yuvProc)
     */
    EventBottleManager( const std::string &moduleName );
    ~EventBottleManager();

    bool    open();
    bool    init();
    void    close();
    void    onRead(eventBottle &bot);
    void    interrupt();

  
};

class EventClustering:public yarp::os::RFModule
{
    /* module parameters */
    std::string             moduleName;
    std::string             rpcPortName;
    yarp::os::RpcServer     rpcPort;

    std::string                                 fileName;
    int                                         alphaShape;
    int                                         alphaPos;

    /* pointer to a new manager */
    EventBottleManager      *eventBottleManager;
    bool                    closing;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();

  /**
     * @brief function that sets the output file name for saving the clustered data
     */
    void setFileName(std::string value) {fileName = value; }; 
 
    /**
     * @brief function that sets the alpha shape
     */
    void setAlphaShape(int value) {alphaShape = value; }; 
 
    /**
     * @brief function that sets the alpha shape
     */
    void setAlphaPos(int value) {alphaPos = value; }; 



};


#endif
//empty line to make gcc happy
